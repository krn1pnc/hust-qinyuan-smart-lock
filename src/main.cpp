#include "config.h"

#include "hal/rtc_cntl_ll.h"
#include "rom/rtc.h"

#include <Arduino.h>
#include <MFRC522.h>
#include <SPI.h>
#include <Servo.h>

FORCE_INLINE_ATTR uint32_t clk_ll_rtc_slow_load_cal(void) {
    return REG_READ(RTC_SLOW_CLK_CAL_REG);
}

FORCE_INLINE_ATTR void gpio_output_enable(uint8_t pin) {
    GPIO.enable_w1ts.enable_w1ts = 1u << pin;
}

FORCE_INLINE_ATTR uint32_t digital_read(uint8_t pin) {
    return GPIO.in.data >> pin & 1;
}

FORCE_INLINE_ATTR void digital_write(uint8_t pin, bool value) {
    (value) ? (GPIO.out_w1ts.out_w1ts = 1u << pin) : (GPIO.out_w1tc.out_w1tc = 1u << pin);
}

// Hardware SPI is not initialized in wake stub.
uint8_t RTC_IRAM_ATTR ws_spi_transfer(uint8_t data) {
    digital_write(MFRC_SCK, 0);
    for (uint8_t i = 0; i < 8; i++) {
        digital_write(MFRC_MOSI, data >> 7 & 1);
        digital_write(MFRC_SCK, 1);
        data = data << 1 | digital_read(MFRC_MISO);
        digital_write(MFRC_SCK, 0);
    }
    return data;
}

uint8_t RTC_IRAM_ATTR ws_pcd_read_register(MFRC522::PCD_Register reg) {
    digital_write(MFRC_CS, 0);
    ws_spi_transfer(1u << 7 | reg);
    uint8_t res = ws_spi_transfer(0);
    digital_write(MFRC_CS, 1);
    return res;
}

void RTC_IRAM_ATTR ws_pcd_write_register(MFRC522::PCD_Register reg, uint8_t value) {
    digital_write(MFRC_CS, 0);
    ws_spi_transfer(reg);
    ws_spi_transfer(value);
    digital_write(MFRC_CS, 1);
}

#define MFRC_REGISTER_READ_TIME 7 // us, used to calculate timeouts

bool RTC_IRAM_ATTR ws_mfrc522_fastdetect() {
    ws_pcd_write_register(MFRC522::CollReg, 0);
    ws_pcd_write_register(MFRC522::ComIrqReg, 0x7F);
    ws_pcd_write_register(MFRC522::FIFOLevelReg, 0x80);
    ws_pcd_write_register(MFRC522::FIFODataReg, MFRC522::PICC_CMD_REQA);
    ws_pcd_write_register(MFRC522::BitFramingReg, 7);
    ws_pcd_write_register(MFRC522::CommandReg, MFRC522::PCD_Transceive);
    ws_pcd_write_register(MFRC522::BitFramingReg, 0x80 | 7);
    // 50ms timeout. Much longer than required, but should not ever be used at all => does not matter.
    for (uint32_t timeout = 0; timeout < 50000 / MFRC_REGISTER_READ_TIME; timeout++) {
        uint8_t irq_flags = ws_pcd_read_register(MFRC522::ComIrqReg);
        if (irq_flags & 0x30) return true;  // RxIrq || IdleIrq
        if (irq_flags & 0x01) return false; // TimeoutIrq
    }
    // should never get here
    return false;
}

void RTC_IRAM_ATTR ws_mfrc522_fastreset() {
    gpio_output_enable(MFRC_CS);
    gpio_output_enable(MFRC_SCK);
    gpio_output_enable(MFRC_MOSI);

    // Reset & Wakeup
    ws_pcd_write_register(MFRC522::CommandReg, MFRC522::PCD_SoftReset);
    while ((ws_pcd_read_register(MFRC522::CommandReg) & (1 << 4))) {
        // ~ 200us wake up time from power down
    }

    // Init timer
    ws_pcd_write_register(MFRC522::TModeReg, 0x80);    // TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
    ws_pcd_write_register(MFRC522::TPrescalerReg, 67); // 13.56MHz / (2 * 67 + 1) = ~100kHz => 10μs
    ws_pcd_write_register(MFRC522::TReloadRegH, 0);    // Reload timer with 30, ie 0.3ms before timeout.
    ws_pcd_write_register(MFRC522::TReloadRegL, 30);

    ws_pcd_write_register(MFRC522::TxASKReg, 0x40);     // Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
    ws_pcd_write_register(MFRC522::ModeReg, 0x3D);      // Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
    ws_pcd_write_register(MFRC522::TxControlReg, 0x83); // Antenna on
}

void RTC_IRAM_ATTR ws_mfrc522_soft_power_down() {
    ws_pcd_write_register(MFRC522::CommandReg, MFRC522::PCD_NoCmdChange | 0x10);
}

void RTC_IRAM_ATTR esp_wake_stub_set_wakeup_time(uint64_t time_in_us) {
    uint64_t rtc_count_delta = time_in_us * (1 << RTC_CLK_CAL_FRACT) / clk_ll_rtc_slow_load_cal();
    uint64_t rtc_curr_count = rtc_cntl_ll_get_rtc_time();
    rtc_cntl_ll_set_wakeup_timer(rtc_curr_count + rtc_count_delta);
}

void RTC_IRAM_ATTR esp_wake_stub_sleep(esp_deep_sleep_wake_stub_fn_t new_stub) {
    REG_WRITE(RTC_ENTRY_ADDR_REG, (uint32_t)new_stub);
    set_rtc_memory_crc();
    rtc_cntl_ll_sleep_enable();
    asm volatile("wfi\n");
}

void RTC_IRAM_ATTR wake_stub(void) {
    ws_mfrc522_fastreset();
    esp_rom_delay_us(1000);
    if (ws_mfrc522_fastdetect()) {
        // Card detected => Wake up system
        esp_default_wake_deep_sleep();
    } else {
        // No card = > go to sleep again
        ws_mfrc522_soft_power_down();
        esp_wake_stub_set_wakeup_time(SLEEP_TIME);
        esp_wake_stub_sleep(&wake_stub);
    }
}

MFRC522 mfrc522(MFRC_CS, MFRC_RST);
Servo mg996r;

bool is_authorized_card(const MFRC522::Uid &uid) {
    if (uid.sak != CARD_SAK || uid.size != UID_SIZE) return false;

    for (uint32_t i = 0; i < CARD_NUM; i++) {
        bool match = true;
        for (uint32_t j = 0; j < UID_SIZE; j++) {
            if (uid.uidByte[j] != authorized_cards[i][j]) {
                match = false;
                break;
            }
        }
        if (match) return true;
    }
    return false;
}

void setup() {
    SPI.begin(MFRC_SCK, MFRC_MISO, MFRC_MOSI, MFRC_CS);
    mg996r.attach(SERVO_PWM);

    if (mfrc522.PICC_ReadCardSerial()) {
        if (is_authorized_card(mfrc522.uid)) {
            mg996r.write(180), delay(3000);
            mg996r.write(0), delay(1500);
        }
    }

    esp_sleep_enable_timer_wakeup(SLEEP_TIME);
    esp_set_deep_sleep_wake_stub(&wake_stub);
    esp_deep_sleep_start();
}

void loop() {}