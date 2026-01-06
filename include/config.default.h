#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

#define SLEEP_TIME (1 * 1000 * 1000)

#define MFRC_SCK 2
#define MFRC_MOSI 3
#define MFRC_MISO 10
#define MFRC_CS 8
#define MFRC_RST 6

#define SERVO_PWM 13

#define CARD_NUM 0
#define UID_SIZE 4
#define CARD_SAK 0x28
static const byte authorized_cards[CARD_NUM][UID_SIZE] = {};

#endif
