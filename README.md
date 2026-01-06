# hust-qinyuan-smart-lock

Inspired by [vaaandark/smart-door](https://github.com/vaaandark/smart-door)．

使用 esp32c3，每秒检测一次卡片，若未出现卡片则进入休眠，降低了功耗，从而使得设备可由干电池供电．

配置文件应位于 `include/config.h`．默认配置示例位于 `include/config.default.h`，其中包含推荐的接线方式．

## 参考

https://github.com/vaaandark/smart-door

https://github.com/miguelbalboa/rfid/issues/269#issuecomment-922404208

https://gist.github.com/igrr/54f7fbe0513ac14e1aea3fd7fbecfeab

https://github.com/espressif/esp-idf
