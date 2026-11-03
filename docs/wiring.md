# 接线说明与版本区别

## 当前单板

摄像头通过软排线接Ti60的J4，HDMI屏幕接该板HDMI TX。J7通过转接板引出2.54mm J28排针。

| FPGA转接板 | ESP32-S3 | 信号 |
|---|---|---|
| J28-39 | GPIO4 | READY，FPGA→S3 |
| J28-37 | GPIO13 | MISO，FPGA→S3 |
| J28-32 | GPIO12 | SCLK，S3→FPGA |
| J28-34 | GPIO11 | MOSI，S3→FPGA |
| J28-38 | GPIO10 | CS_N，S3→FPGA |
| J28-36（地网络） | 真正GND | 共同参考地 |

| SPH0645 | ESP32-S3白色单板固件 |
|---|---|
| 3V | 3V3 |
| GND | 真正GND |
| BCLK | GPIO16 |
| LRCL / WS | GPIO17 |
| DOUT | GPIO18 |
| SEL | 真正GND |

## 当前以太网双板视频主链

1. 摄像头接A板J4；A、B的RJ45口用网线直连。
2. 屏幕接A板HDMI TX。当前上下堆叠模型里A在上层，HDMI从上层板引出。
3. A负责编码器、同帧高清原图、结果接收与融合显示；B负责全部五个残差块和解码器。
4. 各板独立USB数据线及供电。

当前以太网是板间专用传输，不依赖电脑静态IP或S3热点。真实新帧率需检查A采集/发送、B接收/计算/回传、A接收/发布/不同FID扫描各阶段，而非只看网口灯或HDMI扫描。
