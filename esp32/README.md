# ESP32-S3与浏览器界面

`single/{esp32,mobile,remote}`保留正式单板白色竖屏720p手机页面与电脑遥控面板。手机接收FPGA输出的256×144风格图再上采样，不在手机运行CNN，不用原图做融合。

`dual/{esp32,mobile,remote}`是双板外设迁移源码，单独保存UART/SPI、会话、帧池与主机离线测试。

## 构建

目标ESP32-S3-WROOM-1-N16R8，原构建使用ESP-IDF 6.0.1。先激活官方export环境，使Python/CMake/Ninja/编译器进入PATH。

仓库根目录执行单板正式配置：

```bash
python esp32/tools/build.py --project esp32/single/esp32 -- -D SDKCONFIG=sdkconfig.mobile720-white build
```

也可在单板esp32目录直接执行`idf.py -D SDKCONFIG=sdkconfig.mobile720-white menuconfig`或`build`。双板迁移配置为`esp32/dual/esp32`下的`sdkconfig.physical-voice-fpga`；`sdkconfig.physical-voice-s3`为S3本地语音选择，`sdkconfig.mock-voice-fpga`为模拟源选择。

演示热点Ti60-Style-Camera，演示密码ti60style，页面http://192.168.4.1/，电脑遥控http://192.168.4.1/debug；使用者可在menuconfig修改。引脚与FPGA协议需匹配接线文档。

两个`voice_local.c`及其派生口令模板、配置与正式源保持一致，没有改成空模板或关闭识别。原始个人语音录音不随仓库复制。需要换说话人可参考`tools/README.md`。

## 离线验证

已安装GCC和Node.js时，可在`esp32/dual/esp32`执行`python tools/verify_offline.py`；
