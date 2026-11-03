# FPGA 源码

## 三个工程入口

| 工程 | Efinity XML | 对应行为与状态 |
|---|---|---|
| 单板 | `single/board/ddr3/final_camera_style.xml` | 原有约30帧单板；三风格、1/2高清细节融合、三布局、倒装修正、三行帧率和原SPI手机／遥控接口。 |
| 以太网双板 A | `dual/board_a/board/ddr3/d2e_board_a.xml` | 对应正式 A 固件标识 `f746`：1080p60摄像头输入、编码计算、以太网发送／接收、原图引导融合与HDMI输出；包含180度修正和三行帧率。 |
| 以太网双板 B | `dual/board_b/board/ddr3/d2e_board_b_full5.xml` | 对应正式 B 固件标识 `97b`：全部五个残差块与解码。 |

两板之间使用以太网线。摄像头及最终HDMI显示接 A 板，机械堆叠时 A 为上层板。不要从机械上下位置推断代码中的角色名，按摄像头与输出接口确认。

