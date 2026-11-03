# 三风格模型与配置

`specs/{vangogh,ukiyoe,inkwash}`包含最终轻量模型逐层结构、初始校准参数、MAC统计和折叠检查。`weights/<style>/model_fp32.pth`是对应最终折叠的checkpoint，三款合计约0.4MB。

- 结构：3→8→16→32、5个瓶颈残差块32→16→32、32→16→8→3；归一化折叠，转置卷积上采样。
- JSON规范输入为256×256；FPGA视频实际是256×144，HDMI输出1920×1080。
- `quant_params.json`是24图初始校准导出记录；最终部署契约由`tools/d64_quant.py`派生：INT8逐输出通道权重、普通激活8位、残差主干共享10位码值格式，以及按通道定点乘数、算术移位、规定舍入和饱和。

可以按`training/README.md`重新训练和导出，也可直接用这里的折叠checkpoint作校准/整数参考。
