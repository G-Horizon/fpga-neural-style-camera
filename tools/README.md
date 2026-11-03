# 模型与语音准备工具

Python文件位于FPGA工程之外。下述校验工具只读文件，不访问USB或调用烧录器。

## 当前正式单板：ABI3／128路

正式参数已经包含在`fpga/single/profiles`。每款为247个1024位逻辑权重tile，按低512位、再高512位顺序形成494个512位物理tile；403个64位配置项；完整profile为2179个128位beat／34,864字节。风格ID是1/2/3，Flash模型区起始0x710000。128路计算和512位物理加载宽度是两个不同概念。

`profile_assets.py`取自实际使用的ABI3只读验证器，默认指向上述正式单板参数。主示例：

```bash
python tools/profile_assets.py --project fpga/single
```

它核对原有header、ABI3、CRC、MEM表示、BIN/HEX、Flash槽位、编译时CRC/zero和loader的ABI判断。当前检查结论见仓库文档／本次整理验证记录；这个命令不会重新生成或覆盖参数。`--binary-only`仅用于明确区分BIN与programmer HEX时的诊断，不表示忽略HEX后就允许烧录。

## 训练、浮点和整数参考

- `d64_net.py`：折叠网络算子语义；`--contract`查看padding、激活、转置卷积约定。兼容原folded_model/folded_spec以及最终export的model/spec（norm=none）容器，不改变计算。
- `d64_quant.py`：从折叠checkpoint及校准图派生整数格式、权重与定点参数。
- `d64_int_ref.py`：逐层整数参考与质量计算；按需提供`--checkpoint --data-root --split --input`，源码不带原图golden数组。

例如重新派生量化参数（不改正式FPGA参数）：

```bash
python tools/d64_quant.py --checkpoint models/weights/vangogh/model_fp32.pth --data-root training/data/coco_val2017 --split training/data/splits/val_fixed.txt --write --json models/derived/vangogh/derived_contract.json --npz models/derived/vangogh/derived_weights.npz
```

## 可选自备语音模板

`voice_features.py`、`make_voice_templates.py`保留通用特征与替换模板入口。正式ESP32源码保留原口令模板，原始本人录音未复制。分别录制四个口令的单声道PCM16、16kHz WAV，每文件一个口令并保留短静音：

```bash
python tools/make_voice_templates.py --vangogh YOUR_VANGOGH.wav --ukiyoe YOUR_UKIYOE.wav --inkwash YOUR_INKWASH.wav --next YOUR_NEXT.wav --output voice_templates.h
```

更换说话人时，将对应voice_local.c内training_lengths和templates原数组替换为包含该头文件，再用新口令和负例验证；默认交付没有替换，保持原正式行为。C模板头不是FPGA模板ROM，该方法为特定说话人近距离原型，不是通用ASR。
