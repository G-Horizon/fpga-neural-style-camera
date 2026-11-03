# 三风格教师训练与学生蒸馏

精选最终梵高、浮世绘、水墨模型所需代码。训练和固定验证裁剪是256×256；FPGA视频网络实际输入256×144，MAC/质量指标须注明尺寸。学生结构为3→8→16→32、5个32→16→32瓶颈残差块、32→16→8→3，两层上采样为转置卷积。归一化在导出时折叠进卷积。

教师模型结构参考PyTorch examples的fast_neural_style。

## 准备

在`training`目录运行。安装`requirements.txt`依赖；CUDA版PyTorch按官方安装说明与设备选择。

自行准备内容图片到`data/coco_val2017`。`data/splits/train.txt`和`val_fixed.txt`只含COCO文件名（训练4976张、固定验证24张），不含图片。数据来源：https://cocodataset.org/ 。VGG缓存和原始风格图不随源码复制。

## 训练、蒸馏与导出

以梵高为例；另两款使用对应配置和教师名重复。三款教师已提供在`../models/teachers/`，可直接从学生蒸馏开始。若要重新训练教师，先自行准备`styles/vangogh.png`，并将新教师输出到不同文件名，避免覆盖当前配套版本。

```bash
cd training
python scripts/train_teacher_style.py --style styles/vangogh.png --out teachers/vangogh_teacher_new.pth --steps 5000
# 可选重训教师后，先把对应配置teacher.checkpoint改成teachers/vangogh_teacher_new.pth；否则直接使用已带的配套教师。
python -m tinystyle.cli train -c configs/student_d64_vangogh_affine.yaml --max-steps 12000
python -m tinystyle.cli export -c configs/student_d64_vangogh_affine.yaml --checkpoint runs/YOUR_RUN/checkpoints/best.pth -o export/vangogh
```

原训练记录为各教师5000步、各学生12000步，并选验证L1最小checkpoint。VGG16使用torchvision官方ImageNet权重作感知损失，首次使用可能下载。`export`输出折叠模型、结构、折叠检查、MAC及初始校准报告；最终FPGA整数格式还需`tools/d64_quant.py`派生。

三款最终折叠学生权重已放`../models/weights/<style>/model_fp32.pth`，可以跳过重训直接作整数参考。三款配套教师已在`../models/teachers/`，配置自动引用。

## 派生FPGA整数模型

回到仓库根目录，以已带的梵高权重为例：

```bash
python tools/d64_quant.py --checkpoint models/weights/vangogh/model_fp32.pth --data-root training/data/coco_val2017 --split training/data/splits/val_fixed.txt --write --json models/derived/vangogh/derived_contract.json --npz models/derived/vangogh/derived_weights.npz
```

当前正式单板ABI3参数已在`../fpga/single/profiles`；`tools/README.md`提供真实校验入口，并将旧ABI1/ABI2生成器单独标注，不能用旧生成器覆盖正式ABI3。大数据集、VGG缓存、历史Candy权重、实验输出和原始个人语音不在此目录。
