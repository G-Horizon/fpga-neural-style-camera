"""Train a Fast Neural Style teacher compatible with tinystyle/models/teacher.py."""
import argparse
import os
import random

import torch
import torch.nn as nn
import torch.nn.functional as F
from PIL import Image
from torch.utils.data import DataLoader, Dataset
from torchvision import models, transforms

import sys; sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from fast_style_transfer import TransformerNet


class ContentDataset(Dataset):
    def __init__(self, root, split, crop):
        with open(split, encoding="utf-8") as handle:
            self.names = [line.strip() for line in handle if line.strip()]
        self.root = root
        self.transform = transforms.Compose([
            transforms.RandomResizedCrop(crop, scale=(0.6, 1.0), ratio=(0.8, 1.25)),
            transforms.RandomHorizontalFlip(),
            transforms.ToTensor(),
        ])

    def __len__(self):
        return len(self.names)

    def __getitem__(self, index):
        path = os.path.join(self.root, self.names[index])
        return self.transform(Image.open(path).convert("RGB")) * 255.0


class VGGFeatures(nn.Module):
    taps = {3: "relu1_2", 8: "relu2_2", 15: "relu3_3", 22: "relu4_3"}

    def __init__(self):
        super().__init__()
        self.features = models.vgg16(weights=models.VGG16_Weights.IMAGENET1K_V1).features[:23]
        self.features.eval()
        for parameter in self.features.parameters():
            parameter.requires_grad_(False)

    def forward(self, x):
        values = {}
        for index, layer in enumerate(self.features):
            x = layer(x)
            if index in self.taps:
                values[self.taps[index]] = x
        return values


def imagenet_input(x):
    mean = x.new_tensor([0.485, 0.456, 0.406])[None, :, None, None]
    std = x.new_tensor([0.229, 0.224, 0.225])[None, :, None, None]
    return (x / 255.0 - mean) / std


def gram(x):
    n, channels, height, width = x.shape
    flat = x.reshape(n, channels, height * width)
    return flat.bmm(flat.transpose(1, 2)) / (channels * height * width)


def load_style(path, crop, device):
    image = Image.open(path).convert("RGB")
    resize = transforms.Resize(crop, antialias=True)
    tensor = transforms.ToTensor()(resize(image)).unsqueeze(0) * 255.0
    return tensor.to(device)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--style", required=True)
    parser.add_argument("--data-root", default="data/coco_val2017")
    parser.add_argument("--train-list", default="data/splits/train.txt")
    parser.add_argument("--out", required=True)
    parser.add_argument("--steps", type=int, default=5000)
    parser.add_argument("--batch-size", type=int, default=4)
    parser.add_argument("--crop", type=int, default=256)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--style-weight", type=float, default=5e5)
    parser.add_argument("--content-weight", type=float, default=1.0)
    parser.add_argument("--log-every", type=int, default=100)
    parser.add_argument("--seed", type=int, default=1234)
    args = parser.parse_args()

    random.seed(args.seed)
    torch.manual_seed(args.seed)
    torch.cuda.manual_seed_all(args.seed)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    dataset = ContentDataset(args.data_root, args.train_list, args.crop)
    loader = DataLoader(dataset, batch_size=args.batch_size, shuffle=True, drop_last=True,
                        num_workers=2, pin_memory=True, persistent_workers=True)
    iterator = iter(loader)
    model = TransformerNet().to(device).train()
    vgg = VGGFeatures().to(device).eval()
    with torch.no_grad():
        style_features = vgg(imagenet_input(load_style(args.style, args.crop, device)))
        style_grams = {name: gram(value).detach() for name, value in style_features.items()}
    optimizer = torch.optim.Adam(model.parameters(), lr=args.lr)
    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    for step in range(1, args.steps + 1):
        try:
            content = next(iterator)
        except StopIteration:
            iterator = iter(loader)
            content = next(iterator)
        content = content.to(device, non_blocking=True)
        with torch.no_grad():
            content_features = vgg(imagenet_input(content))["relu2_2"]
        output = model(content)
        output_features = vgg(imagenet_input(output))
        content_loss = F.mse_loss(output_features["relu2_2"], content_features)
        style_loss = sum(F.mse_loss(gram(output_features[name]), target.expand(content.shape[0], -1, -1))
                         for name, target in style_grams.items())
        total = args.content_weight * content_loss + args.style_weight * style_loss
        optimizer.zero_grad(set_to_none=True)
        total.backward()
        torch.nn.utils.clip_grad_norm_(model.parameters(), 5.0)
        optimizer.step()
        if step == 1 or step % args.log_every == 0 or step == args.steps:
            print(f"step={step}/{args.steps} total={total.item():.4f} content={content_loss.item():.4f} style={style_loss.item():.8f}", flush=True)
    model.eval()
    torch.save(model.state_dict(), args.out)
    print(f"saved {args.out}", flush=True)


if __name__ == "__main__":
    main()
