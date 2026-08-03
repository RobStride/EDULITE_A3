#!/usr/bin/env python3
"""在上位机截图上用红框标注上电前检查步骤，生成 image_annotated.png"""
from PIL import Image, ImageDraw, ImageFont

SRC = "image.png"
DST = "image_annotated.png"
FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Black.ttc"

im = Image.open(SRC).convert("RGB")
w, h = im.size
s = w / 1024.0  # 标注坐标按 1024 宽基准估算，等比缩放
draw = ImageDraw.Draw(im)
font = ImageFont.truetype(FONT, int(20 * s))

RED = (230, 40, 40)
LW = max(3, int(3 * s))

marks = [
    # (框 x1,y1,x2,y2 基准坐标, 标签文字, 标签放置位置)
    ((512, 248, 604, 284), "1. 扫描电机", "left"),
    ((510, 452, 650, 484), "2. 校验 / 一键设置 ZERO_STA", "left"),
    ((886, 506, 1020, 538), "3. 全部设零（检查零点）", "left"),
]

for (x1, y1, x2, y2), label, side in marks:
    box = tuple(int(v * s) for v in (x1, y1, x2, y2))
    draw.rectangle(box, outline=RED, width=LW)

    tb = draw.textbbox((0, 0), label, font=font)
    tw, th = tb[2] - tb[0], tb[3] - tb[1]
    pad = int(6 * s)
    lx = box[0] - tw - int(24 * s)
    ly = (box[1] + box[3]) // 2 - th // 2 - pad
    # 标签底色，避免深色背景上看不清
    draw.rectangle((lx - pad, ly, lx + tw + pad, ly + th + 2 * pad), fill=(0, 0, 0))
    draw.text((lx, ly + pad // 2), label, fill=RED, font=font)
    # 标签指向红框的连线
    draw.line((lx + tw + pad, ly + th // 2 + pad, box[0], (box[1] + box[3]) // 2), fill=RED, width=LW)

im.save(DST)
print("saved", DST, im.size)
