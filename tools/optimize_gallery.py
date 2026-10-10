"""Generate lightweight README previews from existing robot photos.

Run from the repository root:
    python -m pip install Pillow
    python tools/optimize_gallery.py
"""
from pathlib import Path
from PIL import Image, ImageOps

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "images_videos"
DEST = SOURCE / "optimized"
FILES = ("IMG_0701.png", "IMG_0703.png", "IMG_0704.png")
DEST.mkdir(parents=True, exist_ok=True)

for filename in FILES:
    src = SOURCE / filename
    if not src.is_file():
        raise FileNotFoundError(f"Missing original image: {src}")
    target = DEST / (src.stem + ".webp")
    with Image.open(src) as original:
        img = ImageOps.exif_transpose(original).convert("RGB")
        img.thumbnail((1400, 1400), Image.Resampling.LANCZOS)
        img.save(target, "WEBP", quality=82, method=6)
    before = src.stat().st_size / 1024 / 1024
    after = target.stat().st_size / 1024 / 1024
    print(f"{filename}: {before:.2f} MB -> {after:.2f} MB ({100*(1-after/before):.1f}% smaller)")

readme_path = ROOT / "README.md"
readme = readme_path.read_text(encoding="utf-8")
replacements = {
    "images_videos/IMG_0701.png": "images_videos/optimized/IMG_0701.webp",
    "images_videos/IMG_0703.png": "images_videos/optimized/IMG_0703.webp",
    "images_videos/IMG_0704.png": "images_videos/optimized/IMG_0704.webp",
}
for old, new in replacements.items():
    if old not in readme and new not in readme:
        raise ValueError(f"README does not reference expected image: {old}")
    readme = readme.replace(old, new)
readme_path.write_text(readme, encoding="utf-8")
print("README image links updated; originals retained.")
