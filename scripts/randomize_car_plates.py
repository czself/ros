#!/usr/bin/env python3
"""Choose three distinct license plate images and prepare Gazebo materials."""

import argparse
import hashlib
import random
import shutil
from pathlib import Path


MATERIAL_TEMPLATE = """material CarStandee/Background
{{
  technique
  {{
    pass
    {{
      ambient 1 1 1 1
      diffuse 1 1 1 1
      texture_unit {{ texture car_background.png }}
    }}
  }}
}}

{plate_materials}
"""


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("inventory", type=Path, help="Directory containing plate image files")
    parser.add_argument("output", type=Path, help="Temporary output directory for Gazebo assets")
    args = parser.parse_args()

    inventory = args.inventory.expanduser().resolve()
    if not inventory.is_dir():
        parser.error(f"inventory directory does not exist: {inventory}")

    allowed_extensions = {".png", ".jpg", ".jpeg"}
    candidates = sorted(
        path for path in inventory.iterdir()
        if path.is_file() and path.suffix.lower() in allowed_extensions
    )
    stock = []
    seen_stems = set()
    seen_contents = set()
    for path in candidates:
        stem = path.stem.casefold()
        content = path.read_bytes()
        # Preserve the inventory's historical empty placeholders as files,
        # but never hand an invalid JPEG to Gazebo's material loader.
        if not content:
            continue
        digest = hashlib.sha256(content).digest()
        if stem in seen_stems or digest in seen_contents:
            continue
        stock.append(path)
        seen_stems.add(stem)
        seen_contents.add(digest)
    if len(stock) < 3:
        parser.error(f"need at least 3 plate images in {inventory}; found {len(stock)}")

    selected = random.SystemRandom().sample(stock, 3)
    scripts_dir = args.output / "materials" / "scripts"
    textures_dir = args.output / "materials" / "textures"
    scripts_dir.mkdir(parents=True, exist_ok=True)
    textures_dir.mkdir(parents=True, exist_ok=True)

    material_blocks = []
    for slot, source in enumerate(selected, start=1):
        texture_name = f"random_plate_{slot}{source.suffix.lower()}"
        shutil.copy2(source, textures_dir / texture_name)
        material_blocks.append(
            f"material CarStandee/Plate{slot}\n"
            "{\n"
            "  technique\n"
            "  {\n"
            "    pass\n"
            "    {\n"
            "      ambient 1 1 1 1\n"
            "      diffuse 1 1 1 1\n"
            f"      texture_unit {{ texture {texture_name} }}\n"
            "    }\n"
            "  }\n"
            "}"
        )

    (scripts_dir / "car_standees.material").write_text(
        MATERIAL_TEMPLATE.format(plate_materials="\n\n".join(material_blocks)),
        encoding="utf-8",
    )
    print("本次抽取车牌：" + "、".join(path.name for path in selected))


if __name__ == "__main__":
    main()
