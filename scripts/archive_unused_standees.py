#!/usr/bin/env python3
"""Move unused standees out of the active world while retaining their SDF."""

import re
from pathlib import Path

WORLD = Path('worlds/competition_classic.world')
ARCHIVE = Path('insert/person_standees/reserved_standees.sdf')
RESERVED = ('01', '02', '07', '08', '13', '14', '15', '16')

text = WORLD.read_text()
models = []
for number in RESERVED:
    pattern = re.compile(
        rf"\s*<model name='person_standee_{number}'>.*?</model>\n", re.DOTALL
    )
    text, count = pattern.subn(lambda match: models.append(match.group().strip()) or '\n', text)
    if count != 1:
        raise SystemExit(f'expected one person_standee_{number}, found {count}')

ARCHIVE.write_text(
    "<?xml version='1.0'?>\n<sdf version='1.6'>\n"
    "  <!-- Reserved standees removed from competition_classic.world. -->\n"
    + '\n'.join(f'  {model}' for model in models)
    + "\n</sdf>\n"
)
WORLD.write_text(text)
print(f'archived and removed {len(models)} standees')
