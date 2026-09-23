import re
from pathlib import Path

path = Path('worlds/competition_classic.world')
text = path.read_text()
text = re.sub(
    r"(<model name='person_standee_\d+'><static>1</static><pose>[^ ]+ [^ ]+) 0( 0 0 0</pose>)",
    r"\1 0.083333333\2",
    text,
)
for index in range(1, 19):
    number = f'{index:02d}'
    material = f'PersonStandee/Person{number}' if index <= 16 else f'PersonStandee/PersonF{index - 16}'
    old = (
        "<visual name='placeholder'><pose>0 0 0.075 0 0 0</pose>"
        "<geometry><box><size>0.045 0.005 0.15</size></box></geometry>"
        "<material><script><uri>file:///root/person_standees/materials/scripts</uri>"
        "<uri>file:///root/person_standees/materials/textures</uri>"
        f"<name>{material}</name></script></material></visual>"
    )
    new = (
        "<visual name='standee_board'><pose>0 0 0.075 0 0 0</pose>"
        "<geometry><box><size>0.045 0.005 0.15</size></box></geometry>"
        "<material><ambient>0.82 0.82 0.82 1</ambient><diffuse>0.92 0.92 0.92 1</diffuse></material></visual>"
        "<visual name='person_image_front'><pose>0 -0.00255 0.075 0 0 0</pose>"
        "<geometry><box><size>0.045 0.0001 0.15</size></box></geometry>"
        "<material><script><uri>file:///root/person_standees/materials/scripts</uri>"
        "<uri>file:///root/person_standees/materials/textures</uri>"
        f"<name>{material}</name></script></material></visual>"
    )
    if text.count(old) == 1:
        text = text.replace(old, new)
    elif text.count(new) != 1:
        raise SystemExit(f'expected one standee visual for {index}')
path.write_text(text)
