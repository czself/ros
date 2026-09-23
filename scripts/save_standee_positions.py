#!/usr/bin/env python3
import math
import re
from pathlib import Path

path = Path('worlds/competition_classic.world')
updates = {
    '03': (-0.3681309223, 0.4114572704, 0.0833333358, 0.0, 0.0, 0.0, 1.0),
    '04': (0.0750442594, 1.1031066179, 0.0833333358, 0.0, 0.0, 0.7388321757, 0.6738895178),
    '05': (0.9385417700, 1.2196197510, 0.0833333358, 0.0, 0.0, 0.9999979734, 0.0020350595),
    '06': (1.2217563391, 1.0660088062, 0.0833333358, 0.0, 0.0, 0.7006561756, 0.7134990692),
    '09': (-0.2528789937, 0.4158908725, 0.0833333358, 0.0, 0.0, 0.0, 1.0),
    '10': (0.0752871707, 0.5902152658, 0.0833333358, 0.0, 0.0, 0.6839552522, 0.7295239568),
    '11': (0.8087888956, 0.7923882604, 0.0833333358, 0.0, 0.0, -0.6957179904, 0.7183150649),
    '12': (0.8335768580, 0.6249683499, 0.0833333358, 0.0, 0.0, -0.6993780732, 0.7147520185),
    '17': (0.0783333629, 0.8944921494, 0.0833333358, 0.0, 0.0, 0.7402392626, 0.6723435521),
    '18': (1.2013981342, 0.9105842113, 0.0833333358, 0.0, 0.0, 0.7145826817, 0.6995509863),
}

text = path.read_text()
for number, (x, y, z, qx, qy, qz, qw) in updates.items():
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    pattern = rf"(<model name='person_standee_{number}'><static>1</static><pose>)[^<]+(</pose>)"
    replacement = rf"\g<1>{x:.9f} {y:.9f} {z:.9f} 0 0 {yaw:.9f}\g<2>"
    text, count = re.subn(pattern, replacement, text)
    if count != 1:
        raise SystemExit(f'expected one pose for person_standee_{number}, got {count}')
path.write_text(text)
print(f'updated {len(updates)} standee poses')
