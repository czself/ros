import math
import re
from pathlib import Path

import rospy
from gazebo_msgs.srv import GetModelState, GetWorldProperties

WORLD = Path('/root/competition_classic.world')
CAR_DIR = Path('/root/car_standees')

rospy.init_node('save_live_standees', anonymous=True)
world = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)()
get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
live = set(world.model_names)

poses = {}
for name in live:
    if name.startswith(('person_standee_', 'car_standee_plate_')):
        pose = get_state(name, 'world').pose
        yaw = math.atan2(
            2 * (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y),
            1 - 2 * (pose.orientation.y ** 2 + pose.orientation.z ** 2),
        )
        poses[name] = f'{pose.position.x:.9f} {pose.position.y:.9f} {pose.position.z:.9f} 0 0 {yaw:.9f}'

text = WORLD.read_text()
person_templates = {}
for index in range(1, 19):
    name = f'person_standee_{index:02d}'
    block = re.compile(rf"    <model name='{name}'>.*?</model>\n", re.DOTALL)
    matches = list(block.finditer(text))
    if matches:
        person_templates[name] = matches[0].group(0)
    text = block.sub('', text)

for index in range(1, 19):
    name = f'person_standee_{index:02d}'
    template = person_templates.get(name)
    if not template or name not in poses:
        continue
    saved = re.sub(
        r'(<static>1</static>\s*<pose>)[^<]+(</pose>)',
        rf'\g<1>{poses[name]}\g<2>',
        template,
        count=1,
    )
    text = text.replace('    <gravity>', saved + '    <gravity>', 1)

# Replace saved car standees, then serialize the three currently placed models.
for index in range(1, 4):
    name = f'car_standee_plate_{index}'
    text = re.sub(rf"    <model name='{name}'>.*?</model>\n", '', text, flags=re.DOTALL)
    if name not in poses:
        continue
    sdf = (CAR_DIR / f'{name}.sdf').read_text()
    model = re.search(r'<model name="[^"]+">.*?</model>', sdf, re.DOTALL).group(0)
    model = model.replace(f'<model name="{name}">', f"<model name='{name}'><pose>{poses[name]}</pose>")
    model = model.replace('file://materials/scripts', 'file:///root/car_standees/materials/scripts')
    model = model.replace('file://materials/textures', 'file:///root/car_standees/materials/textures')
    text = text.replace('    <gravity>', f'    {model}\n\n    <gravity>', 1)

WORLD.write_text(text)
print('Saved:', ', '.join(sorted(poses)))
