import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest

from PIL import Image
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from prepare_localization_map import prepare_localization_map


class LocalizationMapTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.raw = self.root / 'raw.pgm'
        self.overlay = self.root / 'overlay.pgm'
        Image.new('L', (3, 3), 254).save(self.raw)
        image = Image.new('L', (3, 3), 254)
        image.putpixel((1, 1), 0)
        image.save(self.overlay)
        data = dict(resolution=.05, origin=[-1., -1., 0.], negate=0,
                    occupied_thresh=.65, free_thresh=.196)
        self.raw_yaml = self.root / 'raw.yaml'
        self.nav_yaml = self.root / 'overlay.yaml'
        self.raw_yaml.write_text(yaml.safe_dump(dict(data, image='raw.pgm')))
        self.nav_yaml.write_text(yaml.safe_dump(dict(data, image='overlay.pgm')))
        self.manifest = dict(kind='navigation_real_white_line_overlay',
                             source_yaml=str(self.raw_yaml),
                             source_map_sha256=hashlib.sha256(self.raw.read_bytes()).hexdigest())
        self.nav_yaml.with_suffix('.manifest.json').write_text(json.dumps(self.manifest))

    def test_amcl_copy_excludes_virtual_paint_and_preserves_safety_map(self):
        before = self.overlay.read_bytes()
        result = prepare_localization_map(self.nav_yaml, self.root / 'out' / 'physical')
        data = yaml.safe_load(result.read_text())
        self.assertEqual((result.parent / data['image']).read_bytes(), self.raw.read_bytes())
        self.assertEqual(self.overlay.read_bytes(), before)

    def test_changed_source_map_is_rejected(self):
        Image.new('L', (3, 3), 0).save(self.raw)
        with self.assertRaisesRegex(ValueError, 'hash'):
            prepare_localization_map(self.nav_yaml, self.root / 'out')

    def test_different_coordinate_frame_geometry_is_rejected(self):
        data = yaml.safe_load(self.raw_yaml.read_text())
        data['origin'][0] += .1
        self.raw_yaml.write_text(yaml.safe_dump(data))
        with self.assertRaisesRegex(ValueError, 'geometry'):
            prepare_localization_map(self.nav_yaml, self.root / 'out')
