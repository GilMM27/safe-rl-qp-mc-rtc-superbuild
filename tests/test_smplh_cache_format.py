import json
from pathlib import Path
import struct
import tempfile
import unittest
import xml.etree.ElementTree as ET

from smplh_playback.convert_cache import convert
class CacheFormatTests(unittest.TestCase):
    def test_legacy_upgrade_and_dimensions(self):
        import subprocess
        import sys

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            source = root / 'legacy'
            subprocess.run([sys.executable, '-m', 'smplh_playback.make_smoke_fixture',
                            '--output', str(source), '--frames', '3'], check=True)
            metadata_path = source / 'metadata.json'
            metadata = json.loads(metadata_path.read_text())
            metadata.pop('cache_format')
            metadata.pop('region_faces')
            metadata.pop('vertices')
            metadata.pop('faces')
            metadata_path.write_text(json.dumps(metadata))
            converted = root / 'converted'
            result = convert(source, converted)
            self.assertEqual(result['cache_format'], 1)
            self.assertEqual((result['frames'], result['vertices'], result['faces']), (3, 208, 208))
            self.assertEqual(len(result['regions']), 14)
            self.assertEqual(result['region_faces']['head'], 4)
            native = (converted / 'smplh_cache.bin').read_bytes()
            magic, frames, vertices, faces, regions, nq, fps = struct.unpack_from('<8sIIIII d', native)
            self.assertEqual((magic, frames, vertices, faces, regions, nq),
                             (b'SMPLHC1\0', 3, 208, 208, 14, 211))
            self.assertEqual(fps, 30.0)
            qpos_offset = 36 + 14 * (68 + 4 * 4)
            self.assertAlmostEqual(struct.unpack_from('<f', native, qpos_offset + 8)[0], 1.0)
            human = ET.parse(converted / 'human.xml').getroot()
            surface = human.find('./asset/mesh[@name="smplh_surface"]')
            self.assertEqual(Path(surface.get('file')), (converted / 'surface.obj').resolve())
            self.assertIn(str((converted / 'human.xml').resolve()),
                          (converted / 'smplh.yaml').read_text())


if __name__ == '__main__':
    unittest.main()
