"""ROS reference loader tests without V2 skill-service dependencies."""
import hashlib,json,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
from camera_ids import CAMERAS
from halo_task_references import load_reference_set

class ReferenceTests(unittest.TestCase):
    def fixture(self, root):
        refs = root / "halo_skill_memory/task_references/fixture"
        refs.mkdir(parents=True)
        ref = {"schema": "franka_task_reference_set_v1", "reference_set_id": "fixture"}
        for n, role in [(1, "initial_loose_blocks"), (2, "assembled_building_target")]:
            scene = refs / ("scene_%02d" % n)
            scene.mkdir()
            rows = []
            for camera, serial in CAMERAS.items():
                raw = b"\xff\xd8" + (str(n) + camera).encode() * 50 + b"\xff\xd9"
                image = scene / (camera + ".jpg")
                image.write_bytes(raw)
                rows.append({"camera_id": camera, "camera_identity": serial,
                             "path": str(image.relative_to(root)), "width": 640, "height": 360,
                             "sha256": hashlib.sha256(raw).hexdigest()})
            manifest = scene / "scene_manifest.json"
            manifest.write_text(json.dumps({"reference_set_id": "fixture", "role": role,
                                             "captured_at": "fixture", "images": rows}))
            ref["scene_%02d" % n] = {"status": "saved", "role": role,
                                       "manifest": str(manifest.relative_to(root))}
        path = refs / "reference_set.json"
        path.write_text(json.dumps(ref))
        return path


    def test_pairs_and_roles(self):
        with tempfile.TemporaryDirectory() as folder, patch('halo_task_references.ROOT',Path(folder)):
            ref=self.fixture(Path(folder)); context,groups=load_reference_set(ref)
            self.assertEqual(len(groups),2); self.assertEqual(sum(len(v) for _,v in groups),8)
            self.assertIn('object_appearance',groups[0][0]);self.assertIn('TARGET_STRUCTURE',groups[1][0])
    def test_hash_or_identity_change_rejected(self):
        for bad in ('identity','hash','path'):
            with self.subTest(bad=bad), tempfile.TemporaryDirectory() as folder, patch('halo_task_references.ROOT',Path(folder)):
                ref=self.fixture(Path(folder));manifest=ref.parent/'scene_01/scene_manifest.json';value=json.loads(manifest.read_text())
                row=value['images'][0]
                if bad=='identity':row['camera_identity']='wrong'
                elif bad=='hash':row['sha256']='0'*64
                else:row['path']='/tmp/outside-reference.jpg'
                manifest.write_text(json.dumps(value))
                with self.assertRaises(ValueError):load_reference_set(ref)

if __name__=='__main__':unittest.main()
