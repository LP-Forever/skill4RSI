"""Offline reference-role and actual image-attachment checks."""

import base64
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

from model_io import PolicyError
from halo_pipeline_skills import verify_health
from halo_skill_service import SkillService
from halo_task_references import load_reference_set
from camera_ids import CAMERAS


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

    def config(self, root, ref):
        return {"recording_enabled": False, "memory_dir": str(root / "memory"),
                "run_dir": str(root / "runs"), "model": "gpt-6.1-sol", "record_fps": 2,
                "reasoning_effort": "high", "timeout_s": 30, "model_base_url": "http://unused",
                "finish_behavior": "hold", "task_reference_set": str(ref)}

    def observation(self):
        image = base64.b64encode(b"\xff\xd8" + b"x" * 100 + b"\xff\xd9").decode()
        return {"exp_id": "reference_test", "actual_state": {"actual_pose": {
            "position_m": [0.4, 0, 0.4], "quaternion_wxyz": [0, 1, 0, 0]}},
                "images": {c: image for c in CAMERAS}}

    def test_exact_images_and_roles_are_actually_passed_to_predictor(self):
        with tempfile.TemporaryDirectory() as folder, patch("halo_task_references.ROOT", Path(folder)):
            root = Path(folder)
            ref = self.fixture(root)
            context, groups = load_reference_set(ref)
            self.assertEqual(len(groups), 2)
            self.assertEqual(sum(len(images) for _, images in groups), 8)
            self.assertIn("object_appearance", groups[0][0])
            self.assertIn("TARGET_STRUCTURE", groups[1][0])
            predictor = Mock(return_value=({"status": "planned", "message": "fixture only",
                                            "skills": [{"op": "open"}]}, {}, "{}"))
            service = SkillService(self.config(root, ref), "test references", predictor)
            service.predict(self.observation())
            passed = predictor.call_args.args
            self.assertEqual(passed[4]["task_references"], context)
            self.assertEqual(passed[5][:2], groups)
            self.assertEqual(passed[5][2][0], "current")
            self.assertEqual(service.health()["reference_image_count"], 8)

    def test_tamper_delete_and_reference_change_fail_closed(self):
        for change in ("tamper", "delete", "context_change"):
            with self.subTest(change=change), tempfile.TemporaryDirectory() as folder:
                root = Path(folder)
                with patch("halo_task_references.ROOT", root):
                    ref = self.fixture(root)
                    predictor = Mock()
                    service = SkillService(self.config(root, ref), "fixture", predictor)
                    image = next((ref.parent / "scene_01").glob("*.jpg"))
                    if change == "delete":
                        image.unlink()
                    elif change == "tamper":
                        image.write_bytes(image.read_bytes() + b"tampered")
                    else:
                        path = ref.parent / "scene_01/scene_manifest.json"
                        value = json.loads(path.read_text())
                        value["captured_at"] = "different"
                        path.write_text(json.dumps(value))
                    with self.assertRaises((ValueError, OSError)):
                        service.predict(self.observation())
                    predictor.assert_not_called()

    def test_wrong_camera_identity_and_cross_set_paths_rejected(self):
        for bad in ("serial", "escape"):
            with self.subTest(bad=bad), tempfile.TemporaryDirectory() as folder:
                root = Path(folder)
                with patch("halo_task_references.ROOT", root):
                    ref = self.fixture(root)
                    path = ref.parent / "scene_01/scene_manifest.json"
                    value = json.loads(path.read_text())
                    if bad == "serial":
                        value["images"][0]["camera_identity"] = "wrong"
                    else:
                        value["images"][0]["path"] = "/tmp/outside-reference.jpg"
                    path.write_text(json.dumps(value))
                    with self.assertRaises(ValueError):
                        load_reference_set(ref)

if __name__ == "__main__":
    unittest.main()
