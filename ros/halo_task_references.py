"""Load explicitly selected reference images; never opens hardware or a model."""

import base64
import hashlib
import json
from pathlib import Path

from model_io import decode_camera_images
from camera_ids import CAMERAS


ROOT = Path(__file__).resolve().parent
USES = (
    ("scene_01", "initial_loose_blocks", "reference_01_object_appearance_NOT_current_positions",
     "Identify the five blocks' appearance, shape and identity; NOT current grasp coordinates."),
    ("scene_02", "assembled_building_target", "reference_02_FINAL_TARGET_STRUCTURE",
     "The desired final assembly: relative arrangement, orientations and stacking; NOT live state."),
)


def load_reference_set(path):
    path = Path(path).resolve()
    allowed = (ROOT / "halo_skill_memory/task_references").resolve()
    if not path.is_relative_to(allowed):
        raise ValueError("Reference set must be inside task_references")
    ref = json.loads(path.read_text())
    if (ref.get("schema") != "franka_task_reference_set_v1"
            or ref.get("reference_set_id") != path.parent.name):
        raise ValueError("Reference-set identity/schema mismatch")

    def local(value):
        result = (ROOT / value).resolve()
        if not result.is_relative_to(path.parent):
            raise ValueError("Reference artifact escapes selected set")
        return result

    context = {"reference_set_id": ref["reference_set_id"], "scenes": [],
               "live_observation_rule": "Only the current image group and native state describe current positions."
               " Reference images are not metric depth, absolute heights, or motion authorization."}
    groups = []
    for key, role, label, use in USES:
        selection = ref[key]
        if selection.get("status") != "saved" or selection.get("role") != role:
            raise ValueError("Missing or wrong reference scene: " + key)
        scene_path = local(selection["manifest"])
        scene = json.loads(scene_path.read_text())
        if scene.get("reference_set_id") != ref["reference_set_id"] or scene.get("role") != role:
            raise ValueError("Reference scene identity/role mismatch")
        rows = scene["images"]
        if len(rows) != 4 or {row["camera_id"] for row in rows} != set(CAMERAS):
            raise ValueError("Reference scene requires exactly four named cameras")
        by_camera = {row["camera_id"]: row for row in rows}
        images = []
        evidence = []
        for camera, identity in CAMERAS.items():
            row = by_camera[camera]
            if (row["camera_identity"] != identity
                    or (row["width"], row["height"]) != (640, 360)):
                raise ValueError("Reference camera identity or dimensions mismatch")
            image_path = local(row["path"])
            if not 100 <= image_path.stat().st_size <= 3 * 1024 * 1024:
                raise ValueError("Reference JPEG size outside bounds")
            raw = image_path.read_bytes()
            if hashlib.sha256(raw).hexdigest() != row["sha256"]:
                raise ValueError("Reference image hash changed")
            images.append(base64.b64encode(raw).decode("ascii"))
            evidence.append({"camera_id": camera, "camera_identity": identity,
                             "path": row["path"], "sha256": row["sha256"]})
        decode_camera_images(images)
        context["scenes"].append({"scene": key, "image_group": label, "use": use,
                                  "captured_at": scene["captured_at"], "images": evidence})
        groups.append((label, images))
    return context, groups
