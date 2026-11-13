"""Read-only historical calibration and current-preview geometry for V2 skills."""

import base64
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation

from camera_ids import CAMERAS


ROOT = Path(__file__).resolve().parents[1]
WRIST = "realsense_1"


def source(path):
    raw = Path(path).read_bytes()
    return json.loads(raw), {"path": str(path), "sha256": hashlib.sha256(raw).hexdigest()}


def transform(value):
    matrix = np.asarray(value, dtype=float)
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
            or not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-8)
            or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-4)
            or abs(np.linalg.det(matrix[:3, :3]) - 1) > 1e-4):
        raise ValueError("Invalid calibration rigid transform")
    return matrix


def pose_matrix(value):
    position = np.asarray(value["position_m"], dtype=float)
    quaternion = np.asarray(value["quaternion_wxyz"], dtype=float)
    if (position.shape != (3,) or quaternion.shape != (4,)
            or not np.isfinite(position).all() or not np.isfinite(quaternion).all()
            or abs(np.linalg.norm(quaternion) - 1) > 1e-3):
        raise ValueError("Invalid measured calibration pose")
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat(
        np.roll(quaternion, -1),
    ).as_matrix()
    matrix[:3, 3] = position
    return matrix


def scaled_intrinsics(intrinsics, width, height):
    sx, sy = width / intrinsics["width"], height / intrinsics["height"]
    if not math.isclose(sx, sy, abs_tol=1e-9):
        raise ValueError("Preview is not a uniform resize of the calibrated source stream")
    if (not all(math.isfinite(intrinsics[k]) for k in ("fx", "fy", "ppx", "ppy"))
            or min(intrinsics["fx"], intrinsics["fy"]) <= 0
            or any(abs(c) > 1e-9 for c in intrinsics["coeffs"])):
        raise ValueError("Invalid or nonzero-distortion intrinsics; rectification is required")
    return {
        "width": width, "height": height,
        "fx": intrinsics["fx"] * sx, "fy": intrinsics["fy"] * sy,
        "ppx": (intrinsics["ppx"] + 0.5) * sx - 0.5,
        "ppy": (intrinsics["ppy"] + 0.5) * sy - 0.5,
        "coeffs": list(intrinsics["coeffs"]),
    }


def factory_intrinsics(serial, stream):
    import pyrealsense2 as rs

    for device in rs.context().query_devices():
        if device.get_info(rs.camera_info.serial_number) != serial:
            continue
        for sensor in device.query_sensors():
            for profile in sensor.get_stream_profiles():
                if profile.stream_type() != rs.stream.color or profile.format() != rs.format.bgr8:
                    continue
                video = profile.as_video_stream_profile()
                if (video.width(), video.height(), profile.fps()) != (
                        stream["width"], stream["height"], stream["fps"]):
                    continue
                intr = video.get_intrinsics()
                return {
                    "width": intr.width, "height": intr.height,
                    "fx": intr.fx, "fy": intr.fy, "ppx": intr.ppx, "ppy": intr.ppy,
                    "coeffs": list(intr.coeffs), "distortion_model": str(intr.model),
                }
    raise ValueError("No matching read-only RealSense color profile for " + serial)


def project_pixels(pixels, intrinsics, base_T_camera, plane_normal, plane_offset, height):
    pixels = np.asarray(pixels, dtype=float)
    if pixels.ndim != 2 or pixels.shape[1] != 2 or not np.isfinite(pixels).all():
        raise ValueError("Invalid image coordinates")
    rays = np.column_stack((
        (pixels[:, 0] - intrinsics["ppx"]) / intrinsics["fx"],
        (pixels[:, 1] - intrinsics["ppy"]) / intrinsics["fy"],
        np.ones(len(pixels)),
    )) @ base_T_camera[:3, :3].T
    origin = base_T_camera[:3, 3]
    denominator = rays @ plane_normal
    if np.any(np.abs(denominator) < 1e-6):
        raise ValueError("Image ray is parallel to the historical table plane")
    distance = (height - plane_offset - plane_normal @ origin) / denominator
    if np.any(distance <= 0) or not np.isfinite(distance).all():
        raise ValueError("Historical table plane is behind the current camera")
    return origin + rays * distance[:, None]


def calibration_geometry(config, state, metadata, read_intrinsics=factory_intrinsics):
    if config["setup_unchanged_confirmed"] is not True:
        raise ValueError("Historical calibration reuse requires explicit unchanged-setup confirmation")
    site, site_ref = source(config["site_profile"])
    table, table_ref = source(config["table_reference"])
    taught, taught_ref = source(config["taught_trajectory"])
    binding = site["bindings"]
    if (site["site"]["camera_parent"] != "tcp"
            or site["site"]["camera_id"] != "real_2"
            or binding["camera_serial"] != CAMERAS[WRIST]):
        raise ValueError("Historical wrist camera binding does not match V2")
    tool = np.asarray(binding["controller_F_T_EE"])
    if (tool.shape != (16,)
            or not np.allclose(tool, taught["robot_binding"]["F_T_EE"], atol=1e-7, rtol=0)
            or not np.allclose(tool, table["frozen_start_state"]["F_T_EE"], atol=1e-7, rtol=0)):
        raise ValueError("Historical TCP bindings disagree")
    live_tool = state.get("feedback", {}).get("F_T_EE")
    if live_tool is not None and not np.allclose(tool, live_tool, atol=1e-7, rtol=0):
        raise ValueError("Current TCP differs from historical calibration")
    if not taught["validated_pick_and_lift"]:
        raise ValueError("Missing validated grasp-height teaching")
    if set(metadata) != set(CAMERAS):
        raise ValueError("Calibration requires metadata for all four current cameras")
    now = time.monotonic_ns()
    for name, serial in CAMERAS.items():
        item = metadata[name]
        if (item["actual_identity"] != serial or item["source"] != "v2_web_rtc_preview"
                or (item["width"], item["height"]) != (640, 360)
                or not 0 <= now - item["host_rx_mono_ns"] <= 1_500_000_000):
            raise ValueError("Wrong or stale calibrated preview: " + name)
    item = metadata[WRIST]
    intrinsics = read_intrinsics(binding["camera_serial"], item["source_stream"])
    preview_intrinsics = scaled_intrinsics(intrinsics, item["width"], item["height"])
    tcp = pose_matrix(state["actual_pose"])
    tilt = math.degrees(math.acos(np.clip(-tcp[2, 2], -1, 1)))
    if tilt > 5:
        raise ValueError("Taught grasp height requires the unchanged near-upright tool")
    base_T_camera = tcp @ transform(site["site"]["camera_transform"])
    historical = transform(table["camera_to_base_candidate"])
    plane = table["table_plane_camera"]
    normal = np.asarray(plane["normal"], dtype=float)
    if (normal.shape != (3,) or not np.isfinite(normal).all()
            or abs(np.linalg.norm(normal) - 1) > 1e-4
            or not math.isfinite(plane["offset_m"])
            or not 0 <= plane["rms_m"] <= 0.002 or plane["inlier_fraction"] < 0.75):
        raise ValueError("Invalid historical table-plane evidence")
    historical_tcp = np.asarray(table["frozen_start_state"]["O_T_EE"]).reshape(4, 4, order="F")
    if not np.allclose(historical, historical_tcp @ transform(site["site"]["camera_transform"]),
                       atol=1e-7, rtol=0):
        raise ValueError("Table-plane reference and hand-eye transform disagree")
    normal = historical[:3, :3] @ normal
    offset = float(plane["offset_m"] - normal @ historical[:3, 3])
    grasp_z = taught["grasp_target_tcp_z_m"]
    height_interval = taught["operator_measurements"]["block_height_interval_m"]
    if (not isinstance(grasp_z, (int, float)) or not math.isfinite(grasp_z)
            or not 0 < height_interval[0] <= height_interval[1] < 0.1):
        raise ValueError("Invalid historical grasp-height evidence")
    return {
        "status": "historical_candidate_reused_with_operator_confirmation",
        "camera_id": WRIST, "legacy_camera_id": "real_2", "camera_serial": binding["camera_serial"],
        "setup_unchanged_confirmed": True,
        "tcp_binding_verification": "live_F_T_EE" if live_tool is not None
                                    else "operator_confirmation; native feedback does not expose F_T_EE",
        "source_refs": [site_ref, table_ref, taught_ref],
        "source_color_intrinsics": intrinsics, "preview_intrinsics": preview_intrinsics,
        "base_T_camera": base_T_camera.tolist(),
        "table_plane_base": {"normal": normal.tolist(), "offset_m": offset,
                             "historical_fit_rms_m": plane["rms_m"]},
        "taught_grasp_tcp_z_m": grasp_z, "compatible_unstacked_block_height_interval_m": height_interval,
        "minimum_horizontal_tcp_z_m": taught["robot_binding"]["reference_O_T_EE"][14] - 0.180,
        "current_pose": state["actual_pose"], "current_tool_tilt_deg": tilt,
        "current_closing_axis_yaw_deg": math.degrees(math.atan2(tcp[1, 1], tcp[0, 1])),
        "current_gripper_width_m": state["gripper"]["width_m"],
        "assumed_top_height_m": sum(height_interval) / 2,
        "limits": [
            "Historical candidate hand-eye, not commissioned metric accuracy or collision certification.",
            "Historical plane reused because operator confirmed base/table/mount/fingers/TCP unchanged; no live depth.",
            "13-15mm unstacked blocks only for this top-plane and taught grasp-height assumption.",
            "Contour candidates are not object identities, safe corridors or proof of finger clearance.",
            "Old target XY/yaw, grasp parameters and task instructions are NOT replayed.",
        ],
    }


def object_candidates(image, geometry):
    import cv2

    intr = geometry["preview_intrinsics"]
    if image is None or image.shape[:2] != (intr["height"], intr["width"]):
        raise ValueError("Image dimensions do not match calibrated preview")
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    candidates = []
    ranges = {
        "red": [(0, 8), (170, 179)], "orange": [(9, 17)], "yellow": [(18, 38)],
        "green": [(39, 84)], "blue_cyan": [(85, 130)], "purple": [(131, 169)],
    }
    plane = geometry["table_plane_base"]
    for color, intervals in ranges.items():
        mask = np.zeros(image.shape[:2], np.uint8)
        for low, high in intervals:
            mask |= cv2.inRange(hsv, (low, 70, 60), (high, 255, 255))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
        for contour in contours:
            area = cv2.contourArea(contour)
            if not 100 <= area <= 20000:
                continue
            box = cv2.boxPoints(cv2.minAreaRect(contour))
            points = project_pixels(
                box, intr, np.asarray(geometry["base_T_camera"]),
                np.asarray(plane["normal"]), plane["offset_m"], geometry["assumed_top_height_m"],
            )
            edges = np.roll(points, -1, axis=0) - points
            lengths = np.linalg.norm(edges[:, :2], axis=1)
            axis = edges[int(np.argmax(lengths)), :2]
            angle = math.degrees(math.atan2(axis[1], axis[0]))
            yaw = (angle - geometry["current_closing_axis_yaw_deg"] + 90) % 180 - 90
            center = points.mean(axis=0)
            long_side, short_side = float(max(lengths)), float(min(lengths))
            if not 0.008 <= short_side <= long_side <= 0.16:
                continue
            variants = [
                project_pixels(
                    box, intr, np.asarray(geometry["base_T_camera"]),
                    np.asarray(plane["normal"]), plane["offset_m"], height,
                ).mean(axis=0).tolist()
                for height in geometry["compatible_unstacked_block_height_interval_m"]
            ]
            candidates.append({
                "color_hint": color, "center_uv_px": box.mean(axis=0).tolist(),
                "box_uv_px": box.tolist(), "center_base_estimate_m": center.tolist(),
                "height_interval_center_estimates_m": variants,
                "long_side_m": long_side, "short_side_m": short_side,
                "rectangular_fill_fraction": float(area / max(cv2.contourArea(box), 1)),
                "short_end_grasp_rotate_deg": yaw,
                "long_side_grasp_rotate_deg": (yaw + 180) % 180 - 90,
                "short_end_span_fits_current_opening": long_side + 0.006 < geometry["current_gripper_width_m"],
                "touches_image_border": bool(
                    np.any(box < 2) or np.any(box[:, 0] > intr["width"] - 3)
                    or np.any(box[:, 1] > intr["height"] - 3)),
            })
    candidates.sort(key=lambda c: (c["center_uv_px"][1], c["center_uv_px"][0], c["color_hint"]))
    for index, candidate in enumerate(candidates):
        candidate["id"] = "object_%02d" % index
    return candidates


def build_evidence(config, payload):
    result = calibration_geometry(config, payload["actual_state"], payload["camera_metadata"])
    import cv2

    raw = base64.b64decode(payload["images"][WRIST], validate=True)
    image = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
    result["object_candidates"] = object_candidates(image, result)
    result["hardware_commands_sent"] = 0
    return result


def request_evidence(config, payload):
    try:
        process = subprocess.run(
            [config["camera_python"], "-m", "robot_calibration.skill_calibration"],
            input=json.dumps({"config": config["calibration"], "observation": payload}),
            text=True, capture_output=True, timeout=30, cwd=ROOT, check=True,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        detail = exc.stderr if isinstance(exc, subprocess.CalledProcessError) else str(exc)
        raise ValueError("Calibration geometry unavailable: " + detail[-4000:]) from exc
    return json.loads(process.stdout)


if __name__ == "__main__":
    request = json.load(sys.stdin)
    print(json.dumps(build_evidence(request["config"], request["observation"]), allow_nan=False))
