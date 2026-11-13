"""V2 skill geometry and execution; no historical poses, ROS, or model arithmetic."""

import copy
import json
import math
from pathlib import Path
import sys
import time

from model_io import number
from v2_transport import HouseExecutor
from v2_state import multiply_quaternions


PROTOCOL = "halo_v2_skills_v2"
MAX_SKILLS = 64
STANDARD_SPEEDS = {"horizontal_mps": 0.14, "vertical_mps": 0.06, "angular_radps": 0.80}
OPERATIONS = {
    "translate": {"position_m": "absolute base [x,y,z] metres; same horizontal plane"},
    "descend": {"distance_m": "positive base-Z descent only; gripper unchanged, empty or holding"},
    "close": {},
    "rotate": {"angle_deg": "base-Z right-hand rotation in degrees; TCP position unchanged"},
    "lift": {"distance_m": "positive base-Z rise"},
    "open": {},
}


def arm_speeds(scale):
    if not number(scale) or not 0 < scale <= 1:
        raise ValueError("Arm speed scale must be in (0, 1]")
    return {key: speed * scale for key, speed in STANDARD_SPEEDS.items()}


def pose_value(pose):
    result = {}
    for key, length in (("position_m", 3), ("quaternion_wxyz", 4)):
        value = pose.get(key)
        if not isinstance(value, list) or len(value) != length or not all(map(number, value)):
            raise ValueError("Invalid measured pose: " + key)
        result[key] = list(map(float, value))
    if abs(math.sqrt(sum(x * x for x in result["quaternion_wxyz"])) - 1) > 1e-3:
        raise ValueError("Pose quaternion must be normalized wxyz")
    return result


def validate_skill(skill):
    if not isinstance(skill, dict) or skill.get("op") not in OPERATIONS:
        raise ValueError("Unknown V2 skill")
    op = skill["op"]
    if set(skill) != {"op", *OPERATIONS[op]}:
        raise ValueError("Unexpected or missing parameters for " + op)
    if op == "translate":
        xyz = skill["position_m"]
        if not isinstance(xyz, list) or len(xyz) != 3 or not all(map(number, xyz)):
            raise ValueError("translate requires three finite base coordinates")
    elif op in ("descend", "lift"):
        distance = skill["distance_m"]
        if not number(distance) or not 0 < distance <= 0.25:
            raise ValueError("Vertical distance must be in (0, 0.25] metres")
    elif op == "rotate":
        angle = skill["angle_deg"]
        if not number(angle) or not 0 < abs(angle) <= 180:
            raise ValueError("Rotation magnitude must be in (0, 180] degrees")
    return copy.deepcopy(skill)


def expected_pose(start, skill):
    result = pose_value(start)
    skill = validate_skill(skill)
    op = skill["op"]
    if op == "translate":
        # Operator disabled this local nominal-vs-measured Z rejection, 2026-11-12.
        # if abs(skill["position_m"][2] - result["position_m"][2]) > 0.003:
        #     raise ValueError("translate must remain in the current horizontal plane")
        # Preserve the measured plane instead of commanding accumulated Z error.
        result["position_m"][:2] = skill["position_m"][:2]
    elif op in ("descend", "lift"):
        result["position_m"][2] += skill["distance_m"] * (-1 if op == "descend" else 1)
    elif op == "rotate":
        half = math.radians(skill["angle_deg"]) / 2
        result["quaternion_wxyz"] = multiply_quaternions(
            [math.cos(half), 0, 0, math.sin(half)], result["quaternion_wxyz"],
        )
    x, y, z = result["position_m"]
    if not 0.20 <= x <= 0.65 or not -0.35 <= y <= 0.35 or z > 0.60:
        raise ValueError("Skill exceeds existing Halo software workspace")
    if math.dist(start["position_m"], result["position_m"]) > 0.25 + 1e-9:
        raise ValueError("Skill exceeds the native V2 0.25m finite-action bound")
    return result


def plan_sequence(initial_pose, skills):
    if not isinstance(skills, list) or not 1 <= len(skills) <= MAX_SKILLS:
        raise ValueError("A plan requires 1 to %d skills" % MAX_SKILLS)
    pose = pose_value(initial_pose)
    records = []
    for index, skill in enumerate(skills):
        skill = validate_skill(skill)
        pose = expected_pose(pose, skill)
        records.append({"index": index, "skill": skill, "expected_pose": copy.deepcopy(pose)})
    return records


def duration_for(start, target, skill, arm_speed_scale=1):
    speeds = arm_speeds(arm_speed_scale)
    if skill["op"] == "rotate":
        return max(0.5 / arm_speed_scale, 1.875 * abs(math.radians(skill["angle_deg"])) /
                   speeds["angular_radps"])
    speed = speeds["horizontal_mps" if skill["op"] == "translate" else "vertical_mps"]
    return max(0.5 / arm_speed_scale,
               1.875 * math.dist(start["position_m"], target["position_m"]) / speed)


class SkillExecutor(HouseExecutor):
    def __init__(self, robot, directory, cameras, *, recording=True, arm_speed_scale=1):
        self.arm_speeds = arm_speeds(arm_speed_scale)
        self.arm_speed_scale = arm_speed_scale
        self.recording = recording
        super().__init__(robot, Path(directory), cameras, 0.06)
        self.records = []
        self.last_actual = self.state()["actual_pose"]

    def append(self, filename, value):
        if not self.recording:
            return
        with (self.directory / filename).open("a") as output:
            output.write(json.dumps(value, allow_nan=False) + "\n")

    def log(self, event, **fields):
        if self.recording:
            super().log(event, **fields)

    def open(self):
        self.cameras.request("health")
        state = self.state()
        operation = self.robot.gripper.move(
            width_m=min(0.08, state["gripper"]["max_width_m"]), speed_mps=0.08,
            timeout_s=10, wait=False,
        )
        after = self.wait(operation)
        self.holding = False
        return after

    def execute_skill(self, planned):
        self.cameras.request("health")
        before = self.state()
        actual = pose_value(before["actual_pose"])
        # Operator disabled only this local between-skill drift rejection, 2026-11-12.
        # Native state, ownership, feedback and protection checks above remain active.
        # if (math.dist(actual["position_m"], self.last_actual["position_m"]) > 0.003
        #         or abs(sum(a * b for a, b in zip(
        #             actual["quaternion_wxyz"], self.last_actual["quaternion_wxyz"],
        #         ))) < math.cos(0.03 / 2)):
        #     raise RuntimeError("Robot moved between skills; refusing a stale one-shot plan")
        skill = planned["skill"]
        target = expected_pose(actual, skill)
        record = {
            **copy.deepcopy(planned), "before_actual_pose": actual,
            "dispatch_expected_pose": target, "status": "started",
            "arm_speed_scale": self.arm_speed_scale, "speed_contract": self.arm_speeds,
            "host_started_monotonic_s": time.monotonic(),
        }
        self.log("skill_started", **record)
        self.append("dispatched_expected_poses.jsonl", {
            "index": planned["index"], "pose": target, "anchor": "measured_before_skill",
        })
        try:
            if skill["op"] == "close":
                if self.holding or before["gripper"]["width_m"] < 0.005:
                    raise RuntimeError("close requires an open, empty gripper")
                operation = self.robot.gripper.grasp(
                    width_m=0, speed_mps=0.08, force_n=20, epsilon_inner_m=0.005,
                    epsilon_outer_m=min(0.08, before["gripper"]["max_width_m"]),
                    timeout_s=10, wait=False,
                )
                after = self.wait(operation)
                if not after["gripper"]["is_grasped"]:
                    raise RuntimeError("Native gripper did not confirm grasp; sequence stopped")
                self.holding = True
            elif skill["op"] == "open":
                after = self.open()
            else:
                operation = self.robot.move_pose(
                    target["position_m"], target["quaternion_wxyz"],
                    duration_s=duration_for(actual, target, skill, self.arm_speed_scale), wait=False,
                )
                after = self.wait(operation, arm_motion=True)
            if skill["op"] == "descend":
                self.log("descent_completed", actual_state=after)
            record.update(status="succeeded", actual_pose=pose_value(after["actual_pose"]))
            self.last_actual = record["actual_pose"]
            return record
        except (RuntimeError, OSError, ValueError, KeyError, TypeError, KeyboardInterrupt) as exc:
            record.update(status="failed", error=type(exc).__name__ + ": " + str(exc))
            raise
        finally:
            pending_error = sys.exc_info()[0] is not None
            measurement_error = None
            try:
                state = self.robot.get_state()
                record["actual_pose"] = pose_value(state["actual_pose"])
                record["measurement_valid"] = (
                    state["instance_id"] == self.robot.instance_id
                    and state["feedback"]["valid"] and not state["feedback"]["stale"]
                )
                if not record["measurement_valid"]:
                    raise RuntimeError("Final pose feedback is stale, invalid or from another instance")
                record["position_error_m"] = math.dist(
                    record["actual_pose"]["position_m"], target["position_m"],
                )
                dot = sum(a * b for a, b in zip(
                    record["actual_pose"]["quaternion_wxyz"], target["quaternion_wxyz"],
                ))
                record["quaternion_error_rad"] = 2 * math.acos(min(1, abs(dot)))
            except (RuntimeError, OSError, ValueError, KeyError) as exc:
                measurement_error = str(exc)
                state = None
                record.update(actual_pose=None, measurement_valid=False,
                              measurement_error=measurement_error)
                if not pending_error:
                    record.update(status="failed", error="Final pose measurement failed")
            record["host_finished_monotonic_s"] = time.monotonic()
            record["measurement_source"] = "teleop_v2.actual_pose; encoder-based FK, not vision"
            self.records.append(record)
            self.append("skill_results.jsonl", record)
            self.append("actual_poses.jsonl", {
                "index": planned["index"], "pose": record["actual_pose"],
                "status": record["status"], "source": record["measurement_source"],
                "feedback": state["feedback"] if state else None,
                "measurement_valid": record["measurement_valid"],
            })
            if measurement_error is not None and not pending_error:
                raise RuntimeError("Final pose measurement failed: " + measurement_error)

    def stop_owned(self):
        state = self.robot.get_state()
        if state["owner_session_id"] != self.robot.session_id:
            raise RuntimeError("Writer ownership lost; refusing to stop or reset another writer")
        self.log("protected_stop", state=self.robot.stop_all())

    def finish_reset(self):
        self.cameras.request("health")
        self.state()
        self.log("cleanup_open_started")
        self.open()
        self.log("cleanup_reset_started")
        self.cameras.request("health")
        self.state()
        operation = self.robot.reset(wait=False)
        after = self.wait(operation, arm_motion=True)
        self.log("cleanup_reset_completed", actual_state=after)
        return after
