"""Single-plan GPT skill experiment with V2 feedback, video and open/reset cleanup."""

import argparse
import base64
import copy
from datetime import datetime
import json
import math
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time
import uuid

from v2_transport import Cameras
from halo_skill_service import ROOT, configuration, save
from camera_ids import CAMERAS
from robot_calibration.block_skills_v2 import (
    PROTOCOL, STANDARD_SPEEDS, SkillExecutor, arm_speeds, plan_sequence,
)


class SkillRecorder:
    def __init__(self, config, directory):
        self.directory = Path(directory)
        self.recording = config.get("recording_enabled", True)
        if self.recording:
            self.directory.mkdir(parents=True)
        self.fps = config["record_fps"]
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.thread = None
        self.latest = None
        self.received = 0
        self.error = None
        self.count = 0
        if self.recording:
            for camera in CAMERAS:
                (self.directory / camera).mkdir()
        self.bridge = Cameras(config, self.directory if self.recording else None)

    def capture(self):
        frame = self.bridge.request("snapshot")
        if self.recording:
            for camera in CAMERAS:
                raw = base64.b64decode(frame["images"][camera], validate=True)
                (self.directory / camera / ("%08d.jpg" % self.count)).write_bytes(raw)
        received = time.monotonic()
        if self.recording:
            with (self.directory / "frames.jsonl").open("a") as output:
                output.write(json.dumps({
                    "frame": self.count, "host_received_monotonic_s": received,
                    "metadata": frame["metadata"], "record_fps": self.fps,
                }) + "\n")
        self.count += 1
        with self.lock:
            self.latest, self.received = frame, received

    def start(self):
        self.bridge.start()
        self.capture()
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()

    def loop(self):
        deadline = time.monotonic()
        while not self.stop_event.wait(max(0, deadline + 1 / self.fps - time.monotonic())):
            try:
                self.capture()
            except (OSError, ValueError, KeyError, RuntimeError) as exc:
                self.error = exc
                print("SKILL RECORDING FAILED: " + str(exc), file=sys.stderr, flush=True)
                return
            deadline += 1 / self.fps
            if deadline < time.monotonic() - 1 / self.fps:
                deadline = time.monotonic()

    def check(self):
        if self.error is not None:
            raise RuntimeError("Camera recording failed: " + str(self.error)) from self.error
        with self.lock:
            age = time.monotonic() - self.received
        if age > max(0.75, 2 / self.fps):
            raise RuntimeError("Recorded observation is stale")

    def request(self, command):
        self.check()
        if command == "health":
            return {"ready": True}
        if command != "snapshot":
            raise ValueError("Invalid recorder command")
        with self.lock:
            return copy.deepcopy(self.latest)

    def snapshot_after(self, moment):
        deadline = time.monotonic() + 3 + 2 / self.fps
        while True:
            self.check()
            with self.lock:
                if self.received > moment:
                    return copy.deepcopy(self.latest)
            if time.monotonic() >= deadline:
                raise TimeoutError("No new image after skill execution ended")
            time.sleep(0.05)

    def stop(self):
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=12)
            if self.thread.is_alive():
                raise RuntimeError("Camera capture thread did not stop; helper still active")
        self.bridge.close()

    def encode(self):
        if not self.recording:
            return
        for camera in CAMERAS:
            if not self.count:
                raise RuntimeError("No recorded frames; cannot save video")
            subprocess.run([
                "ffmpeg", "-nostdin", "-hide_banner", "-loglevel", "error", "-y",
                "-framerate", str(self.fps), "-i",
                str(self.directory / camera / "%08d.jpg"),
                "-c:v", "libx264", "-threads", "1", "-preset", "ultrafast",
                "-pix_fmt", "yuv420p", str(self.directory / (camera + ".mp4")),
            ], check=True)


def verify_health(health, config):
    try:
        speeds = arm_speeds(health.get("arm_speed_scale", 1))
    except ValueError as exc:
        raise RuntimeError("Invalid skill service arm speed") from exc
    if (health.get("protocol") != PROTOCOL or health.get("model") != config["model"]
            or health.get("status") != "waiting_for_halo"
            or health.get("predictions") != 0
            or health.get("robot_control") is not False
            or health.get("camera_ids") != list(CAMERAS)
            or health.get("record_fps") != config["record_fps"]
            or health.get("calibration_enabled", False) != ("calibration" in config)
            or health.get("speed_contract", STANDARD_SPEEDS) != speeds
            or not isinstance(health.get("recording_enabled"), bool)
            or health.get("finish_behavior") not in ("hold", "open_reset")
            or not isinstance(health.get("task"), str) or not health["task"].strip()):
        raise RuntimeError("Wrong or previously used configured skill service; no implicit replay")


def execute_task(robot, cameras, directory, request, config, health):
    """Run exactly one proposed sequence; request is injectable for offline tests."""
    directory = Path(directory)
    recording = config.get("recording_enabled", True)
    finish_behavior = config.get("finish_behavior", "open_reset")

    def persist(filename, value):
        if recording:
            save(directory / filename, value)

    outcome = {
        "status": "starting", "task": health["task"],
        "directory": str(directory) if recording else None,
        "task_success": "not_assessed; controller sequence completion is not object success",
        "reset": {"status": "not_attempted"}, "skills_executed": 0,
        "arm_speed_scale": config.get("arm_speed_scale", 1),
    }
    executor = None
    expected = []
    initial = None
    interrupted = False
    try:
        executor = SkillExecutor(
            robot, directory, cameras, recording=recording,
            arm_speed_scale=config.get("arm_speed_scale", 1),
        )
        initial = executor.state()
        persist("initial_state.json", initial)
        snapshot = cameras.request("snapshot")
        response = request("/predict", {
            "exp_id": directory.name, "actual_state": initial,
            "images": snapshot["images"], "camera_metadata": snapshot.get("metadata", {}),
        })
        if (response.get("protocol") != PROTOCOL or response.get("model") != config["model"]
                or response.get("status") != "planned" or response.get("exp_id") != directory.name
                or response.get("arm_speed_scale", 1) != config.get("arm_speed_scale", 1)
                or response.get("speed_contract", STANDARD_SPEEDS) != executor.arm_speeds):
            raise ValueError("Invalid configured model skill sequence response")
        persist("skill_sequence.json", response)
        expected = plan_sequence(initial["actual_pose"], response["skills"])
        persist("expected_poses.json", {
            "source": "deterministic skill geometry, not model output",
            "initial_pose": initial["actual_pose"], "sequence": expected,
        })
        current = executor.state()
        # Operator disabled only this local inference-drift rejection, 2026-11-12.
        # Keep the live state read and native readiness/protection checks above.
        # if (math.dist(initial["actual_pose"]["position_m"],
        #               current["actual_pose"]["position_m"]) > 0.003
        #         or abs(sum(a * b for a, b in zip(
        #             initial["actual_pose"]["quaternion_wxyz"],
        #             current["actual_pose"]["quaternion_wxyz"],
        #         ))) < math.cos(0.03 / 2)
        #         or abs(initial["gripper"]["width_m"] - current["gripper"]["width_m"]) > 0.003):
        #     raise RuntimeError("Robot changed during inference; one-shot plan rejected")
        for planned in expected:
            result = executor.execute_skill(planned)
            outcome["skills_executed"] += 1
            if recording:
                print(json.dumps({"skill_result": result}, ensure_ascii=False), flush=True)
        outcome["status"] = "sequence_completed"
    except KeyboardInterrupt as exc:
        interrupted = True
        outcome.update(status="interrupted", error=str(exc))
    except (RuntimeError, OSError, ValueError, KeyError, TypeError) as exc:
        outcome.update(status="failed", error=type(exc).__name__ + ": " + str(exc))
    finally:
        persist("outcome.json", outcome)
        if executor is not None:
            try:
                if outcome["status"] != "sequence_completed":
                    if executor.owns_writer:
                        executor.stop_owned()
                if finish_behavior == "hold":
                    outcome["reset"] = {
                        "status": "not_requested", "reason": "Hold until explicit operator open/reset",
                    }
                elif interrupted:
                    outcome["reset"] = {
                        "status": "blocked", "reason": "Operator interruption: no new automatic motion",
                    }
                else:
                    after = executor.finish_reset()
                    persist("after_reset_state.json", after)
                    outcome["reset"] = {"status": "succeeded", "actual_pose": after["actual_pose"]}
            except KeyboardInterrupt:
                interrupted = True
                if executor.owns_writer:
                    executor.stop_owned()
                outcome["reset"] = {"status": "interrupted"}
            except (RuntimeError, OSError, ValueError, KeyError) as exc:
                if executor.owns_writer:
                    try:
                        executor.stop_owned()
                    except (RuntimeError, OSError) as stop_error:
                        outcome["stop_error"] = str(stop_error)
                outcome["reset"] = {"status": "blocked", "reason": str(exc)}
                print("OPEN/RESET BLOCKED: " + str(exc), file=sys.stderr, flush=True)
            persist("outcome.json", outcome)
    evidence = {
        "exp_id": directory.name, "outcome": outcome, "initial_state": initial,
        "planned_sequence": expected, "executed_skills": executor.records if executor else [],
        "coordinate_source": "V2 actual_pose (encoders/FK), not estimated by the model",
    }
    persist("execution_evidence.json", evidence)
    return outcome, evidence


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task", choices=("skills",), default="skills")
    parser.add_argument("--config", type=Path, default=ROOT / "halo_skills.yaml")
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    config = configuration(args.config)
    sys.path.insert(0, config["v2_sdk_path"])
    from teleop_v2 import Robot
    from v2_transport import request_json
    from v2_state import require_state

    if not config["policy_url"].endswith("/predict"):
        raise ValueError("policy_url must end with /predict")
    base_url = config["policy_url"].removesuffix("/predict")
    health = request_json(base_url + "/health")
    verify_health(health, config)
    config["recording_enabled"] = health["recording_enabled"]
    config["finish_behavior"] = health["finish_behavior"]
    config["arm_speed_scale"] = health.get("arm_speed_scale", 1)
    if config["recording_enabled"] and shutil.which("ffmpeg") is None:
        raise RuntimeError("ffmpeg is required for skill recording")
    if args.check:
        with Robot() as robot:
            require_state(robot.get_state(), robot.instance_id, robot.session_id)
        print(json.dumps({"ready": True, "robot_motion_sent": False, "policy": health}))
        return 0

    def interrupt(signum, frame):
        raise KeyboardInterrupt("Received signal %d" % signum)

    signal.signal(signal.SIGTERM, interrupt)
    run_root = Path(config["run_dir"])
    if config["recording_enabled"]:
        run_root.mkdir(parents=True, exist_ok=True)
        directory = Path(tempfile.mkdtemp(prefix=datetime.now().strftime("run_%Y%m%d_%H%M%S_"),
                                          dir=run_root))
    else:
        directory = run_root / ("test_" + uuid.uuid4().hex)

    def persist(filename, value):
        if config["recording_enabled"]:
            save(directory / filename, value)

    persist("policy_health.json", health)
    print("Skill task: " + health["task"], flush=True)
    print(("Recording directory: " + str(directory)) if config["recording_enabled"]
          else "NO RECORDING: no images/videos/skill/pose artifacts or memory updates.", flush=True)
    print("Finish behavior: " + config["finish_behavior"], flush=True)
    print("Arm speed scale: " + str(config["arm_speed_scale"]), flush=True)
    cameras = None
    evidence = {"exp_id": directory.name, "executed_skills": []}
    outcome = {"status": "startup_failed",
               "directory": str(directory) if config["recording_enabled"] else None}
    try:
        cameras = SkillRecorder(config, directory / "cameras")
        cameras.start()
        with Robot() as robot:
            outcome, evidence = execute_task(
                robot, cameras, directory,
                lambda endpoint, payload: request_json(
                    base_url + endpoint, payload, timeout=config["timeout_s"] + 45,
                ), config, health,
            )
            if config["finish_behavior"] == "hold" and outcome["status"] != "interrupted":
                try:
                    frame = cameras.snapshot_after(time.monotonic())
                    evidence["final_observation"] = {
                        "actual_state": robot.get_state(),
                        "images": frame["images"],
                    }
                except (RuntimeError, OSError, ValueError, KeyError) as exc:
                    outcome["final_observation_error"] = str(exc)
                    print("FINAL OBSERVATION UNAVAILABLE: " + str(exc), file=sys.stderr, flush=True)
    except KeyboardInterrupt:
        outcome.update(status="interrupted", error="Operator interrupted; no reset commanded")
    except (RuntimeError, OSError, ValueError, KeyError) as exc:
        outcome["error"] = type(exc).__name__ + ": " + str(exc)
    finally:
        persist("outcome.json", outcome)
        if cameras is not None:
            try:
                cameras.stop()
                cameras.encode()
            except (OSError, RuntimeError, subprocess.SubprocessError, KeyboardInterrupt) as exc:
                outcome["recording_error"] = str(exc) or type(exc).__name__
                print("RECORDING FINALIZATION FAILED: " + str(exc), file=sys.stderr, flush=True)
        evidence["outcome"] = outcome
        persist("execution_evidence.json", evidence)
        try:
            outcome["model_finalization"] = request_json(
                base_url + "/finish", evidence, timeout=config["timeout_s"] + 15,
            )
        except (OSError, ValueError, RuntimeError, KeyboardInterrupt) as exc:
            outcome["model_stop_error"] = str(exc) or type(exc).__name__
            print("MODEL SHUTDOWN NOT CONFIRMED: " + str(exc), file=sys.stderr, flush=True)
        persist("outcome.json", outcome)
        print(json.dumps(outcome, ensure_ascii=False), flush=True)
    return 0 if (outcome["status"] == "sequence_completed"
                 and outcome.get("reset", {}).get("status") in ("succeeded", "not_requested")
                 and "recording_error" not in outcome and "model_stop_error" not in outcome) else 1


if __name__ == "__main__":
    raise SystemExit(main())
