"""Task-side camera IPC and V2 operation monitoring only; no original robot controller."""
import json, math, os, select, subprocess, sys, time
from pathlib import Path
from urllib.error import HTTPError
from urllib.request import Request, urlopen
GRIPPER_REFRESH_WAIT_S = 0.2
MAX_BODY_BYTES = 32 * 1024 * 1024
def request_json(url, payload=None, timeout=10):
    request = Request(url)
    if payload is not None:
        request = Request(
            url, data=json.dumps(payload, allow_nan=False).encode(),
            headers={"Content-Type": "application/json"}, method="POST",
        )
    try:
        with urlopen(request, timeout=timeout) as response:
            raw = response.read(MAX_BODY_BYTES + 1)
    except HTTPError as exc:
        raise RuntimeError(
            "HTTP %d from %s: %s" % (exc.code, url, exc.read(1000).decode(errors="replace"))
        ) from exc
    if len(raw) > MAX_BODY_BYTES:
        raise ValueError("HTTP response exceeds the supported size")
    value = json.loads(raw)
    if not isinstance(value, dict):
        raise ValueError("HTTP response must be an object")
    return value


class Cameras:
    def __init__(self, config, directory):
        self.log = ((directory / "cameras.log").open("a") if directory is not None
                    else open(os.devnull, "w"))
        self.process = subprocess.Popen(
            [config["camera_python"], "-u", str(Path(__file__).with_name("camera_preview.py")),
             "--console-url", config["console_url"]],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=self.log,
        )

    def request(self, command):
        if self.process.poll() is not None:
            raise RuntimeError("House camera process exited; see cameras.log")
        self.process.stdin.write((json.dumps({"command": command}) + "\n").encode())
        self.process.stdin.flush()
        data, deadline = bytearray(), time.monotonic() + 10
        while not data.endswith(b"\n"):
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not select.select([self.process.stdout], [], [], remaining)[0]:
                raise TimeoutError("House camera response timed out")
            block = os.read(self.process.stdout.fileno(), 65536)
            if not block:
                raise RuntimeError("House camera process closed its output")
            data.extend(block)
            if len(data) > 32 * 1024 * 1024:
                raise ValueError("Camera response is too large")
        result = json.loads(data)
        if "error" in result:
            raise RuntimeError(result["error"])
        return result

    def start(self):
        deadline = time.monotonic() + 20
        while True:
            try:
                return self.request("snapshot")
            except RuntimeError as exc:
                if str(exc) != "Waiting for all four house cameras" or time.monotonic() >= deadline:
                    raise
                time.sleep(0.1)

    def close(self):
        try:
            if self.process.poll() is None:
                self.process.stdin.write(b'{"command":"stop"}\n')
                self.process.stdin.flush()
                try:
                    self.process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    self.process.terminate()
                    self.process.wait(timeout=5)
        finally:
            self.process.stdin.close()
            self.process.stdout.close()
            self.log.close()


class HouseExecutor:
    def __init__(self, robot, directory, cameras, gripper_threshold_m):
        from v2_state import require_state
        self.require_state = require_state
        self.robot, self.directory, self.cameras = robot, directory, cameras
        self.owns_writer = False
        initial = self.state()
        if not math.isfinite(gripper_threshold_m) or not 0 < gripper_threshold_m < initial["gripper"]["max_width_m"]:
            raise ValueError("Binary gripper threshold must be inside the SDK opening range")
        self.gripper_threshold_m = gripper_threshold_m
        self.previous_gripper = int(initial["gripper"]["width_m"] < gripper_threshold_m)
        self.holding = initial["gripper"]["is_grasped"]

    def log(self, event, **fields):
        with (self.directory / "operations.jsonl").open("a") as output:
            output.write(json.dumps(
                dict(host_monotonic_s=time.monotonic(), event=event, **fields), allow_nan=False,
            ) + "\n")

    def state(self):
        return self.read_state()

    def read_state(self, *, idle=True, gripper_busy=False, arm_motion=False):
        started = None
        while True:
            state = self.robot.get_state()
            self.require_state(state, self.robot.instance_id, self.robot.session_id,
                               idle=idle, gripper_busy=True)
            if self.owns_writer and state["owner_session_id"] != self.robot.session_id:
                raise RuntimeError("V2 writer ownership lost")
            if arm_motion and self.holding and not state["gripper"]["is_grasped"]:
                raise RuntimeError("Grasp lost during arm motion")
            gripper = state["gripper"]
            if gripper_busy or not gripper["stale"]:
                self.require_state(state, self.robot.instance_id, self.robot.session_id,
                                   idle=idle, gripper_busy=gripper_busy)
                if started is not None:
                    self.log("gripper_feedback_refreshed", waited_s=time.monotonic() - started,
                             gripper=gripper)
                return state
            now = time.monotonic()
            if started is None:
                started = now
                self.log("gripper_feedback_wait", gripper=gripper,
                         maximum_wait_s=GRIPPER_REFRESH_WAIT_S)
            age, threshold = gripper["age_s"], gripper["stale_after_s"]
            if (not isinstance(age, (int, float)) or not math.isfinite(age) or age < 0
                    or not isinstance(threshold, (int, float))
                    or not math.isfinite(threshold) or threshold <= 0):
                raise RuntimeError("Invalid native gripper freshness timestamps")
            if (now - started >= GRIPPER_REFRESH_WAIT_S
                    or age >= threshold + GRIPPER_REFRESH_WAIT_S):
                self.log("gripper_feedback_timeout", waited_s=now - started, gripper=gripper)
                raise RuntimeError(
                    "Native V2 gripper feedback is stale: age=%.3fs, refresh wait=%.3fs"
                    % (age, now - started)
                )
            self.cameras.request("health")
            time.sleep(0.01)

    def wait(self, operation, *, arm_motion=False):
        self.owns_writer = True
        self.log("accepted", operation_id=operation.operation_id, accepted=operation.accepted)
        deadline = time.monotonic() + 180
        while True:
            self.cameras.request("health")
            result = operation.status()
            state = self.read_state(
                idle=False, gripper_busy=not arm_motion, arm_motion=arm_motion,
            )
            self.log("monitor", result=result, state=state)
            if result["terminal"]:
                if result["status"] != "SUCCEEDED":
                    raise RuntimeError("V2 operation failed: " + result["status"])
                return self.state()
            if time.monotonic() > deadline:
                raise TimeoutError("V2 operation exceeded the local deadline")
            time.sleep(0.1)


