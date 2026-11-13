"""Read existing V2 WebRTC previews; never open cameras or command the robot."""

import argparse
import asyncio
import base64
import json
import sys
import time
from urllib.parse import urlparse
from urllib.request import Request, urlopen

import cv2

from camera_ids import CAMERAS


WAITING = "Waiting for all four house cameras"


def validate_status(status, generation=None):
    if status.get("schema") != "teleop-v2-console-1" or status.get("demo") is not False:
        raise ValueError("Expected the real V2 console")
    if generation is not None and status["generation"] != generation:
        raise RuntimeError("V2 preview generation changed; restart Halo")
    cameras = status["cameras"]
    if len(cameras) != 4 or {c["id"] for c in cameras} != set(CAMERAS):
        raise ValueError("V2 preview must contain exactly the four house cameras")
    now = time.monotonic_ns()
    result = {}
    for camera in cameras:
        name = camera["id"]
        if camera["identity"] != CAMERAS[name]:
            raise ValueError("V2 preview identity mismatch: " + name)
        fault = status["recording"]["cameras"].get(name, {}).get("error")
        if fault:
            raise RuntimeError("V2 preview source failed: %s: %s" % (name, fault))
        preview = camera["preview"]
        if preview is None:
            raise RuntimeError(WAITING)
        if (preview["width"], preview["height"]) != (640, 360):
            raise ValueError("Expected a 640x360 V2 preview: " + name)
        age = (now - preview["host_rx_mono_ns"]) / 1e9
        if not 0 <= age <= 0.5:
            raise RuntimeError("Stale V2 preview source: " + name)
        result[name] = camera
    return result


def console_json(url, payload=None):
    data = None if payload is None else json.dumps(payload).encode()
    request = Request(url, data=data, headers={"Content-Type": "application/json"})
    with urlopen(request, timeout=5) as response:
        raw = response.read(1024 * 1024 + 1)
    if len(raw) > 1024 * 1024:
        raise ValueError("V2 console response exceeds 1 MiB")
    return json.loads(raw)


class PreviewClient:
    def __init__(self, url):
        parsed = urlparse(url)
        if (parsed.scheme != "http" or parsed.hostname not in ("127.0.0.1", "localhost")
                or parsed.path not in ("", "/") or parsed.query or parsed.fragment):
            raise ValueError("V2 console URL must be a loopback HTTP origin")
        self.url = url.rstrip("/")
        self.pc = None
        self.generation = None
        self.latest, self.errors, self.tasks = {}, {}, []

    async def status(self):
        for attempt in range(3):
            status = await asyncio.to_thread(console_json, self.url + "/api/status")
            try:
                cameras = validate_status(status, self.generation)
                return status, cameras
            except RuntimeError as exc:
                if str(exc) != WAITING or attempt == 2:
                    raise
                await asyncio.sleep(0.02)

    async def open(self):
        from aiortc import RTCConfiguration, RTCPeerConnection, RTCSessionDescription

        status, _ = await self.status()
        self.generation = status["generation"]
        self.pc = RTCPeerConnection(RTCConfiguration(iceServers=[]))
        for _ in CAMERAS:
            self.pc.addTransceiver("video", direction="recvonly")
        await self.pc.setLocalDescription(await self.pc.createOffer())
        answer = await asyncio.to_thread(
            console_json, self.url + "/api/offer",
            {"sdp": self.pc.localDescription.sdp, "type": self.pc.localDescription.type},
        )
        mapping = {item["mid"]: item["id"] for item in answer["cameras"]}
        transceivers = self.pc.getTransceivers()
        if (answer["generation"] != self.generation or len(mapping) != 4
                or set(mapping.values()) != set(CAMERAS)
                or set(mapping) != {t.mid for t in transceivers}):
            raise ValueError("V2 WebRTC camera/MID mapping changed or is incomplete")
        await self.pc.setRemoteDescription(RTCSessionDescription(
            sdp=answer["sdp"], type=answer["type"],
        ))
        for transceiver in transceivers:
            track = transceiver.receiver.track
            if track is None or track.kind != "video":
                raise ValueError("Missing V2 video track")
            self.tasks.append(asyncio.create_task(self.receive(mapping[transceiver.mid], track)))

    async def receive(self, camera, track):
        origin = None
        previous = None
        try:
            while True:
                frame = await track.recv()
                now = time.monotonic_ns()
                if (frame.width, frame.height) != (640, 360) or frame.pts is None:
                    raise ValueError("Unexpected V2 decoded preview dimensions or timestamp")
                stamp = float(frame.pts * frame.time_base)
                if previous is not None and stamp <= previous:
                    raise ValueError("Non-increasing V2 preview timestamp")
                previous = stamp
                if origin is None:
                    origin = (now, stamp)
                presentation = origin[0] + round((stamp - origin[1]) * 1e9)
                if abs(now - presentation) > 500_000_000:
                    raise RuntimeError("V2 decoded preview is delayed: " + camera)
                ok, jpeg = cv2.imencode(".jpg", frame.to_ndarray(format="bgr24"))
                if not ok:
                    raise RuntimeError("Preview JPEG encoding failed")
                self.latest[camera] = {
                    "image": base64.b64encode(jpeg).decode("ascii"),
                    "metadata": {
                        "actual_identity": CAMERAS[camera], "width": 640, "height": 360,
                        "host_rx_mono_ns": now, "presentation_mono_ns": presentation,
                        "preview_pts_s": stamp, "source": "v2_web_rtc_preview",
                        "generation": self.generation,
                    },
                }
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            self.errors[camera] = type(exc).__name__ + ": " + str(exc)
            print("V2 PREVIEW FAILED %s: %s" % (camera, exc), file=sys.stderr, flush=True)

    async def snapshot(self, include_images=True):
        _, cameras = await self.status()
        if self.errors:
            raise RuntimeError("V2 preview receive failed: " + json.dumps(self.errors))
        if self.pc.connectionState in ("failed", "closed", "disconnected"):
            raise RuntimeError("V2 WebRTC connection lost")
        if set(self.latest) != set(CAMERAS):
            raise RuntimeError(WAITING)
        now = time.monotonic_ns()
        latest = dict(self.latest)
        for camera, item in latest.items():
            metadata = item["metadata"]
            if (not 0 <= now - metadata["host_rx_mono_ns"] <= 500_000_000
                    or abs(now - metadata["presentation_mono_ns"]) > 500_000_000):
                raise RuntimeError("Stale decoded V2 preview: " + camera)
        value = {"metadata": {
            camera: dict(item["metadata"],
                         source_stream={key: cameras[camera][key] for key in ("width", "height", "fps")},
                         source_preview=cameras[camera]["preview"])
            for camera, item in latest.items()
        }}
        if include_images:
            value["images"] = {camera: item["image"] for camera, item in latest.items()}
        return value

    async def close(self):
        for task in self.tasks:
            task.cancel()
        await asyncio.gather(*self.tasks, return_exceptions=True)
        if self.pc is not None:
            await self.pc.close()


async def serve(url, output):
    client = PreviewClient(url)
    try:
        await client.open()
        while True:
            line = await asyncio.to_thread(sys.stdin.readline)
            if not line:
                break
            try:
                command = json.loads(line)["command"]
                if command == "stop":
                    break
                if command not in ("snapshot", "health"):
                    raise ValueError("Unknown preview command")
                response = await client.snapshot(command == "snapshot")
            except (OSError, ValueError, RuntimeError, KeyError) as exc:
                response = {"error": str(exc)}
            output.write(json.dumps(response, allow_nan=False) + "\n")
            output.flush()
    except Exception as exc:
        print("V2 PREVIEW START FAILED: %s: %s" % (type(exc).__name__, exc),
              file=sys.stderr, flush=True)
        output.write(json.dumps({"error": str(exc)}) + "\n")
        output.flush()
        return 1
    finally:
        await client.close()
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--console-url", required=True)
    args = parser.parse_args()
    output = sys.stdout
    sys.stdout = sys.stderr
    cv2.setNumThreads(1)
    return asyncio.run(serve(args.console_url, output))


if __name__ == "__main__":
    raise SystemExit(main())
