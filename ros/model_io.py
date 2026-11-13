"""Responses-compatible model transport and JPEG validation only; no robot connection."""
import base64
import binascii
import json
import math
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen
from camera_ids import CAMERAS
from halo_model_usage import ModelUsage, collect_response
INSTRUCTIONS = ''  # Callers supply their V2-skill or ROS-chunk contract explicitly.
class PolicyError(Exception):
    def __init__(self, message, status=422):
        super().__init__(message)
        self.status = status


def number(value):
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(value)
    )


def decode_camera_images(images):
    if not isinstance(images, list) or len(images) != len(CAMERAS):
        raise PolicyError("Exactly four camera JPEGs are required", 400)
    decoded = []
    for image in images:
        if not isinstance(image, str):
            raise PolicyError("Images must be base64 JPEG strings", 400)
        try:
            raw = base64.b64decode(image, validate=True)
        except (ValueError, binascii.Error) as exc:
            raise PolicyError("Invalid base64 image", 400) from exc
        if not raw.startswith(b"\xff\xd8") or not raw.endswith(b"\xff\xd9"):
            raise PolicyError("Expected complete JPEG images", 400)
        if not 100 <= len(raw) <= 3 * 1024 * 1024:
            raise PolicyError("JPEG size outside supported bounds", 400)
        decoded.append(raw)
    return decoded


def request_prediction(base_url, model, effort, timeout, context, image_groups, *,
                       instructions=INSTRUCTIONS, raw_response_path=None,
                       max_output_tokens=6000, camera_ids=CAMERAS, stream=False,
                       usage_path=None, usage_callback=None, request_label="prediction"):
    monitor = ModelUsage(model, effort, timeout, max_output_tokens, path=usage_path,
                         callback=usage_callback, label=request_label)
    content = [{"type": "input_text", "text": json.dumps(context, ensure_ascii=False)}]
    for temporal_role, images in image_groups:
        for camera, image in zip(camera_ids, images, strict=True):
            content.extend([
                {"type": "input_text", "text": temporal_role + " camera=" + camera},
                {"type": "input_image", "image_url": "data:image/jpeg;base64," + image},
            ])
    body = {
        "model": model,
        "instructions": instructions,
        "input": [{"role": "user", "content": content}],
        "reasoning": {"effort": effort},
        "max_output_tokens": max_output_tokens,
        "stream": stream,
    }
    request = Request(
        base_url.rstrip("/") + "/responses",
        data=json.dumps(body).encode(),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urlopen(request, timeout=timeout) as response:
            if stream and "text/event-stream" in response.headers.get("Content-Type", ""):
                events = (Path(raw_response_path).with_suffix(".events.jsonl")
                          if raw_response_path is not None else None)
                result = collect_response(response, monitor, events)
            else:
                result = json.load(response)
                if isinstance(result, dict):
                    monitor.finish(result, terminal_seen=result.get("status") in (
                        "completed", "incomplete", "failed", "cancelled", "canceled"))
                else:
                    monitor.error("invalid_response", "Response is not an object")
    except HTTPError as exc:
        monitor.error("http_error", "HTTP " + str(exc.code))
        raise PolicyError("Model bridge returned HTTP " + str(exc.code), 502) from exc
    except (URLError, TimeoutError, OSError, ValueError) as exc:
        monitor.error("transport_or_stream_error", str(exc))
        raise PolicyError("Model bridge request failed: " + type(exc).__name__, 502) from exc
    if raw_response_path is not None:
        Path(raw_response_path).write_text(json.dumps(result, ensure_ascii=False), encoding="utf-8")
    if not isinstance(result, dict) or result.get("status") != "completed":
        reason = (result.get("incomplete_details") or {}).get("reason") if isinstance(result, dict) else None
        raise PolicyError("Model response did not complete" + (": " + reason if reason else ""), 502)
    if not isinstance(result.get("output"), list):
        raise PolicyError("Model response is missing output items", 502)
    parts = []
    for item in result["output"]:
        if not isinstance(item, dict):
            raise PolicyError("Invalid model output item", 502)
        if item.get("type") == "message":
            if item.get("status", "completed") != "completed":
                monitor.error("incomplete_message", "Message status is not completed")
                raise PolicyError("Model returned an unfinished message", 502)
            if not isinstance(item.get("content"), list):
                raise PolicyError("Invalid model message content", 502)
            for part in item["content"]:
                if not isinstance(part, dict):
                    raise PolicyError("Invalid model message part", 502)
                if part.get("type") == "output_text":
                    if not isinstance(part.get("text"), str):
                        raise PolicyError("Invalid model output text", 502)
                    parts.append(part["text"])
                else:
                    raise PolicyError("Model returned refusal or non-text content", 502)
        elif item.get("type") != "reasoning":
            raise PolicyError("Unexpected non-text model output", 502)
    text = "".join(parts)
    try:
        prediction = json.loads(text)
    except ValueError as exc:
        monitor.error("invalid_json", str(exc))
        raise PolicyError("Model did not return a complete JSON prediction", 502) from exc
    return prediction, result.get("usage", {}), text

