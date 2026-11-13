"""Responses SSE collection and reported-token telemetry; no model/robot calls."""

import copy
from datetime import datetime
import json
from pathlib import Path
import time


class ModelUsage:
    def __init__(self, model, effort, timeout, limit, *, path=None, callback=None, label="prediction"):
        self.path, self.callback = Path(path) if path is not None else None, callback
        self.started, self.last_write = time.monotonic(), 0
        self.raw_usage = {}
        self.data = {
            "request_label": label, "model": model, "reasoning_effort": effort,
            "timeout_s": timeout, "max_output_tokens": limit, "state": "connecting",
            "started_at": datetime.now().astimezone().isoformat(),
            "usage_source": "upstream reported values only; not inferred from characters",
            "usage_available": False, "input_tokens": None, "output_tokens": None,
            "total_tokens": None, "reasoning_tokens": None, "cached_input_tokens": None,
            "non_reasoning_output_tokens": None, "output_limit_reached": None,
            "response_status": None, "incomplete_reason": None,
            "terminal_event_seen": False, "event_count": 0, "output_text_characters": 0,
        }
        self.emit(force=True)

    def usage(self, value):
        if not isinstance(value, dict):
            return
        self.raw_usage.update(value)
        output_details = value.get("output_tokens_details")
        input_details = value.get("input_tokens_details")
        fields = {
            "input_tokens": value.get("input_tokens"), "output_tokens": value.get("output_tokens"),
            "total_tokens": value.get("total_tokens"),
            "reasoning_tokens": output_details.get("reasoning_tokens") if isinstance(output_details, dict) else None,
            "cached_input_tokens": input_details.get("cached_tokens") if isinstance(input_details, dict) else None,
        }
        for key, count in fields.items():
            if isinstance(count, int) and not isinstance(count, bool) and count >= 0:
                self.data[key] = count
        self.data["usage_available"] = any(self.data[k] is not None for k in fields)
        output, reasoning = self.data["output_tokens"], self.data["reasoning_tokens"]
        if output is not None:
            self.data["output_limit_reached"] = output >= self.data["max_output_tokens"]
        if output is not None and reasoning is not None and output >= reasoning:
            self.data["non_reasoning_output_tokens"] = output - reasoning

    def response(self, value):
        if not isinstance(value, dict):
            return
        self.usage(value.get("usage"))
        if value.get("status"):
            self.data["response_status"] = value["status"]
        reason = (value.get("incomplete_details") or {}).get("reason")
        if reason is not None:
            self.data["incomplete_reason"] = reason
            if reason == "max_output_tokens":
                self.data["output_limit_reached"] = True
        if value.get("error"):
            self.data["upstream_error"] = value["error"]

    def event(self, kind, value):
        self.data.update(state="streaming", last_event_type=kind,
                         last_event_at=datetime.now().astimezone().isoformat())
        self.data["event_count"] += 1
        if kind == "response.output_text.delta":
            self.data["output_text_characters"] += len(value.get("delta", ""))
        self.response(value.get("response"))
        self.usage(value.get("usage"))
        self.emit()

    def finish(self, result, *, terminal_seen):
        self.response(result)
        self.data.update(state=result.get("status", "unknown"), terminal_event_seen=terminal_seen)
        self.emit(force=True)

    def error(self, state, message):
        self.data.update(state=state, error=str(message))
        self.emit(force=True)

    def emit(self, force=False):
        now = time.monotonic()
        self.data["elapsed_s"] = round(now - self.started, 3)
        if self.callback is not None:
            self.callback(copy.deepcopy(self.data))
        if self.path is not None and (force or now - self.last_write >= 0.5):
            temporary = self.path.with_name(self.path.name + ".tmp")
            temporary.write_text(json.dumps(self.data, ensure_ascii=False, indent=2) + "\n")
            temporary.replace(self.path)
            self.last_write = now


def sse_events(response):
    event, data = None, []
    for raw in response:
        line = raw.decode("utf-8") if isinstance(raw, bytes) else raw
        line = line.rstrip("\r\n")
        if not line:
            if data:
                payload = "\n".join(data)
                if payload == "[DONE]":
                    return
                obj = json.loads(payload)
                if not isinstance(obj, dict):
                    raise ValueError("Responses SSE event must be an object")
                yield event or obj.get("type"), obj
            event, data = None, []
        elif line.startswith("event:"):
            event = line[6:].strip()
        elif line.startswith("data:"):
            data.append(line[5:].lstrip(" "))
    if data and "\n".join(data) != "[DONE]":
        obj = json.loads("\n".join(data))
        if not isinstance(obj, dict):
            raise ValueError("Responses SSE event must be an object")
        yield event or obj.get("type"), obj


def collect_response(response, monitor, events_path=None):
    """Never promote EOF, incomplete, failed or error to completed."""
    items, texts, final, terminal = {}, {}, None, False
    output = Path(events_path).open("w") if events_path is not None else None
    try:
        for kind, obj in sse_events(response):
            if output is not None:
                output.write(json.dumps({"event": kind, "data": obj}, ensure_ascii=False) + "\n")
                output.flush()
            monitor.event(kind, obj)
            index = obj.get("output_index", 0)
            if kind in ("response.output_item.added", "response.output_item.done"):
                items[index] = copy.deepcopy(obj.get("item") or {})
                if kind.endswith(".done"):
                    for content_index, part in enumerate(items[index].get("content", [])):
                        if part.get("type") == "output_text" and isinstance(part.get("text"), str):
                            texts[index, content_index] = part["text"]
            elif kind in ("response.output_text.delta", "response.output_text.done"):
                key = (index, obj.get("content_index", 0))
                if kind.endswith(".delta"):
                    texts[key] = texts.get(key, "") + obj.get("delta", "")
                else:
                    texts[key] = obj.get("text", texts.get(key, ""))
            elif kind in ("response.completed", "response.incomplete", "response.failed",
                          "response.cancelled", "response.canceled"):
                terminal = True
                final = copy.deepcopy(obj.get("response") or {})
                # The event itself is authoritative even when response.status is absent/wrong.
                final["status"] = kind.split(".", 1)[1]
                break
            elif kind in ("error", "response.error"):
                terminal = True
                final = {"status": "failed", "error": obj.get("error", obj)}
                break
    finally:
        if output is not None:
            output.close()
    if final is None:
        final = {"status": "incomplete",
                 "incomplete_details": {"reason": "stream_ended_without_terminal_event"}}
    if not final.get("output"):
        for (index, content_index), text in sorted(texts.items()):
            item = items.setdefault(index, {"type": "message", "role": "assistant"})
            content = item.setdefault("content", [])
            while len(content) <= content_index:
                content.append({})
            content[content_index] = {"type": "output_text", "text": text}
        final["output"] = [items[i] for i in sorted(items)]
    if not final.get("usage") and monitor.raw_usage:
        final["usage"] = monitor.raw_usage
    monitor.finish(final, terminal_seen=terminal)
    return final
