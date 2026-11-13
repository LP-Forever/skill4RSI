"""Synthetic Responses events only: no upstream model or robot access."""

import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from model_io import PolicyError, request_prediction
from halo_model_usage import ModelUsage, collect_response


class Stream(io.BytesIO):
    headers = {"Content-Type": "text/event-stream; charset=utf-8"}

    def __init__(self, events):
        super().__init__(b"".join(("event: " + kind + "\ndata: " + json.dumps(value)
                                   + "\n\n").encode() for kind, value in events))


def message(text, status="completed"):
    return {"type": "message", "role": "assistant", "status": status,
            "content": [{"type": "output_text", "text": text}]}


class UsageTests(unittest.TestCase):
    def call(self, events, root, **kwargs):
        with patch("model_io.urlopen", return_value=Stream(events)) as url:
            result = request_prediction("http://unused/v1", "gpt-6.1-sol", "high", 300, {}, [],
                                        stream=True, max_output_tokens=12000,
                                        raw_response_path=root / "response.json",
                                        usage_path=root / "usage.json", **kwargs)
        self.assertEqual(url.call_count, 1)
        self.assertTrue(json.loads(url.call_args.args[0].data)["stream"])
        return result

    def test_completed_counts_reported_usage_without_double_counting_reasoning(self):
        text = '{"status":"planned","message":"fixture","skills":[{"op":"close"}]}'
        usage = {"input_tokens": 100, "output_tokens": 80, "total_tokens": 180,
                 "output_tokens_details": {"reasoning_tokens": 60},
                 "input_tokens_details": {"cached_tokens": 12}}
        final = {"status": "completed", "usage": usage, "output": [message(text)]}
        snapshots = []
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            value, actual_usage, _ = self.call([
                ("response.in_progress", {"response": {"status": "in_progress"}}),
                ("response.completed", {"response": final}),
            ], root, usage_callback=snapshots.append, request_label="planning")
            meter = json.loads((root / "usage.json").read_text())
            self.assertEqual(value["skills"], [{"op": "close"}])
            self.assertEqual(actual_usage, usage)
            self.assertEqual(meter["total_tokens"], 180)
            self.assertEqual(meter["reasoning_tokens"], 60)
            self.assertEqual(meter["non_reasoning_output_tokens"], 20)
            self.assertEqual(meter["cached_input_tokens"], 12)
            self.assertTrue(meter["usage_available"])
            self.assertTrue(meter["terminal_event_seen"])
            self.assertEqual(snapshots[-1]["request_label"], "planning")

    def test_incomplete_preserves_limit_reason_and_tokens_and_never_retries(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            events = [("response.output_text.delta", {"delta": '{"status":"planned",'}),
                      ("response.incomplete", {"response": {
                          "status": "incomplete", "incomplete_details": {"reason": "max_output_tokens"},
                          "usage": {"output_tokens": 12000,
                                    "output_tokens_details": {"reasoning_tokens": 11900}},
                          "output": []}})]
            with self.assertRaisesRegex(PolicyError, "max_output_tokens"):
                self.call(events, root)
            meter = json.loads((root / "usage.json").read_text())
            self.assertEqual(meter["state"], "incomplete")
            self.assertTrue(meter["output_limit_reached"])
            self.assertEqual(meter["reasoning_tokens"], 11900)
            self.assertEqual(json.loads((root / "response.json").read_text())["status"], "incomplete")
            self.assertIn("response.incomplete", (root / "response.events.jsonl").read_text())

    def test_early_eof_is_not_completed_and_unknown_tokens_are_not_zero(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            with self.assertRaisesRegex(PolicyError, "stream_ended_without_terminal_event"):
                self.call([("response.output_text.delta", {"delta": '{"status":"planned"}'})], root)
            meter = json.loads((root / "usage.json").read_text())
            self.assertFalse(meter["terminal_event_seen"])
            self.assertFalse(meter["usage_available"])
            self.assertIsNone(meter["output_tokens"])
            self.assertIsNone(meter["reasoning_tokens"])
            self.assertGreater(meter["output_text_characters"], 0)

    def test_completed_without_usage_remains_unknown(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            self.call([("response.completed", {"response": {
                "status": "completed", "output": [message('{"ready":true}')]}})], root)
            meter = json.loads((root / "usage.json").read_text())
            self.assertEqual(meter["state"], "completed")
            self.assertIsNone(meter["total_tokens"])
            self.assertFalse(meter["usage_available"])

    def test_output_item_done_is_authoritative_over_partial_deltas(self):
        text = '{"ready":true}'
        events = [("response.output_item.added", {"output_index": 0, "item": message("", "in_progress")}),
                  ("response.output_text.delta", {"output_index": 0, "delta": text[:5]}),
                  ("response.output_item.done", {"output_index": 0, "item": message(text)}),
                  ("response.completed", {"response": {"status": "completed"}})]
        with tempfile.TemporaryDirectory() as folder:
            value, _, _ = self.call(events, Path(folder))
            self.assertEqual(value, {"ready": True})

    def test_unfinished_message_is_rejected_even_when_outer_status_is_completed(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            with self.assertRaisesRegex(PolicyError, "unfinished message"):
                self.call([("response.completed", {"response": {
                    "status": "completed", "output": [message('{"ready":true}', "in_progress")]}})], root)
            self.assertEqual(json.loads((root / "usage.json").read_text())["state"], "incomplete_message")

    def test_failed_event_and_actual_zero_usage(self):
        monitor = ModelUsage("fixture", "high", 300, 12000)
        events = [("response.failed", {"response": {"status": "failed", "error": {"code": "fixture"},
                    "usage": {"input_tokens": 0, "output_tokens": 0, "total_tokens": 0}}})]
        result = collect_response(Stream(events), monitor)
        self.assertEqual(result["status"], "failed")
        self.assertTrue(monitor.data["usage_available"])
        self.assertEqual(monitor.data["output_tokens"], 0)
        self.assertEqual(monitor.data["upstream_error"]["code"], "fixture")


if __name__ == "__main__":
    unittest.main()
