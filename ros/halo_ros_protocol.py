"""ROS chunk contract. Importing this module never opens hardware or a model."""
import json
import math
from pathlib import Path
from urllib.request import Request, urlopen

ROOT = Path(__file__).resolve().parent
PROTOCOL = 'halo_ros_action_chunks_v1'
CAMERAS = ('kinect_0', 'kinect_1', 'realsense_0', 'realsense_1')
CHUNK_SIZE = 20


def config(path=None):
    value = json.loads(Path(path or ROOT / 'halo_ros_config.json').read_text())
    if value['protocol'] != PROTOCOL or value['chunk_size'] != CHUNK_SIZE:
        raise ValueError('Wrong ROS chunk configuration')
    for key in ('task_reference_set', 'memory_dir', 'task_prompt_file', 'run_dir'):
        value[key] = str((ROOT / value[key]).resolve())
    return value


def save(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n')


def request(url, payload=None, timeout=10):
    raw = None if payload is None else json.dumps(payload, allow_nan=False).encode()
    req = Request(url, data=raw, headers={'Content-Type': 'application/json'})
    with urlopen(req, timeout=timeout) as response:
        data = response.read(32 * 1024 * 1024 + 1)
    if len(data) > 32 * 1024 * 1024:
        raise ValueError('Response too large')
    return json.loads(data)


def finite(x):
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def validate(value, state):
    if (not isinstance(state, list) or len(state) != 8 or not all(map(finite, state))
            or abs(math.sqrt(sum(x*x for x in state[3:7])) - 1) > .01):
        raise ValueError('Expected ROS state [xyz, quaternion_xyzw, measured_width_m]')
    if not isinstance(value, dict) or set(value) != {'status', 'stage', 'message', 'act'}:
        raise ValueError('Wrong model output fields')
    if value['status'] not in ('continue', 'done', 'error'):
        raise ValueError('Wrong model status')
    if any(not isinstance(value[k], str) or not value[k].strip() for k in ('stage', 'message')):
        raise ValueError('Model evidence is required')
    actions = value['act']
    if value['status'] != 'continue':
        if actions != []:
            raise ValueError('Terminal response must contain no actions')
        return value
    if not isinstance(actions, list) or len(actions) != CHUNK_SIZE:
        raise ValueError('Expected exactly 20 action rows')
    position, length = list(state[:3]), 0.
    for row in actions:
        if not isinstance(row, list) or len(row) != 7 or not all(map(finite, row)):
            raise ValueError('Expected finite seven-dimensional action rows')
        if any(abs(x) > .05 for x in row[:3]) or any(abs(x) > .1 for x in row[3:6]):
            raise ValueError('Action exceeds existing per-axis translation/rotation contract')
        if row[6] not in (0, 1):
            raise ValueError('Gripper: exactly 1=open, 0=close')
        position = [a+b for a,b in zip(position, row[:3])]
        length += math.sqrt(sum(x*x for x in row[:3]))
        if not (.20 <= position[0] <= .65 and -.35 <= position[1] <= .35 and position[2] <= .60):
            raise ValueError('Cumulative waypoint exceeds existing Halo workspace')
    if length > .20 + 1e-9:
        raise ValueError('Chunk path exceeds existing 20 cm contract')
    return value
