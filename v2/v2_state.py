"""Read-only V2 feedback checks and quaternion multiplication; no SDK / robot construction."""
import math
from model_io import number
def multiply_quaternions(a, b):
    w, x, y, z = a
    v, i, j, k = b
    result = [
        w * v - x * i - y * j - z * k,
        w * i + x * v + y * k - z * j,
        w * j - x * k + y * v + z * i,
        w * k + x * j - y * i + z * v,
    ]
    norm = math.sqrt(sum(value * value for value in result))
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("Invalid cumulative quaternion")
    return [value / norm for value in result]


def require_state(state, instance_id, session_id, *, idle=True, gripper_busy=False):
    if state["instance_id"] != instance_id:
        raise RuntimeError("V2 instance changed; refusing to replay motion")
    if state["simulation_only"] or not state["control_active"]:
        raise RuntimeError("Real V2 control is not active")
    if state["feedback"]["stale"] or not state["feedback"]["valid"]:
        raise RuntimeError("V2 robot feedback is stale or invalid")
    if state["recording_fault"] or int(state["feedback"]["current_error_bits"]):
        raise RuntimeError("V2 recording or robot reports a fault")
    if state["owner_releasing"] or state["owner_session_id"] not in (None, session_id):
        raise RuntimeError("Another writer owns or is releasing the robot")
    if idle and (state["state"] != "HOLD" or state["active_operation_id"] is not None
                 or state["active_gripper_operation_id"] is not None):
        raise RuntimeError("V2 is not in idle HOLD")
    gripper = state["gripper"]
    if not gripper["available"] or gripper["fault_latched"]:
        raise RuntimeError("Native V2 gripper is unavailable or faulted")
    if not gripper_busy and gripper["stale"]:
        raise RuntimeError("Native V2 gripper feedback is stale")
    if not state["feedback"]["external_estimates_available"]:
        raise RuntimeError("External-force feedback is unavailable")
    force = state["feedback"]["O_F_ext_hat_K"]
    if not isinstance(force, list) or len(force) != 6 or not all(map(number, force)):
        raise RuntimeError("External-force feedback is malformed or nonfinite")
    # Native V2 owns contact thresholds, obstruction evidence and debounce.
    # Do not recreate a per-action force-delta threshold in the client.
    guard = state.get("contact_guard")
    if not isinstance(guard, dict) or guard.get("enabled") is not True:
        raise RuntimeError("V2 native contact protection is disabled or unavailable")
    if guard.get("latched") is True or guard.get("stage") in (
            "BRAKING", "RELEASING_REFERENCE", "CONTACT_LOCKED"):
        raise RuntimeError("V2 contact protection blocks further motion; no automatic clearing")
    if (guard.get("latched") is not False or guard.get("stage") != "ARMED"
            or guard.get("estimates_valid") is not True):
        raise RuntimeError("V2 native contact protection is not ready")

