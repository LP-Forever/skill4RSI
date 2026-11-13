from copy import deepcopy
def robot_state():
    return {
        "instance_id": "instance", "simulation_only": False,
        "control_active": True, "state": "HOLD", "active_operation_id": None,
        "active_gripper_operation_id": None,
        "owner_session_id": None, "owner_releasing": False, "recording_fault": False,
        "actual_pose": {"position_m": [0.4, 0, 0.4], "quaternion_wxyz": [0, 1, 0, 0]},
        "feedback": {
            "stale": False, "valid": True, "current_error_bits": "0",
            "external_estimates_available": True, "O_F_ext_hat_K": [0] * 6,
        },
        "contact_guard": {
            "enabled": True, "latched": False, "stage": "ARMED",
            "estimates_valid": True, "blocked": False,
        },
        "gripper": {
            "available": True, "stale": False, "fault_latched": False,
            "is_grasped": False, "width_m": 0.0795, "max_width_m": 0.0795,
        },
    }


class FakeOperation:
    accepted = {"accepted": True}
    operation_id = "session:1"

    def __init__(self, status="SUCCEEDED"):
        self.result_status = status

    def status(self):
        return {"terminal": True, "status": self.result_status}


class FakeRobot:
    instance_id = "instance"
    session_id = "session"

    def __init__(self):
        self.current = robot_state()
        self.calls = []
        self.force_after_move = None
        self.native_contact_fault = False
        self.grasp_confirmed = True
        self.operation_status = "SUCCEEDED"
        self.gripper = self
        self.closed = False

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.closed = True
        self.current["owner_session_id"] = None

    def get_state(self):
        return deepcopy(self.current)

    def move_pose(self, position, quaternion, **kwargs):
        self.calls.append(("pose", position[:], quaternion[:], kwargs))
        self.current["owner_session_id"] = self.session_id
        self.current["actual_pose"] = {
            "position_m": position[:], "quaternion_wxyz": quaternion[:],
        }
        if self.force_after_move is not None:
            self.current["feedback"]["O_F_ext_hat_K"] = self.force_after_move[:]
        if self.native_contact_fault:
            self.current["contact_guard"].update(latched=True, stage="CONTACT_LOCKED")
        return FakeOperation(self.operation_status)

    def grasp(self, **kwargs):
        self.current["owner_session_id"] = self.session_id
        self.calls.append(("grasp", kwargs))
        self.current["gripper"].update(
            is_grasped=self.grasp_confirmed, width_m=0.046,
        )
        return FakeOperation()

    def move(self, **kwargs):
        self.current["owner_session_id"] = self.session_id
        self.calls.append(("open", kwargs))
        self.current["gripper"].update(is_grasped=False, width_m=kwargs["width_m"])
        return FakeOperation()

    def stop_all(self):
        self.calls.append(("stop",))
        return self.get_state()

