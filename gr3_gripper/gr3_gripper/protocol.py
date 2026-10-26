"""Transport contract for the existing MuJoCo controller (no Aurora calls)."""
import json
import math
import time
import uuid


BACKEND = "simulation_mujoco"


def finite_number(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("Expected a JSON number")
    if not math.isfinite(value):
        raise ValueError("Expected a finite number")
    return float(value)


class GripperSession:
    """Used only by the simulator thread. One active command per gripper."""
    def __init__(self, controller, clock=time.time):
        if controller.path is not None:
            raise ValueError("Disable command_file when ROS2 owns this gripper")
        self.controller = controller
        self.clock = clock
        self.server_id = uuid.uuid4().hex
        self.active = None
        self.history = {}
        self.last_observation = None

    def status(self, command_id, state, **kwargs):
        return dict(version=1, id=command_id, state=state, backend=BACKEND,
                    server_id=self.server_id, robot_task_success=None, **kwargs)

    def request(self, raw):
        command_id = None
        try:
            if not isinstance(raw, str) or len(raw.encode("utf-8")) > 4096:
                raise ValueError("Command exceeds 4096 bytes")
            command = json.loads(raw)
            if not isinstance(command, dict):
                raise ValueError("Command must be a JSON object")
            command_id = command.get("id")
            if not isinstance(command_id, str) or not 1 <= len(command_id) <= 128:
                command_id = None
                raise ValueError("id must be a nonempty string, max 128 characters")
            if set(command) != {"version", "id", "server_id", "backend", "closure", "expires_at"}:
                raise ValueError("Missing or unexpected fields")
            if type(command["version"]) is not int or command["version"] != 1:
                raise ValueError("Unsupported protocol version")
            if command["server_id"] != self.server_id or command["backend"] != BACKEND:
                raise ValueError("Wrong bridge session/backend")
            closure = finite_number(command["closure"])
            if not 0 <= closure <= 1:
                raise ValueError("closure must be in [0, 1]")
            expiry = finite_number(command["expires_at"])
            now = self.clock()
            if not 0 < expiry - now <= 30:
                raise ValueError("Deadline expired or exceeds 30 seconds; check host clocks")
            fingerprint = json.dumps(command, sort_keys=True, allow_nan=False)
            if command_id in self.history:
                previous = self.history[command_id]
                if fingerprint != previous["fingerprint"]:
                    raise ValueError("id reused with different command")
                return dict(previous["status"], duplicate=True)
            if self.active is not None:
                raise ValueError("BUSY: wait for the current command to finish")
            self.history = {key: record for key, record in self.history.items()
                            if record["expiry"] > now}
            if len(self.history) >= 256:
                raise ValueError("Command history full; wait for deadlines to expire")
            self.controller.set_target(closure)
            result = self.status(command_id, "ACCEPTED", target_closure=closure)
            self.history[command_id] = dict(fingerprint=fingerprint, expiry=expiry, status=result)
            self.active = command_id
            return result
        except (ValueError, KeyError, TypeError) as exc:
            return self.status(command_id, "REJECTED", detail=str(exc))

    def sample(self, data):
        observation = self.controller.observation(data)
        observation["sim_time"] = float(data.time)
        # Reject nonfinite physics values before publishing JSON.
        json.dumps(observation, allow_nan=False)
        self.last_observation = observation
        if self.active is None:
            return self.status(None, "READY", observation=observation)
        command_id = self.active
        record = self.history[command_id]
        if self.clock() >= record["expiry"]:
            self.controller.set_target(self.controller.current)
            observation["target_closure"] = self.controller.target
            result = self.status(command_id, "TIMEOUT", observation=observation,
                                 detail="Ramp timed out; holding last commanded closure")
            self.active = None
        elif abs(self.controller.current - self.controller.target) <= 1e-9:
            result = self.status(command_id, "RAMP_COMPLETE", observation=observation,
                                 detail="Actuator command ramp complete; grasp is not confirmed")
            self.active = None
        else:
            result = self.status(command_id, "RAMPING", observation=observation)
        record["status"] = result
        return result

    def reset(self):
        old = self.status(self.active, "RESET", detail="Simulation reset; old commands invalidated")
        self.server_id = uuid.uuid4().hex
        self.active = None
        self.history.clear()
        self.last_observation = None
        return old

    def fail(self, error):
        self.controller.set_target(self.controller.current)
        result = self.status(self.active, "ERROR", detail=str(error))
        if self.active in self.history:
            self.history[self.active]["status"] = result
        self.active = None
        return result
