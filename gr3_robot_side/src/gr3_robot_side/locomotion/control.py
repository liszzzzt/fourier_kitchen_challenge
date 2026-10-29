import time
import json
from gr3_robot_side.locomotion.remote_control import LocomotionRemoteControl

class LocomotionController():
    def __init__(self, client, move_command_manager, sleep_interval):
        self.client = client
        self.move_command_manager = move_command_manager
        self.sleep_interval = sleep_interval
        self.remote_control = LocomotionRemoteControl(self.client, self.sleep_interval)

    def execute(self, msg):
        # {"id": 1, "target": "simpleTable", "frame_id": "map", "coordinate": [2, 1, 0, 0, 0, 1.57]}
        # {"id": 1, "target": "", "frame_id": "map", "coordinate": [], "type": "remote", "key": "w"}
        command = json.loads(msg.data)
        type = command.get("type", "")
        if type == "remote":
            result = self.remote_control.execute(command["key"])
        elif type =="posture":
            result = self.posture(command)
        else:
            target = command["target"]
            coordinate = command["coordinate"]
            x, y, z, roll, pitch, yaw = map(float, coordinate)
            frame_id = command["frame_id"]

            coordinate = (x, y, z, roll, pitch, yaw)
            result = self.move_to(coordinate)
        return result

    def posture(self, command):
        # {"id": 1, "level": 0.5, "type": "posture", "action": "crouch"}
        # {"id": 1, "level": 0, "type": "posture", "action": "lean_forward"}
        crouch_pose = {
            1: -0.22,
            0.5: -0.14,
            0: 0,
        }
        lean_forward_pose = {
            1: 0.65,
            0.5: 0.35,
            0: 0,
        }

        action = command["action"]
        level = float(command["level"])
        if level not in (0.0, 0.5, 1.0):
            return False
        current = self.client.get_stand_pose()
        p = [current[0], current[1], current[2]]

        if action == "crouch":
            p[0] = crouch_pose[level]
        elif action == "lean_forward":
            p[1] = lean_forward_pose[level]
        else:
            return False
        # execute
        self.client.set_fsm_state(2)
        time.sleep(self.sleep_interval)
        self.client.set_stand_pose(p[0], p[1], p[2])
        timeout = 10.0
        deadline = time.monotonic() + timeout
        tolerances = (0.005, 0.02, 0.02)

        while time.monotonic() < deadline:
            actual = self.client.get_stand_pose()
            height_error = abs(actual[0] - p[0])
            pitch_error = abs(actual[1] - p[1])
            yaw_error = abs(actual[2] - p[2])
            if height_error <= tolerances[0] and pitch_error <= tolerances[1] and yaw_error <= tolerances[2]:
                return True
            time.sleep(0.05)
        return False

    def move_to(self, coordinate):
        # coordinate = [2, 1, 0, 0, 0, 1.57]
        self.client.set_fsm_state(3)
        time.sleep(self.sleep_interval)
        self.client.set_velocity_source(2)

        duration = 8
        self.client.set_velocity(0.2, 0.0, 0.0, duration)
        time.sleep(duration)
        # stop
        duration = 1
        self.client.set_velocity(0.0, 0.0, 0.0, duration)
        time.sleep(duration)

        return True