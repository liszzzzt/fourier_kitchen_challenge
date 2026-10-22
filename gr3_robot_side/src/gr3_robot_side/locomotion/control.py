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
        else:
            target = command["target"]
            coordinate = command["coordinate"]
            x, y, z, roll, pitch, yaw = map(float, coordinate)
            frame_id = command["frame_id"]

            coordinate = (x, y, z, roll, pitch, yaw)
            result = self.move_to(coordinate)
        return result

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

