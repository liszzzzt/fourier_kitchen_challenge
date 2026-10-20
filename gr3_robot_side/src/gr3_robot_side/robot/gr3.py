import time
import json
from fourier_aurora_client import AuroraClient
from fourier_aurora_client import MoveCommandManager

from gr3_robot_side.locomotion.controller import LocomotionController
from gr3_robot_side.manipulation.controller import ManipulationController


class FourierGR3:
    def __init__(self):
        # 0:init, 2:PdStand, 3:RL motion, 11:UpperBody manipulation
        self.fsm_state = 0
        self.client = AuroraClient.get_instance(domain_id=123, robot_name="gr3")
        self.move_command_manager = MoveCommandManager(robot_name='gr3')
        time.sleep(1)
        self.x = 0
        self.y = 0
        self.locomotion_c = LocomotionController(self.client, self.move_command_manager, 1)
        self.manipulation_c = ManipulationController(self.client, self.move_command_manager, 0.5)

    def launch(self):
        if self.fsm_state == 0:
            self.stand()
        # while self.fsm_state != 0:
        #     pass

    def stand(self):
        self.fsm_state = 2
        self.client.set_fsm_state(self.fsm_state)
        time.sleep(1)

    def locomotion(self, msg):
        # {"id": 1, "target": "simpleTable", "frame_id": "map", "coordinate": [2, 1, 0, 0, 0, 1.57]}
        command = json.loads(msg.data)
        target = command["target"]
        coordinate = command["coordinate"]
        x, y, z, roll, pitch, yaw = map(float, coordinate)
        frame_id = command["frame_id"]

        coordinate = (x, y, z, roll, pitch, yaw)
        result = self.locomotion_c.move_to(coordinate)

        return result

    def manipulation(self, msg):
        # {"id": 2, "target": "cup", "frame_id": "camera", "coordinate": [0.3, -0.02, 0.1, 0.8, 1, 1.57]}
        command = json.loads(msg.data)
        target = command["target"]
        coordinate = command["coordinate"]
        x, y, z, roll, pitch, yaw = map(float, coordinate)
        frame_id = command["frame_id"]

        coordinate = (x, y, z, roll, pitch, yaw)
        result = self.manipulation_c.touch(coordinate)

        return result

    def shutdown(self):
        self.fsm_state = 0
        self.client.set_fsm_state(self.fsm_state)
        time.sleep(1)
        self.client.close()

