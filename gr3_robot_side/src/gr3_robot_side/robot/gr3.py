import time
import json
from fourier_aurora_client import AuroraClient
from fourier_aurora_client import MoveCommandManager

from gr3_robot_side.locomotion.control import LocomotionController
from gr3_robot_side.manipulation.control import ManipulationController


class FourierGR3:
    def __init__(self):
        # 0:init, 2:PdStand, 3:RL motion, 11:UpperBody manipulation
        self.fsm_state = 0
        self.client = AuroraClient.get_instance(domain_id=123, robot_name="gr3")
        self.move_command_manager = MoveCommandManager(robot_name='gr3')
        time.sleep(1)
        self.locomotion_c = LocomotionController(self.client, self.move_command_manager, 1)
        self.manipulation_c = ManipulationController(self.client, self.move_command_manager, 0.5)

    def launch(self):
        if self.fsm_state == 0:
            self.stand()
        self.start_position = self.client.get_base_data("pos_W")
        print(18, "start_position", self.start_position)

    def stand(self):
        self.fsm_state = 2
        self.client.set_fsm_state(self.fsm_state)
        time.sleep(1)

    def locomotion(self, msg):
        result = self.locomotion_c.execute(msg)
        return result

    def manipulation(self, msg):
        result = self.manipulation_c.execute(msg)
        return result

    def shutdown(self):
        self.fsm_state = 0
        self.client.set_fsm_state(self.fsm_state)
        time.sleep(1)
        self.client.close()

