import time
import json
from gr3_robot_side.manipulation.remote_control import ManipulationRemoteControl

class ManipulationController():
    def __init__(self, client, move_command_manager, sleep_interval):
        self.client = client
        self.move_command_manager = move_command_manager
        self.sleep_interval = sleep_interval
        self.print_interval = 0.5
        self.remote_control = ManipulationRemoteControl(self.client, self.move_command_manager, self.sleep_interval)

    def execute(self, msg):
        # {"id": 2, "target": "cup", "frame_id": "camera", "coordinate": [0.3, -0.02, 0.1, 0.8, 1, 1.57]}
        # {"id": 2, "target": "", "frame_id": "camera", "coordinate": [], "type": "remote", "key": "w"}
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
            result = self.touch(coordinate)
        return result

    def touch(self, coordinate):
        # target_coordinate = [0.3, -0.02, 0.1, 0.8, 1, 1.57]
        occupied_groups = ["left_manipulator", "right_manipulator"]

        self.client.set_fsm_state(3)
        time.sleep(self.sleep_interval)
        self.client.set_upper_fsm_state(4)
        time.sleep(self.sleep_interval)

        # [x, y, z, qx, qy, qz, qw]
        left_arm_pre_pose = [0.3, 0.25, 0.4, 0.0, -0.7071, 0.0, 0.7071]
        right_arm_pre_pose = [0.3, -0.25, 0.2, 0.0, -0.7071, 0.0, 0.7071]

        self.move_command_manager.cartesian_move_command(
            move_type=1,
            group_name="left_manipulator",
            group_pos=left_arm_pre_pose,
            expect_vel=300
        )
        self.move_command_manager.cartesian_move_command(
            move_type=1,
            group_name="right_manipulator",
            group_pos=right_arm_pre_pose,
            expect_vel=300
        )

        move_command = self.move_command_manager.get_move_command()
        self.client.set_move_command(move_command)
        result = self.client.wait_groups_motion_complete(occupied_groups, print_interval=self.print_interval)

        return result

