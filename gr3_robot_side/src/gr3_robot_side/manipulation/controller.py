import time

class ManipulationController():
    def __init__(self, client, move_command_manager, sleep_interval):
        self.client = client
        self.move_command_manager = move_command_manager
        self.sleep_interval = sleep_interval
        self.print_interval = 0.5

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
        time.sleep(self.print_interval)
        self.client.wait_groups_motion_complete(occupied_groups, print_interval=self.print_interval)
        time.sleep(self.print_interval)

        return True

