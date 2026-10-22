import time

class ManipulationRemoteControl():
    def __init__(self, client, move_command_manager, sleep_interval):
        self.client = client
        self.move_command_manager = move_command_manager
        self.sleep_interval = sleep_interval
        self.print_interval = 0.5

    def execute(self, key):
        result = False
        self.client.set_fsm_state(3)
        time.sleep(self.sleep_interval)
        self.client.set_upper_fsm_state(4)
        time.sleep(self.sleep_interval)

        if key in ["1", "2", "3", "4", "5", "6"]:
            occupied_group = "left_manipulator"
            pose = list(
                self.client.get_cartesian_state(occupied_group, "pose")
            )
            if key == "1":
                pose[0] = pose[0] + 0.04
            if key == "2":
                pose[0] = pose[0] - 0.04
            if key == "3":
                pose[1] = pose[1] + 0.04
            if key == "4":
                pose[1] = pose[1] - 0.04
            if key == "5":
                pose[2] = pose[2] + 0.03
            if key == "6":
                pose[2] = pose[2] - 0.03
            result = self.move_arm(occupied_group, pose)
        elif key in ["7", "8", "9", "0", "-", "="]:
            occupied_group = "right_manipulator"
            pose = list(
                self.client.get_cartesian_state(occupied_group, "pose")
            )
            if key == "7":
                pose[0] = pose[0] + 0.04
            if key == "8":
                pose[0] = pose[0] - 0.04
            if key == "9":
                pose[1] = pose[1] + 0.04
            if key == "0":
                pose[1] = pose[1] - 0.04
            if key == "-":
                pose[2] = pose[2] + 0.03
            if key == "=":
                pose[2] = pose[2] - 0.03
            result = self.move_arm(occupied_group, pose)
        return result

    def move_arm(self, occupied_group, pose):
        self.move_command_manager.cartesian_move_command(
            move_type=1,
            group_name=occupied_group,
            group_pos=pose,
            expect_vel=300
        )
        move_command = self.move_command_manager.get_move_command()
        self.client.set_move_command(move_command)
        result = self.client.wait_groups_motion_complete([occupied_group], print_interval=self.print_interval)
        return result