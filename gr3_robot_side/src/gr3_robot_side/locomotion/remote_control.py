import time
from fourier_aurora_client import MoveCommandManager

class LocomotionRemoteControl():
    def __init__(self, client, sleep_interval):
        self.client = client
        self.sleep_interval = sleep_interval
        self.print_interval = 0.5

        self.move_command_manager = MoveCommandManager(
            robot_name="gr3"
        )

    def execute(self, key):
        result = False

        # walk
        if key == "w" or (key == "s") or key == "a" or key == "d" or key == "q" or key == "e":
            self.client.set_fsm_state(3)
            time.sleep(self.sleep_interval)
            self.client.set_velocity_source(2)
            if key == "w":
                v = [0.3, 0.0, 0.0]
            elif key == "s":
                v = [-0.2, 0.0, 0.0]
            elif key == "a":
                v = [0.0, 0.3, 0.0]
            elif key == "d":
                v = [0.0, -0.3, 0.0]
            elif key == "q":
                v = [0.0, 0.0, 0.2]
            elif key == "e":
                v = [0.0, 0.0, -0.2]
            result = self.walk(v)

        # stand_pose
        if key == "z" or key == "x" or key == "c" or key == "v" or key == "b" or key == "n":
            current = self.client.get_stand_pose()
            p = [current[0], current[1], current[2]]
            self.client.set_fsm_state(2)
            time.sleep(self.sleep_interval)
            if key == "z":
                p[0] = p[0] + 0.04
            elif key == "x":
                p[0] = p[0] - 0.04
            elif key == "c":
                p[1] = p[1] + 0.05
            elif key == "v":
                p[1] = p[1] - 0.05
            elif key == "b":
                p[2] = p[2] + 0.08
            elif key == "n":
                p[2] = p[2] - 0.08
            # p[0] = max(-0.10, min(0.10, p[0]))
            # p[1] = max(-0.10, min(0.10, p[1]))
            # p[2] = max(-0.10, min(0.10, p[2]))
            result = self.stand_pose(p)

        # move_head
        if key == "r" or key == "t" or key == "y" or key == "u":
            current = self.client.get_group_state("head", "position")
            yaw = current[0]
            pitch = current[1]
            self.client.set_fsm_state(3)
            time.sleep(self.sleep_interval)
            self.client.set_upper_fsm_state(4)
            time.sleep(self.sleep_interval)
            if key == "r":
                yaw = yaw + 0.2
            elif key == "t":
                yaw = yaw - 0.2
            elif key == "y":
                pitch = pitch + 0.2
            elif key == "u":
                pitch = pitch - 0.2
            yaw = max(-1.3963, min(1.3963, yaw))
            pitch = max(-0.5236, min(0.1483, pitch))
            result = self.move_head(yaw, pitch)

        return result

    def walk(self, v):
        duration = 2
        self.client.set_velocity(v[0], v[1], v[2], duration)
        time.sleep(duration)
        return True

    def stand_pose(self, p):
        self.client.set_stand_pose(p[0], p[1], p[2])
        timeout = 4.0
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

    def move_head(self, yaw, pitch):
        self.move_command_manager.joint_move_command(
            move_type=0,
            group_name="head",
            group_pos=[yaw, pitch],
            expect_vel=200,
        )
        command = self.move_command_manager.get_move_command()
        self.client.set_move_command(command)
        result = self.client.wait_groups_motion_complete(
            ["head"],
            print_interval=self.print_interval,
        )
        return result