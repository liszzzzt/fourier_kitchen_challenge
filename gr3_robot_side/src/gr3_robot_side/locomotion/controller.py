import time

class LocomotionController():
    def __init__(self, client, move_command_manager, sleep_interval):
        self.client = client
        self.move_command_manager = move_command_manager
        self.sleep_interval = sleep_interval

    def move_to(self, coordinate):
        # coordinate = [2, 1, 0, 0, 0, 1.57]
        self.client.set_fsm_state(3)
        time.sleep(self.sleep_interval)
        self.client.set_velocity_source(2)

        duration = 3
        self.client.set_velocity(0.2, 0.0, 0.0, duration)
        time.sleep(duration)
        # stop
        duration = 1
        self.client.set_velocity(0.0, 0.0, 0.0, duration)
        time.sleep(duration)

        return True

