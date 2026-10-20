import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from gr3_compute_side.high_level_planner.schedule.schedule import schedule

class GR3ComputeSide(Node):
    def __init__(self):
        super().__init__('gr3_compute_side')

        # command_id: state
        self.command_status = {}

        self.locomotion_status_subscription = self.create_subscription(String, "locomotion_status", self.locomotion_status_callback, 10)
        self.manipulation_status_subscription = self.create_subscription(String, "manipulation_status", self.manipulation_status_callback, 10)

        self.locomotion_command_publisher = self.create_publisher(
            String, "locomotion_command", 10
        )
        self.manipulation_command_publisher = self.create_publisher(
            String, "manipulation_command", 10
        )

    def locomotion_status_callback(self, msg):
        status = json.loads(msg.data)
        command_id = status["id"]
        state = status["state"]
        self.command_status[command_id] = state

    def manipulation_status_callback(self, msg):
        status = json.loads(msg.data)
        command_id = status["id"]
        state = status["state"]
        self.command_status[command_id] = state

    def send_locomotion_command(self, command):
        msg = String()
        msg.data = json.dumps(command)
        self.command_status[command["id"]] = "PENDING"
        self.locomotion_command_publisher.publish(msg)

    def send_manipulation_command(self, command):
        msg = String()
        msg.data = json.dumps(command)
        self.command_status[command["id"]] = "PENDING"
        self.manipulation_command_publisher.publish(msg)


class CenterControl():
    def __init__(self, compute_node):
        self.compute_node = compute_node

    def wait_for_success(self, command_id):
        while rclpy.ok():
            rclpy.spin_once(
                self.compute_node,
                timeout_sec=0.1,
            )
            state = self.compute_node.command_status.get(command_id)
            if state == "SUCCESS":
                return True
            if state == "FAIL":
                return False

        return False

    def launch(self):
        for command_id, command in schedule.items():
            print(command_id, command)
            if 'walk' in command:
                self.compute_node.send_locomotion_command({
                    "id": command_id,
                    "target": "simpleTable",
                    "frame_id": "map",
                    "coordinate": [2, 1, 0, 0, 0, 1.57]
                })
            elif 'pick' in command:
                self.compute_node.send_manipulation_command({
                    "id": command_id,
                    "target": "cup",
                    "frame_id": "camera",
                    "coordinate": [0.3, -0.02, 0.1, 0.8, 1, 1.57]
                })

            success = self.wait_for_success(command_id)
            if not success:
                print(f"Command {command_id} failed. Schedule stopped.")
                break

            print(f"Command {command_id} completed.")
