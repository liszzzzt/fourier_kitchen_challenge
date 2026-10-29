import sys
import termios
import tty
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

def read_key(prompt=""):
    print(prompt, end="", flush=True)

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        tty.setcbreak(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(
            fd,
            termios.TCSADRAIN,
            old_settings,
        )

    print(key)
    return key.lower()

class RemoteControl:
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
        LOCOMOTION_KEYS = {
            "w", "s", "a", "d", "q", "e",
            "z", "x", "c", "v", "b", "n",
            "r", "t", "y", "u",
        }
        MANIPULATION_KEYS = {
            "1", "2", "3", "4", "5", "6",
            "7", "8", "9", "0", "-", "=",
        }

        command_id = 1
        while True:
            key = read_key()
            msg = {
                "id": command_id,
                "target": "",
                "frame_id": "map",
                "coordinate": [],
                "type": "remote",
                "key": key,
                "action": "",
                "level": "",
            }
            if key in LOCOMOTION_KEYS:
                self.compute_node.send_locomotion_command(msg)
            elif key in MANIPULATION_KEYS:
                self.compute_node.send_manipulation_command(msg)
            elif key == 'p':
                # test
                msg = {
                    "id": command_id,
                    "level": 1,
                    "type": "posture",
                    "action": "crouch",
                }
                self.compute_node.send_locomotion_command(msg)
            else:
                print(f"Unknown key: {key}")
                continue
            success = self.wait_for_success(command_id)
            if success:
                print(f"Command {command_id} completed. Key: {key}")
            else:
                print(f"Command {command_id} failed. Key: {key}")
            command_id = command_id + 1
