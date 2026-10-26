"""Actual ROS2 transport, when Humble is installed. No mock rclpy modules."""
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import unittest


@unittest.skipUnless(importlib.util.find_spec("rclpy"), "ROS2 rclpy is not installed")
class ROS2Test(unittest.TestCase):
    def test_two_processes(self):
        env = dict(os.environ)
        env["PYTHONPATH"] = str(Path(__file__).resolve().parents[1]) + os.pathsep + env.get("PYTHONPATH", "")
        namespace = "/gripper_test_" + str(os.getpid())
        server = subprocess.Popen([sys.executable, "-m", "gr3_gripper.demo", "--namespace", namespace],
                                  env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            for closure in (0.8, 0.0):
                result = subprocess.run([sys.executable, "-m", "gr3_gripper.client",
                    "--namespace", namespace, "--closure", str(closure), "--timeout", "10"],
                    env=env, capture_output=True, text=True, timeout=25)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                message = json.loads(result.stdout.strip().splitlines()[-1])
                self.assertEqual(message["state"], "RAMP_COMPLETE")
                self.assertIsNone(message["robot_task_success"])
        finally:
            server.terminate()
            try:
                server.wait(timeout=5)
            except subprocess.TimeoutExpired:
                server.kill()
                server.wait()


if __name__ == "__main__":
    unittest.main()
