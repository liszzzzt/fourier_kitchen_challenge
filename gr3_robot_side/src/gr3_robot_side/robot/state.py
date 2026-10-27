"""
robot/state.py —— 把机器人状态发给 compute_side（compute_side 在那边做正运动学、逆解、抓取判断）
=============================================================================================
    robot_state             std_msgs/String JSON，20 Hz
        {"base": [x, y, z, roll, pitch, yaw]（base_link 在 map 下）, "joints": {腰 3 + 头 2 + 右臂 7: rad},
         "stand_pose": [dz, pitch, roll], "fsm": "...", "fingers": [jaw_a, jaw_b]（夹片位置，m）| null}
    /gr3/joint_calib_states sensor_msgs/JointState，50 Hz：腰 3 + 头 2 的实测关节角（Aurora 的关节反馈，真机上就是编码器读数，
                            不是仿真的 qpos），header.stamp = 读数时刻。头部相机定位用：相机位姿 = FK(腰 + 头) · 标定外参，
                            compute_side 按图像的时间戳取最近时刻的这组关节角（vision/rgbd_localizer.py）
自己开一个节点和线程：manipulation_command 的回调会执行很久，主节点在这期间不处理别的消息。
"""
import json
import threading

from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from gr3_robot_side.robot.body import HEAD, WAIST

PERIOD = 0.05
CALIB_JOINTS_TOPIC = "/gr3/joint_calib_states"
CALIB_JOINTS_PERIOD = 0.02


class StatePublisher(Node):
    def __init__(self, body, hand):
        super().__init__("gr3_robot_state")
        self.body = body
        self.hand = hand
        self._pub = self.create_publisher(String, "robot_state", 10)
        self.create_timer(PERIOD, self._publish)
        self._calib_pub = self.create_publisher(JointState, CALIB_JOINTS_TOPIC, 10)
        self.create_timer(CALIB_JOINTS_PERIOD, self._publish_calib_joints)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self)
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _spin(self):
        try:
            self._executor.spin()
        except Exception:          # rclpy 关闭时（Ctrl+C）这里会抛 ExternalShutdownException，正常退出
            pass

    def _publish(self):
        try:
            state = {"base": self.body.base_pose(),
                     "joints": {j: float(v) for j, v in self.body.joints().items()},
                     "stand_pose": [float(v) for v in self.body.c.get_stand_pose()],
                     "fsm": self.body.c.get_fsm_name(),
                     "fingers": self.hand.fingers(timeout=0.0)}
        except Exception as error:   # Aurora 还没有数据时跳过这一拍
            self.get_logger().warn(f"robot_state: {error}", throttle_duration_sec=5.0)
            return
        msg = String()
        msg.data = json.dumps(state)
        self._pub.publish(msg)

    def _publish_calib_joints(self):
        stamp = self.get_clock().now().to_msg()
        try:
            q = list(self.body.c.get_group_state("waist", "position")) + list(self.body.c.get_group_state("head", "position"))
        except Exception as error:   # Aurora 还没有数据时跳过这一拍
            self.get_logger().warn(f"{CALIB_JOINTS_TOPIC}: {error}", throttle_duration_sec=5.0)
            return
        msg = JointState()
        msg.header.stamp = stamp
        msg.header.frame_id = "base_link"
        msg.name = WAIST + HEAD
        msg.position = [float(v) for v in q]
        self._calib_pub.publish(msg)

    def close(self):
        self._executor.shutdown()
        self.destroy_node()
