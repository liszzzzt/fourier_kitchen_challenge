"""
manipulation/actions.py —— 原子动作的接口定义（只考虑右手平行夹爪；先只有签名，没有实现）
=====================================================================================
    move_to            末端在保持姿态自然的前提下运动到目标位姿（自由运动 / 走直线）；
                       目标可以直接给位姿，也可以给“物体 + 抓法”，由它算出抓取位姿
    move_constrained   夹着物体沿约束轨迹运动（现在只有铰链：开关铰链门）
    grasp              合上夹爪，检查有没有夹住物体
    release            张开夹爪
    push               不夹东西，用夹爪 / 手背沿某个方向推

约定：
  * 坐标统一用 map（世界）坐标，单位 m / rad。
  * 夹爪姿态用两个方向描述：pointing = 夹爪指向（手指朝哪），pinch = 两片夹片的连线方向。
  * 每个动作都返回一个结果对象（Result 的子类），不返回单个 bool：
      res.ok      是否完成（等价于 res.status is Status.OK）
      res.status  失败原因（Status），上层技能据此决定重试 / 换方向 / 重新定位
      其余字段    这个动作的测量值（到达位置、开口、转过的角度、推进的距离 ...）
"""
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple, Union

Vec3 = Tuple[float, float, float]          # map 坐标（m）或方向（不要求单位长度）


# ============================================================================ 结果
class Status(Enum):
    OK = "ok"                    # 完成
    UNREACHABLE = "unreachable"  # 逆解够不着（误差超过阈值）
    UNNATURAL = "unnatural"      # 有解，但姿态不自然（手从背后过来、手肘朝后等）
    JUMP = "jump"                # 途中某一步关节变化太大（要甩臂），停在半路
    DEVIATED = "deviated"        # 走直线时横向偏差超限，停在半路
    STALLED = "stalled"          # 手落后指令太多：被卡住 / 顶住
    EMPTY = "empty"              # 夹爪合到底，手里没东西
    WRONG_WIDTH = "wrong_width"  # 开口和期望宽度对不上
    OFF_CENTER = "off_center"    # 夹住了，但物体偏离夹爪中心太多
    NO_FEEDBACK = "no_feedback"  # 收不到夹爪 / 机器人状态


@dataclass
class Result:
    status: Status
    message: str = ""            # 给人看的说明（日志用）

    @property
    def ok(self) -> bool:
        return self.status is Status.OK


@dataclass
class MoveResult(Result):
    """move_to：OK / UNREACHABLE / UNNATURAL / JUMP / DEVIATED / NO_FEEDBACK"""
    reached: Optional[Vec3] = None       # 实际到达的捏取点位置；一步都没动是 None
    error: float = 0.0                   # 到达后离目标还差多少（m）
    steps_done: int = 0                  # path="linear" 时走完了几段


@dataclass
class HingeResult(Result):
    """move_constrained(kind="hinge")：OK（转完了 angle）/ STALLED / JUMP / NO_FEEDBACK"""
    angle: float = 0.0                   # 按手的轨迹算，实际转过的角度（rad，带符号）；失败时也是已转过的角度


@dataclass
class GraspResult(Result):
    """grasp：OK / EMPTY / WRONG_WIDTH / OFF_CENTER / NO_FEEDBACK"""
    opening: Optional[float] = None      # 合上后的开口（m）= 张开间距 − 两片各自向内移动的距离；收不到读数是 None
    offset: Optional[float] = None       # 两片夹片移动距离之差（m）：物体偏离夹爪中心多少


@dataclass
class ReleaseResult(Result):
    """release：OK / WRONG_WIDTH（没能张开）/ NO_FEEDBACK"""
    opening: Optional[float] = None      # 张开后的开口（m）


@dataclass
class PushResult(Result):
    """push：OK（推满 distance；until_blocked=True 时是顶住了）/ STALLED / UNREACHABLE / JUMP / NO_FEEDBACK"""
    pushed: float = 0.0                  # 实际推进了多少（m）
    blocked: bool = False                # 是不是因为顶住才停的


# ============================================================================ 目标
@dataclass
class Pose:
    """末端位姿：捏取点位置 + 夹爪指向 + 两片夹片连线方向（pinch 可省略：只约束指向）"""
    position: Vec3
    pointing: Vec3 = (0.0, 0.0, -1.0)    # 默认竖直朝下
    pinch: Optional[Vec3] = None


@dataclass
class Box:
    """方块 / 立方体：从正上方，夹片对准一组对面"""
    center: Vec3
    yaw: float                           # 绕竖直轴的朝向（rad）
    size: float                          # 边长（m）


@dataclass
class Cylinder:
    """柱体（杯子）：默认从侧面水平夹柱身，夹片连线垂直于柱轴；top=True 时从上方夹杯沿"""
    base_center: Vec3
    axis: Vec3
    radius: float
    height: float
    top: bool = False


@dataclass
class Slender:
    """细长条（刀、勺）：夹握把点，夹片连线垂直于长轴，从上方接近"""
    grip_point: Vec3                     # 握把位置
    long_axis: Vec3
    width: float                         # 握把截面在夹片方向上的宽度（m）


@dataclass
class Disc:
    """圆片（盘子）：夹离机器人最近的边缘点，夹片连线沿厚度方向，夹爪指向盘心"""
    center: Vec3
    normal: Vec3
    radius: float
    thickness: float


@dataclass
class BarHandle:
    """横杆把手（洗碗机门 / 柜门）：夹爪朝门（approach），夹片连线垂直于杆"""
    point: Vec3                          # 杆上要夹的点
    bar_axis: Vec3
    diameter: float
    approach: Vec3                       # 夹爪指向（朝门）


GraspObject = Union[Box, Cylinder, Slender, Disc, BarHandle]


@dataclass
class GraspTarget:
    """“物体 + 抓法”：move_to 由它算出抓取位姿；offset > 0 时沿夹爪指向往回退 offset（= 预抓取位）"""
    obj: GraspObject
    offset: float = 0.0


# ============================================================================ 动作
class Actions:
    """body 是 RobotLink（high_level_planner/robot_link.py），arm 是 Arm（manipulation/arm.py）"""

    def __init__(self, body, arm) -> None:
        ...

    def move_to(self, target: Union[Pose, GraspTarget], path: str = "free", step: float = 0.03,
                then: Optional[Vec3] = None, tol: float = 0.005,
                guard_xy: Optional[float] = None) -> MoveResult:
        """末端运动到 target，全程姿态自然。
        path      "free"：只保证终点（中间路径不受约束）；"linear"：捏取点沿直线走（接近 / 抬起 / 后退）
        step      path="linear" 时每段多长（m）
        then      接下来要直线走到的点：先按它选手臂构型，免得那段直线走到一半要甩臂
        tol       到位精度（m）：到达后按实测关节角修正到 tol 以内
        guard_xy  path="linear" 时每一段检查横向偏差，超过就停（DEVIATED），免得把物体推走"""
        ...

    def move_constrained(self, kind: str, axis_point: Vec3, axis_dir: Vec3, angle: float,
                         step: float = 0.05, stall: float = 0.03) -> HingeResult:
        """夹着物体沿约束轨迹运动，夹爪姿态保持不变（圆柱把手在夹爪里跟着转）。
        kind        现在只有 "hinge"（以后拉篮子再加 "slide"）
        axis_point  铰链轴上一点；axis_dir 轴的方向
        angle       要转多少（rad，右手定则绕 axis_dir）
        step        每段转多少（rad）
        stall       手落后目标超过多少（m）就算卡住（STALLED）"""
        ...

    def grasp(self, width: float, tol: float = 0.012, wait: float = 2.5,
              lift_check: float = 0.0, max_offset: Optional[float] = None) -> GraspResult:
        """合上夹爪，按开口判断有没有夹住。
        width       期望开口 = 物体在夹片方向上的宽度（m）
        tol         开口允许和 width 差多少（m）
        wait        合上后等多久再读数（s）
        lift_check  > 0 时：合上后竖直抬起这么多（m）再判断一次（排除“夹住了但一提就掉”）
        max_offset  两片移动距离之差超过它就算偏心（OFF_CENTER）；None 不检查"""
        ...

    def release(self, wait: float = 1.5) -> ReleaseResult:
        """张开夹爪，等 wait 秒后读开口确认"""
        ...

    def push(self, start: Pose, direction: Vec3, distance: float, stall: float = 0.015,
             back_off: Optional[float] = None, until_blocked: bool = False) -> PushResult:
        """不夹东西，从 start（先 move_to 过去）沿 direction 推，最多 distance（m）。
        stall          手落后指令超过多少（m）就算顶住
        back_off       顶住后退回多少（m），默认 stall / 2（别一直顶着）
        until_blocked  False：推满 distance 才算 OK，顶住是 STALLED；
                       True：目的就是推到顶（推关门、推回篮子），顶住返回 OK"""
        ...
