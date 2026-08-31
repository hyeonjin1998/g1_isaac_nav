"""Phase 2: Isaac Sim 쪽 ROS 2 입출력 (프로세스 A).

Isaac Sim 이 번들한 **cp311 빌드 rclpy** 를 사용합니다. 시스템 ROS 2(py3.10)를 source 한
셸에서는 임포트가 깨지므로 반드시 `scripts/isaac_env.sh` 로 환경을 잡고 실행하세요.

담당 범위
--------
저빈도 제어 신호만 Python 으로 처리합니다. 이미지 같은 고빈도 스트림은 OmniGraph
ROS2 브리지 노드(`ROS2CameraHelper` 등)로 보내야 프레임률이 유지됩니다.

    구독:  /cmd_vel        (geometry_msgs/Twist)     ← Nav2
    발행:  /odom           (nav_msgs/Odometry)
           odom→base_link, base_link→base_footprint  (tf2_msgs/TFMessage)
           /clock          (rosgraph_msgs/Clock)     ← use_sim_time 용

프레임 규약
----------
    map ──(RTAB-Map)── odom ──(여기)── base_link ── base_footprint
                                                 └─ camera_link, imu_link (OmniGraph)

``base_footprint`` 은 pelvis 를 지면에 투영하고 roll/pitch 를 제거한 프레임입니다.
보행 중 pelvis 가 상하 ±3cm, roll/pitch ±5° 진동하므로 Nav2 의 ``robot_base_frame`` 은
반드시 이쪽을 써야 코스트맵이 요동치지 않습니다.
"""

from __future__ import annotations

import math

import numpy as np

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped, Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage

CMD_VEL_TIMEOUT_S = 0.5
"""이 시간 동안 /cmd_vel 이 오지 않으면 정지 명령으로 간주합니다.

Nav2 가 죽거나 네트워크가 끊겼을 때 로봇이 마지막 명령으로 계속 걸어가는 것을 막습니다.
실기에서는 안전 필수 항목이라 시뮬에서도 동일하게 둡니다.
"""


class GaitCommandShim:
    """보행 개시(gait initiation) 데드밴드를 보정합니다.

    Phase 1 실측으로 확인된 정책 특성:

    ==========================  ==========================================
    서 있는 상태에서            큰 명령이 있어야 걷기 시작함
                                (vx 0.3 무반응 / 0.5 동작, wz 0.8 무반응 / 1.0 동작)
    걷고 있는 상태에서          작은 명령도 잘 추종함
                                (wz 0.4 단독은 무반응이지만 vx 0.4 와 함께면 10% 오차로 추종)
    ==========================  ==========================================

    **데드밴드는 축별이 아니라 "보행 중인가"에 걸려 있습니다.** 따라서 축별로 명령을
    부스트하는 것은 틀린 접근이고, *걷기 시작할 때만* 잠깐 명령을 키우면 됩니다.

    동작:
        정지 상태 + 0 아닌 명령  → 방향을 유지한 채 개시 임계값까지 스케일업 ("kick")
        보행 중                  → 원래 명령 그대로 통과
        명령 0                   → 그대로 0 (정지 허용)

    램프 인가는 무해함이 실측으로 확인됐으므로 Nav2 `velocity_smoother` 와 함께 써도 됩니다.
    """

    def __init__(
        self,
        init_vx: float = 0.5,
        init_vy: float = 0.5,
        init_wz: float = 1.0,
        walk_speed_on: float = 0.20,
        walk_speed_off: float = 0.08,
        eps: float = 1e-3,
    ) -> None:
        #: 각 축이 정지 상태에서 보행을 개시하는 데 필요한 크기 (Phase 1 실측)
        self.init_mag = np.array([init_vx, init_vy, init_wz], dtype=np.float32)
        #: 보행 중 판정 히스테리시스 — 경계에서 kick 이 채터링하는 것을 막습니다.
        self.walk_speed_on = walk_speed_on
        self.walk_speed_off = walk_speed_off
        self.eps = eps
        self._walking = False

    @property
    def walking(self) -> bool:
        return self._walking

    def update_state(self, lin_vel_b: np.ndarray, ang_vel_z: float) -> None:
        """실측 속도로 '보행 중' 상태를 갱신합니다. 정책 스텝마다 호출하세요."""
        speed = float(np.hypot(lin_vel_b[0], lin_vel_b[1])) + 0.3 * abs(float(ang_vel_z))
        if self._walking:
            if speed < self.walk_speed_off:
                self._walking = False
        elif speed > self.walk_speed_on:
            self._walking = True

    def __call__(self, cmd: np.ndarray) -> np.ndarray:
        out = np.asarray(cmd, dtype=np.float32).copy()
        out[np.abs(out) < self.eps] = 0.0

        if self._walking or not np.any(out):
            return out

        # 정지 상태에서 걷기를 시작해야 함 → 방향을 보존한 채 최소 스케일만 키웁니다.
        # 0 아닌 축 중 임계값에 가장 가까운 축이 임계값에 닿는 배율을 씁니다.
        ratios = [self.init_mag[i] / abs(out[i]) for i in range(3) if out[i] != 0.0]
        scale = min(ratios)
        if scale > 1.0:
            out = out * scale
        return out.astype(np.float32)


def yaw_to_quat(yaw: float) -> Quaternion:
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw * 0.5), w=math.cos(yaw * 0.5))


def quat_to_yaw(quat_wxyz: np.ndarray) -> float:
    w, x, y, z = (float(v) for v in quat_wxyz)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class G1RosBridge(Node):
    """Isaac Sim 루프에서 폴링 방식으로 쓰는 ROS 2 브리지.

    별도 스레드를 쓰지 않고 시뮬 스텝마다 :meth:`spin_once` 를 호출합니다.
    물리 스텝과 메시지 처리 순서를 결정적으로 유지하기 위해서입니다.
    """

    def __init__(
        self,
        odom_frame: str = "odom",
        base_frame: str = "base_link",
        footprint_frame: str = "base_footprint",
        publish_clock: bool = True,
    ) -> None:
        super().__init__("g1_isaac_bridge")

        self.odom_frame = odom_frame
        self.base_frame = base_frame
        self.footprint_frame = footprint_frame

        self._cmd = np.zeros(3, dtype=np.float32)
        self._last_cmd_time: float | None = None
        self._sim_time = 0.0
        self._cam_xyz: np.ndarray | None = None
        self._cam_quat: np.ndarray | None = None
        self._lidar_xyz: np.ndarray | None = None
        self._lidar_quat: np.ndarray | None = None

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        # /clock 은 늦게 뜬 노드도 즉시 받아야 하므로 transient local.
        clock_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, sensor_qos)
        self._odom_pub = self.create_publisher(Odometry, "/odom", sensor_qos)
        self._tf_pub = self.create_publisher(TFMessage, "/tf", sensor_qos)
        self._clock_pub = self.create_publisher(Clock, "/clock", clock_qos) if publish_clock else None

        self.get_logger().info("G1 ROS bridge 준비 완료 (sub: /cmd_vel, pub: /odom /tf /clock)")

    # ------------------------------------------------------------------ 입력

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._cmd[:] = (msg.linear.x, msg.linear.y, msg.angular.z)
        self._last_cmd_time = self._sim_time

    def get_command(self) -> np.ndarray:
        """최신 속도 명령. 워치독 타임아웃 시 0을 반환합니다."""
        if self._last_cmd_time is None:
            return np.zeros(3, dtype=np.float32)
        if self._sim_time - self._last_cmd_time > CMD_VEL_TIMEOUT_S:
            return np.zeros(3, dtype=np.float32)
        return self._cmd.copy()

    @property
    def command_is_stale(self) -> bool:
        if self._last_cmd_time is None:
            return True
        return self._sim_time - self._last_cmd_time > CMD_VEL_TIMEOUT_S

    # ------------------------------------------------------------------ 출력

    def _stamp(self):
        msg_time = rclpy.time.Time(seconds=self._sim_time)
        return msg_time.to_msg()

    def publish_clock(self, sim_time: float) -> None:
        self._sim_time = sim_time
        if self._clock_pub is not None:
            self._clock_pub.publish(Clock(clock=self._stamp()))

    def publish_odom(
        self,
        position: np.ndarray,
        quat_wxyz: np.ndarray,
        lin_vel_b: np.ndarray,
        ang_vel_b: np.ndarray,
    ) -> None:
        """odom→base_link 변환과 Odometry 메시지를 발행합니다.

        Args:
            position: 월드 좌표 (x, y, z)
            quat_wxyz: 월드 자세 (w, x, y, z)  ← Isaac 규약(scalar first)
            lin_vel_b: **바디 프레임** 선속도
            ang_vel_b: **바디 프레임** 각속도
        """
        stamp = self._stamp()
        qw, qx, qy, qz = (float(v) for v in quat_wxyz)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = float(position[0])
        odom.pose.pose.position.y = float(position[1])
        odom.pose.pose.position.z = float(position[2])
        odom.pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        # twist 는 child_frame_id(base_link) 기준이어야 합니다 — 월드 프레임을 넣으면
        # 로봇이 회전할 때 Nav2 와 RTAB-Map 이 서로 다른 추정을 하게 됩니다.
        odom.twist.twist.linear = Vector3(x=float(lin_vel_b[0]), y=float(lin_vel_b[1]), z=float(lin_vel_b[2]))
        odom.twist.twist.angular = Vector3(x=float(ang_vel_b[0]), y=float(ang_vel_b[1]), z=float(ang_vel_b[2]))
        self._odom_pub.publish(odom)

        tf_odom = TransformStamped()
        tf_odom.header.stamp = stamp
        tf_odom.header.frame_id = self.odom_frame
        tf_odom.child_frame_id = self.base_frame
        tf_odom.transform.translation = Vector3(
            x=float(position[0]), y=float(position[1]), z=float(position[2])
        )
        tf_odom.transform.rotation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        # base_link → base_footprint: 지면 투영 + roll/pitch 제거.
        # base_footprint 가 base_link 의 자식이므로 역변환을 넣습니다.
        yaw = quat_to_yaw(np.array([qw, qx, qy, qz]))
        tf_foot = TransformStamped()
        tf_foot.header.stamp = stamp
        tf_foot.header.frame_id = self.odom_frame
        tf_foot.child_frame_id = self.footprint_frame
        tf_foot.transform.translation = Vector3(x=float(position[0]), y=float(position[1]), z=0.0)
        tf_foot.transform.rotation = yaw_to_quat(yaw)

        self._tf_pub.publish(TFMessage(transforms=[tf_odom, tf_foot] + self._extra_tf(stamp)))

    def set_camera_transform(self, xyz: np.ndarray, quat_wxyz: np.ndarray) -> None:
        """base_link → d435_link 변환을 갱신합니다.

        카메라는 `torso_link` 에 붙어 있고 허리 관절(waist yaw/roll/pitch)이 구동되므로
        pelvis(=base_link) 기준 카메라 자세는 **고정이 아닙니다.** 정적 TF 로 두면
        허리가 움직일 때 포인트클라우드가 어긋나므로 매 스텝 갱신합니다.
        """
        self._cam_xyz = np.asarray(xyz, dtype=np.float64)
        self._cam_quat = np.asarray(quat_wxyz, dtype=np.float64)

    def set_lidar_transform(self, xyz: np.ndarray, quat_wxyz: np.ndarray) -> None:
        """base_link → mid360_link 변환을 갱신합니다.

        MID-360 도 `torso_link` 에 붙어 있어 허리 관절 구동에 따라 움직입니다.
        """
        self._lidar_xyz = np.asarray(xyz, dtype=np.float64)
        self._lidar_quat = np.asarray(quat_wxyz, dtype=np.float64)

    def _make_tf(self, stamp, parent: str, child: str, xyz, quat_wxyz) -> TransformStamped:
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = parent
        tf.child_frame_id = child
        tf.transform.translation = Vector3(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))
        qw, qx, qy, qz = (float(v) for v in quat_wxyz)
        tf.transform.rotation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        return tf

    def _extra_tf(self, stamp) -> list[TransformStamped]:
        out: list[TransformStamped] = []

        if self._lidar_xyz is not None and self._lidar_quat is not None:
            from ros_lidar import MID360_FRAME

            out.append(
                self._make_tf(stamp, self.base_frame, MID360_FRAME, self._lidar_xyz, self._lidar_quat)
            )

        if self._cam_xyz is None or self._cam_quat is None:
            return out

        from ros_camera import CAMERA_LINK_FRAME, COLOR_OPTICAL_FRAME, DEPTH_OPTICAL_FRAME, optical_frame_quat

        tf_cam = TransformStamped()
        tf_cam.header.stamp = stamp
        tf_cam.header.frame_id = self.base_frame
        tf_cam.child_frame_id = CAMERA_LINK_FRAME
        tf_cam.transform.translation = Vector3(
            x=float(self._cam_xyz[0]), y=float(self._cam_xyz[1]), z=float(self._cam_xyz[2])
        )
        qw, qx, qy, qz = (float(v) for v in self._cam_quat)
        tf_cam.transform.rotation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        out.append(tf_cam)

        # d435_link → 광학 프레임 (REP-103 정적 회전). 컬러/깊이 동일 위치로 둡니다
        # (실기 D435i 는 색·깊이 센서 간 baseline 이 있으나 시뮬은 단일 렌더 프로덕트).
        ow, ox, oy, oz = (float(v) for v in optical_frame_quat())
        for child in (COLOR_OPTICAL_FRAME, DEPTH_OPTICAL_FRAME):
            tf_opt = TransformStamped()
            tf_opt.header.stamp = stamp
            tf_opt.header.frame_id = CAMERA_LINK_FRAME
            tf_opt.child_frame_id = child
            tf_opt.transform.rotation = Quaternion(x=ox, y=oy, z=oz, w=ow)
            out.append(tf_opt)
        return out

    # ------------------------------------------------------------------ 루프

    def spin_once(self) -> None:
        rclpy.spin_once(self, timeout_sec=0.0)
