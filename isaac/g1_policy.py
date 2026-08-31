"""G1 velocity 정책 런타임 (시뮬레이터 비의존).

02_mjPDW 의 mjlab 에서 학습한 **default velocity 정책**(PDW 아님)을 Isaac Sim 에서
그대로 돌리기 위한 최소 런타임입니다. 관측 조립 → 정책 추론 → 관절 위치 타깃 산출까지만
담당하고, 물리/렌더링은 호출자가 처리합니다.

원본
----
    02_mjPDW/mjlab_PDW/deploy/robots/g1/config/policy/velocity/v0/
        exported/policy.onnx (+ .data)   ← 실기 배포 검증본
        params/deploy.yaml               ← 관절 순서·게인·obs 스펙

정책 구조 (ONNX 그래프 실측)
---------------------------
    obs(96) → (obs - mean) / std → Linear(96,512) → ELU
                                 → Linear(512,256) → ELU
                                 → Linear(256,128) → ELU
                                 → Linear(128,29) = actions

onnxruntime 의존성을 피하려고 ONNX 이니셜라이저에서 가중치만 뽑아 torch 로 재구성합니다.
(venv 에 onnx 는 있고 onnxruntime 은 없습니다.)

관측 레이아웃 (96, 모두 scale=1.0, history 없음)
---------------------------------------------
    [0:3]    base_ang_vel        pelvis 프레임 각속도
    [3:6]    projected_gravity   pelvis 프레임 중력 단위벡터
    [6:9]    velocity_commands   [vx, vy, wz]
    [9:38]   joint_pos_rel       q - q_default
    [38:67]  joint_vel_rel       qd
    [67:96]  last_action         직전 스텝의 raw action

액션
----
    q_target = action * action_scale + default_joint_pos
    50 Hz (step_dt=0.02) 로 갱신, 그 사이는 PD 가 유지
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import torch
import yaml

# mjlab XML(g1.xml) 과 Isaac USD(g1_29dof_rev_1_0.usd) 의 관절 이름·순서가 일치함을 확인함.
# deploy.yaml 의 배열(stiffness/damping/scale/default_joint_pos)은 모두 이 순서를 따릅니다.
G1_29DOF_JOINT_ORDER: tuple[str, ...] = (
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
)

NUM_JOINTS = len(G1_29DOF_JOINT_ORDER)
OBS_DIM = 3 + 3 + 3 + NUM_JOINTS * 3  # 96


def quat_rotate_inverse(quat_wxyz: np.ndarray, vec: np.ndarray) -> np.ndarray:
    """월드 프레임 벡터를 바디 프레임으로 회전 (R^T @ v).

    Isaac Sim 의 쿼터니언 규약은 scalar-first (w, x, y, z) 입니다.
    """
    w = quat_wxyz[0]
    q_vec = quat_wxyz[1:4]
    a = vec * (2.0 * w * w - 1.0)
    b = np.cross(q_vec, vec) * (2.0 * w)
    c = q_vec * (2.0 * float(np.dot(q_vec, vec)))
    return a - b + c


class _ActorMLP(torch.nn.Module):
    """ONNX 이니셜라이저에서 복원한 actor 네트워크."""

    def __init__(self, layers: list[tuple[np.ndarray, np.ndarray]], mean: np.ndarray, std: np.ndarray) -> None:
        super().__init__()
        # ONNX 이니셜라이저는 read-only 뷰라 복사해서 텐서로 만듭니다.
        self.register_buffer("mean", torch.from_numpy(np.array(mean, dtype=np.float32)))
        self.register_buffer("std", torch.from_numpy(np.array(std, dtype=np.float32)))

        mods: list[torch.nn.Module] = []
        for idx, (weight, bias) in enumerate(layers):
            lin = torch.nn.Linear(weight.shape[1], weight.shape[0])
            with torch.no_grad():
                lin.weight.copy_(torch.from_numpy(np.array(weight, dtype=np.float32)))
                lin.bias.copy_(torch.from_numpy(np.array(bias, dtype=np.float32)))
            mods.append(lin)
            if idx < len(layers) - 1:
                mods.append(torch.nn.ELU())
        self.net = torch.nn.Sequential(*mods)

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        return self.net((obs - self.mean) / self.std)


def load_actor_from_onnx(onnx_path: str | Path, device: str = "cpu") -> tuple[_ActorMLP, int, int]:
    """ONNX 파일에서 actor MLP 를 복원합니다.

    Returns:
        (모델, obs_dim, action_dim)
    """
    import onnx
    from onnx import numpy_helper

    model = onnx.load(str(onnx_path))
    init = {i.name: numpy_helper.to_array(i) for i in model.graph.initializer}

    # actor.{0,2,4,6}.{weight,bias} 형태를 인덱스 순으로 수집
    layer_ids = sorted(
        {int(name.split(".")[1]) for name in init if name.startswith("actor.") and name.endswith(".weight")}
    )
    if not layer_ids:
        raise ValueError(f"actor 레이어를 찾을 수 없습니다: {onnx_path}")
    layers = [(init[f"actor.{i}.weight"], init[f"actor.{i}.bias"]) for i in layer_ids]

    mean = init.get("normalizer._mean")
    if mean is None:
        mean = np.zeros((1, layers[0][0].shape[1]), dtype=np.float32)
    # Div 노드의 두 번째 입력이 표준편차. 익명 이니셜라이저 이름('add')로 저장돼 있어
    # 이름 대신 그래프에서 Div 의 입력을 역추적합니다.
    std = None
    for node in model.graph.node:
        if node.op_type == "Div" and len(node.input) == 2 and node.input[1] in init:
            std = init[node.input[1]]
            break
    if std is None:
        std = np.ones_like(mean)

    obs_dim = int(layers[0][0].shape[1])
    act_dim = int(layers[-1][0].shape[0])
    actor = _ActorMLP(layers, np.asarray(mean, np.float32), np.asarray(std, np.float32)).to(device)
    actor.eval()
    return actor, obs_dim, act_dim


@dataclass
class G1PolicyConfig:
    """deploy.yaml 에서 읽어온 런타임 파라미터."""

    step_dt: float
    stiffness: np.ndarray
    damping: np.ndarray
    default_joint_pos: np.ndarray
    action_scale: np.ndarray
    action_offset: np.ndarray
    cmd_lin_vel_x: tuple[float, float]
    cmd_lin_vel_y: tuple[float, float]
    cmd_ang_vel_z: tuple[float, float]

    @classmethod
    def from_yaml(cls, path: str | Path) -> "G1PolicyConfig":
        with open(path) as fh:
            cfg = yaml.safe_load(fh)

        act = cfg["actions"]["JointPositionAction"]
        ranges = cfg["commands"]["base_velocity"]["ranges"]

        def _arr(key: str, src: dict) -> np.ndarray:
            arr = np.asarray(src[key], dtype=np.float32)
            if arr.shape != (NUM_JOINTS,):
                raise ValueError(f"'{key}' 길이가 {arr.shape}, {NUM_JOINTS} 이어야 합니다")
            return arr

        return cls(
            step_dt=float(cfg["step_dt"]),
            stiffness=_arr("stiffness", cfg),
            damping=_arr("damping", cfg),
            default_joint_pos=_arr("default_joint_pos", cfg),
            action_scale=_arr("scale", act),
            action_offset=_arr("offset", act),
            cmd_lin_vel_x=tuple(ranges["lin_vel_x"]),
            cmd_lin_vel_y=tuple(ranges["lin_vel_y"]),
            cmd_ang_vel_z=tuple(ranges["ang_vel_z"]),
        )


class G1VelocityPolicy:
    """관측 조립 + 추론 + 액션 변환.

    사용 예::

        policy = G1VelocityPolicy(policy_dir, device="cuda")
        policy.set_command(vx, vy, wz)
        q_target = policy.step(root_quat_wxyz, ang_vel_w, joint_pos, joint_vel)
    """

    def __init__(self, policy_dir: str | Path, device: str = "cpu") -> None:
        policy_dir = Path(policy_dir)
        self.cfg = G1PolicyConfig.from_yaml(policy_dir / "deploy.yaml")
        self.actor, obs_dim, act_dim = load_actor_from_onnx(policy_dir / "policy.onnx", device=device)

        if obs_dim != OBS_DIM:
            raise ValueError(
                f"이 런타임은 history 없는 {OBS_DIM} 차원 관측을 가정합니다. "
                f"불러온 정책의 obs_dim={obs_dim} — history 가 있는 정책(예: 480)이라면 "
                f"history 버퍼 구현이 추가로 필요합니다."
            )
        if act_dim != NUM_JOINTS:
            raise ValueError(f"action_dim={act_dim}, {NUM_JOINTS} 이어야 합니다")

        self.device = device
        self._last_action = np.zeros(NUM_JOINTS, dtype=np.float32)
        self._command = np.zeros(3, dtype=np.float32)
        self._obs = np.zeros(OBS_DIM, dtype=np.float32)

    # ------------------------------------------------------------------ 명령

    def set_command(self, vx: float, vy: float, wz: float, clamp: bool = True) -> np.ndarray:
        """속도 명령 설정. 학습 범위를 벗어나면 정책이 추종하지 못하므로 기본 클램프."""
        cmd = np.array([vx, vy, wz], dtype=np.float32)
        if clamp:
            cmd[0] = np.clip(cmd[0], *self.cfg.cmd_lin_vel_x)
            cmd[1] = np.clip(cmd[1], *self.cfg.cmd_lin_vel_y)
            cmd[2] = np.clip(cmd[2], *self.cfg.cmd_ang_vel_z)
        self._command = cmd
        return cmd

    @property
    def command(self) -> np.ndarray:
        return self._command.copy()

    # ------------------------------------------------------------------ 추론

    def reset(self) -> None:
        self._last_action[:] = 0.0

    def build_obs(
        self,
        root_quat_wxyz: np.ndarray,
        ang_vel_w: np.ndarray,
        joint_pos: np.ndarray,
        joint_vel: np.ndarray,
    ) -> np.ndarray:
        """관측 벡터(96) 조립. 입력은 모두 G1_29DOF_JOINT_ORDER 순서여야 합니다."""
        base_ang_vel = quat_rotate_inverse(root_quat_wxyz, ang_vel_w)
        gravity_w = np.array([0.0, 0.0, -1.0], dtype=np.float32)
        projected_gravity = quat_rotate_inverse(root_quat_wxyz, gravity_w)

        obs = self._obs
        obs[0:3] = base_ang_vel
        obs[3:6] = projected_gravity
        obs[6:9] = self._command
        obs[9:38] = joint_pos - self.cfg.default_joint_pos
        obs[38:67] = joint_vel
        obs[67:96] = self._last_action
        return obs

    def step(
        self,
        root_quat_wxyz: np.ndarray,
        ang_vel_w: np.ndarray,
        joint_pos: np.ndarray,
        joint_vel: np.ndarray,
    ) -> np.ndarray:
        """한 스텝 추론 → 관절 위치 타깃(29) 반환."""
        obs = self.build_obs(root_quat_wxyz, ang_vel_w, joint_pos, joint_vel)
        with torch.no_grad():
            tensor = torch.from_numpy(obs).unsqueeze(0).to(self.device)
            action = self.actor(tensor).squeeze(0).cpu().numpy()
        self._last_action = action.astype(np.float32)
        return self._last_action * self.cfg.action_scale + self.cfg.action_offset

    @property
    def last_action(self) -> np.ndarray:
        return self._last_action.copy()


def build_joint_index_map(sim_joint_names: list[str]) -> np.ndarray:
    """시뮬레이터의 DOF 순서 → 정책 관절 순서 인덱스 맵.

    Isaac 의 articulation DOF 순서는 USD 선언 순서와 다를 수 있으므로 **반드시 이름으로**
    매핑합니다. 반환값 ``idx`` 에 대해 ``sim_values[idx]`` 가 정책 순서 배열이 됩니다.
    """
    lookup = {name: i for i, name in enumerate(sim_joint_names)}
    missing = [n for n in G1_29DOF_JOINT_ORDER if n not in lookup]
    if missing:
        raise ValueError(f"시뮬레이터 관절에서 찾을 수 없음: {missing}\n사용 가능: {sim_joint_names}")
    return np.array([lookup[n] for n in G1_29DOF_JOINT_ORDER], dtype=np.int64)
