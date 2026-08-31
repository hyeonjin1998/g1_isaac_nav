# G1 Localization & Navigation 구축 계획 (05_Arch)

작성일: 2026-08-12
참고: [Isaac Sim ROS 2 Navigation Tutorial](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/ros2_tutorials/tutorial_ros2_navigation.html)

---

## 0. 환경 진단 (실측)

| 항목 | 실측값 | 영향 |
|---|---|---|
| OS | Ubuntu 22.04.5 (jammy) | ROS 2 **Humble** 확정 |
| GPU | RTX 5080 16GB | RGB-D 렌더링 + 정책 추론 동시 구동 가능 |
| Isaac Sim | 5.1.0.0 (pip, venv) | 튜토리얼은 6.0.1 기준이나 확장/노드 이름 동일 |
| Isaac Lab | 2.3.0 | 본 작업에서는 **불필요** (§2 참조) |
| venv Python | **3.11** | 시스템 ROS 2 Humble은 py3.10 → `import rclpy` 직접 불가 |
| ROS 2 | **미설치** (`/opt/ros` 없음) | Phase 0에서 설치 필요 (sudo) |
| Isaac 내장 rclpy | `isaacsim.ros2.bridge/humble/rclpy` — **cp311 빌드** | py 버전 충돌 우회 경로 확보 |
| 내장 메시지 | `nav_msgs`, `sensor_msgs`, `tf2_msgs`, `rosgraph_msgs`, `nav2_msgs`, `nav2_simple_commander` 모두 포함 | Isaac 쪽에서 Nav2 goal 전송까지 가능 |
| G1 USD | `03_IsaacPDW/unitree_model/G1/29dof/usd/g1_29dof_rev_1_0/` | 재사용 |

### 핵심 제약 3가지

1. **Python 3.11 vs 3.10 분리** — 이게 이 프로젝트의 가장 큰 함정입니다.
   `/opt/ros/humble/setup.bash`를 source한 상태로 Isaac Sim(py3.11)을 띄우면 py3.10용 rclpy가
   `PYTHONPATH`에 얹혀 충돌합니다. → **두 프로세스를 완전히 분리하고 DDS로만 통신**합니다.

2. **RTAB-Map DB는 실기 맵** — `~/Downloads/rtabmap.db` (224MB, 235 노드, rtabmap 0.23.7) 분석 결과:
   - RGB 1280×720 JPEG + depth (RVL 압축), **LaserScan 없음** → 순수 **RGB-D visual SLAM**
   - `Reg/Strategy:0`(Visual), `Reg/Force3DoF:false`(6DoF), `Kp/DetectorStrategy:8`(GFTT/ORB)
   - `Grid/3D:true`, `Grid/CellSize:0.05`, `Grid/Sensor:1`, `Grid/RangeMax:5.0`, `Optimizer/Strategy:2`(GTSAM)
   - `Mem/IncrementalMemory:true` (매핑 모드로 저장됨)
   - → **이 DB는 시뮬 맵으로 쓸 수 없습니다.** 시뮬용 맵은 동일 파이프라인으로 새로 만들고,
     기존 DB는 Phase 6(실기)의 타깃으로 둡니다. 파라미터 세트는 그대로 재사용합니다.

3. **하위 제어기는 humanoid RL 정책** — 튜토리얼의 `DifferentialController`가 설 자리가 없습니다.
   Nav2 → `/cmd_vel` → **ONNX 정책** → 29 관절 위치 명령이 그 자리를 대체합니다.

---

## 1. 사용할 정책 (default velocity policy)

02_mjPDW에서 PDW가 아닌 기본 velocity 태스크 정책을 확인했습니다. **두 후보는 구조가 다릅니다**
(ONNX 그래프 실측):

| 후보 | 경로 | obs | 특징 |
|---|---|---|---|
| **A. deploy 번들 v0** ← **채택** | `02_mjPDW/mjlab_PDW/deploy/robots/g1/config/policy/velocity/v0/` | **96** (history 없음) | `policy.onnx` + `deploy.yaml`. **실기 배포 검증본** |
| B. 최신 학습 run | `02_mjPDW/.../logs/rsl_rl/g1_velocity/2026-02-28_16-14-55/` | **480** (= 96 × history 5) | 속도 범위 더 넓음 |

**A 채택.** 처음엔 B로 시작하려 했으나 ONNX 입력 차원을 확인하고 뒤집었습니다:

- A는 history 버퍼가 없어 **관측 조립에서 틀릴 여지가 구조적으로 없음** (B는 history 5프레임의
  정렬 순서·term-major/frame-major 여부가 조용히 틀릴 수 있는 지점)
- `deploy.yaml`이 관절 순서·게인·스케일을 전부 명시 → 구현 명세서가 곧 실행 설정
- **실기 배포 검증본**이라 "시뮬 → 실기 이식" 목표와 정확히 일치
- 속도 범위가 더 보수적 → 내비게이션 안전 마진에 유리

B는 나중에 성능이 부족할 때의 업그레이드 경로로 남겨둡니다 (history 버퍼 구현 필요).

### 정책 런타임 스펙 (실측 확정)

```
obs (96) = base_ang_vel(3) + projected_gravity(3) + velocity_commands(3)
         + joint_pos_rel(29) + joint_vel_rel(29) + last_action(29)     ※ 모든 scale = 1.0
action (29) → q_target = action * scale + default_joint_pos
네트워크 = (obs − mean)/std → 96·512·256·128·29 MLP, ELU
step_dt = 0.02 s (50 Hz),  physics dt = 0.005 s,  decimation = 4
관절 순서 = 좌각6 → 우각6 → 허리3 → 좌팔7 → 우팔7 (Unitree SDK 순서)
```

**✅ 관절 이름·순서가 mjlab `g1.xml` 과 Isaac USD `g1_29dof_rev_1_0.usd` 에서 완전히 일치**
(둘 다 29개, 이름·순서 동일). 최대 위험 요소로 봤던 항목이 해소됐습니다. 다만 PhysX 런타임의
articulation DOF 순서는 USD 선언 순서와 다를 수 있어 **이름 기반 매핑**을 유지합니다.

**✅ onnxruntime 불필요** — 정책이 순수 MLP라 ONNX 이니셜라이저에서 가중치만 뽑아 torch로
재구성했습니다. 독립 numpy 구현과 대조해 **최대 오차 3.8e-6** 확인 (float32 정밀도 한계).
venv에 새 의존성을 추가하지 않았습니다.

### 속도 명령 범위 — **Nav2 설정에 그대로 반영해야 함**

| | **채택본 A (deploy v0)** | 참고: B |
|---|---|---|
| `lin_vel_x` | **−0.5 ~ 1.0** | −0.8 ~ 1.5 |
| `lin_vel_y` | **−0.5 ~ 0.5** | −0.8 ~ 0.8 |
| `ang_vel_z` | **−1.0 ~ 1.0** | −1.0 ~ 1.0 |

> **`vy ≠ 0` → G1은 Nav2 관점에서 holonomic 로봇입니다.** 컨트롤러는 omni를 지원하는
> **MPPI**를 씁니다 (RPP는 vy를 못 냅니다). 학습 범위를 넘는 명령은 정책이 추종하지 못하고
> 넘어지므로, Nav2 `max_vel_*`는 학습 범위의 **80% 수준**으로 잡습니다.

---

## 2. 아키텍처

Isaac Lab의 env 매니저 스택은 **쓰지 않습니다.** 학습이 아니라 단일 로봇 인터랙티브 구동이므로,
`isaacsim.robot.policy.examples`의 `PolicyController` 패턴(관절 상태 → obs 조립 → ONNX → PD 타깃)을
그대로 따르는 편이 훨씬 가볍고 ROS 브리지와 궁합이 좋습니다. Isaac Lab은 정책 검증(play) 용도로만
필요하면 씁니다.

```
┌──────────── 프로세스 A: Isaac Sim (venv py3.11, /opt/ros 미source) ────────────┐
│                                                                                │
│  Warehouse USD + G1 USD + RGB-D 카메라(머리) + IMU                              │
│                                                                                │
│  [Python] G1PolicyController  50Hz                                             │
│     obs(480) ← articulation 상태 + /cmd_vel                                    │
│     action(29) → PD joint targets                                              │
│                                                                                │
│  [Python/rclpy(내장 cp311)]        [OmniGraph ROS2 bridge]                     │
│     sub  /cmd_vel                     ROS2PublishClock      → /clock           │
│     pub  /odom                        ROS2CameraHelper      → rgb/depth/info   │
│                                       ROS2PublishTransformTree → /tf           │
│                                       ROS2PublishImu        → /imu             │
└────────────────────────────────┬───────────────────────────────────────────────┘
                                 │  DDS (FastRTPS, 동일 ROS_DOMAIN_ID)
┌────────────────────────────────┴─── 프로세스 B: ROS 2 Humble (py3.10) ─────────┐
│  rtabmap_ros  (mapping 모드 → 이후 localization 모드)   map→odom               │
│  nav2  (MPPI controller + Smac/NavFn planner + velocity_smoother) → /cmd_vel   │
│  rviz2                                                                         │
└────────────────────────────────────────────────────────────────────────────────┘
```

**고빈도 센서 스트림(이미지)은 OmniGraph 노드로**, 저빈도 제어 신호(`/cmd_vel`, `/odom`)는
Python rclpy로 처리합니다. 이미지를 Python으로 끌어오면 프레임률이 무너집니다.

### TF 트리

```
map ──(rtabmap)── odom ──(우리 odom publisher)── base_link ──┬── base_footprint
                                                             ├── camera_link → camera_*_optical_frame
                                                             └── imu_link
```

- `base_link` = pelvis
- **`base_footprint`(지면 투영, yaw만 유지)를 Nav2의 `robot_base_frame`으로 사용** — 보행 중
  pelvis가 상하 ±3cm, roll/pitch ±5° 진동하므로 pelvis를 직접 쓰면 코스트맵이 요동칩니다.
- footprint는 반경 0.30 m 원형 (G1 어깨폭 ≈ 0.45 m + 여유)

---

## 3. 디렉터리 구조 (05_Arch)

```
05_Arch/
├── docs/
│   ├── plan.md                     ← 본 문서
│   └── setup_notes.md              ← 환경변수·트러블슈팅 기록
├── isaac/                          # 프로세스 A (py3.11)
│   ├── g1_nav_sim.py               # 메인 standalone 스크립트
│   ├── g1_policy_controller.py     # ONNX 정책 러너 (deploy.yaml 파싱)
│   ├── ros_graph.py                # OmniGraph ROS2 노드 구성
│   ├── odometry.py                 # odom 산출 (GT → 추정 전환 가능)
│   ├── policy/velocity_v0/         # onnx + deploy.yaml 사본 (재현성 위해 복사)
│   ├── scenes/                     # 시뮬 씬 USD
│   └── tools/make_occupancy_map.py # omap 확장 headless 호출 (선택)
├── ros2_ws/src/                    # 프로세스 B (py3.10)
│   ├── g1_description/             # URDF/xacro (robot_state_publisher용)
│   ├── g1_bringup/launch/          # 통합 launch
│   ├── g1_localization/            # rtabmap mapping/localization launch + db
│   └── g1_navigation/              # nav2 params + maps
└── scripts/
    ├── isaac_env.sh                # 프로세스 A 환경변수
    └── ros_env.sh                  # 프로세스 B 환경변수
```

---

## Phase 0 — 사전 설치 및 DDS 상호운용 검증 🔶 진행 중

**목표: 두 Python 버전이 DDS로 서로 말이 통하는지 먼저 증명한다.** 여기서 막히면 이후 전부 막힙니다.

- [x] `scripts/isaac_env.sh` — venv + 내장 cp311 rclpy 경로 설정 (`/opt/ros` 미source)
- [x] `scripts/ros_env.sh` — 시스템 ROS 2 + 동일 `RMW_IMPLEMENTATION`/`ROS_DOMAIN_ID`
- [x] **Isaac 측 rclpy 동작 확인** — py3.11에서 `import rclpy` + 노드/퍼블리셔 생성 +
      DDS 루프백 pub/sub 왕복 성공 (상세: `docs/setup_notes.md`)
- [ ] ROS 2 Humble 설치 — `bash scripts/install_ros2.sh` (**일반 터미널에서 실행, sudo 필요**)
      ※ 이 머신에 ROS 2 apt 저장소가 미등록 상태였음. 스크립트가 `ros2-apt-source` .deb
        방식으로 저장소 등록부터 수행합니다.
- [ ] 프로세스 간(cp311 ↔ cp310) 통신 확인 — `scripts/dds_ping.py`

**검증**: A 터미널 `python scripts/dds_ping.py pub` → B 터미널 `ros2 topic echo /dds_ping` 수신.
(discovery 실패 시 양쪽 `ROS_LOCALHOST_ONLY`/`ROS_DOMAIN_ID` 일치 여부부터 확인)

---

## Phase 1 — Isaac Sim에서 G1이 `cmd_vel`로 걷게 만들기 (ROS 없이) ✅ 통과

**목표: 정책 이식이 맞았는지를 ROS 변수 없이 격리 검증.**

구현: `isaac/g1_policy.py` (시뮬 비의존 런타임) + `isaac/g1_walk_test.py` (Isaac 하네스)

### 결과 — `vx=0.5` 13초 주행

```
낙상 없음
pelvis 높이  평균 0.780 m (min 0.774)
월드 vx     +0.478 m/s (명령 +0.50)  → 추종 오차 4.3%   PASS
```

**MuJoCo→PhysX 전이가 무튜닝으로 성공.** 최대 리스크로 봤던 항목이 해소돼
03_IsaacPDW Isaac baseline 폴백은 불필요합니다.

### 실측으로 드러난 것 — 관절 순서는 **실제로 달랐음**

PhysX articulation 의 DOF 순서는 USD 선언 순서가 아니라 **운동학 트리의 BFS 순서**입니다:

```
시뮬 DOF:  0 left_hip_pitch   1 right_hip_pitch   2 waist_yaw     ← pelvis 직계 자식
           3 left_hip_roll    4 right_hip_roll    5 waist_roll    ← 다음 레벨
           6 left_hip_yaw     7 right_hip_yaw     8 waist_pitch
           ...
정책 순서:  좌각6 → 우각6 → 허리3 → 좌팔7 → 우팔7
```

29개 중 **27개의 인덱스가 어긋납니다.** 이름 기반 매핑(`build_joint_index_map`)이 없었다면
정책은 완전히 엉뚱한 관절을 구동했을 것이고, 증상은 "그냥 넘어짐"이라 원인 파악이 어려웠을 것입니다.
Phase 1을 ROS 없이 격리한 판단이 유효했습니다.

### 속도 명령 특성 — **Nav2 설계를 바꾸는 발견**

7개 명령 조건을 바디 프레임으로 실측한 결과 (각 12초 주행):

| 명령 (vx, vy, wz) | 실측 (vx, vy, wz) | 판정 |
|---|---|---|
| (0.5, 0, 0) | (**+0.493**, −0.002, −0.014) | 오차 1.4% |
| (1.0, 0, 0) | (**+0.971**, −0.024, −0.003) | 2.9% |
| (0, 0.3, 0) | (+0.003, **−0.001**, −0.003) | **무반응** |
| (0, 0.5, 0) | (+0.005, **+0.460**, −0.033) | 8.0% |
| (0, 0, 0.5) | (+0.003, +0.001, **+0.003**) | **무반응** |
| (0.4, 0, 0.4) | (**+0.366**, −0.003, **+0.358**) | 8.5% / 10.5% |
| (−0.3, 0, 0) | (**+0.004**, 0.000, +0.005) | **무반응** |

**패턴: 명령이 충분히 크면 정확히 추종하지만, 작은 단독 명령에는 아예 반응하지 않습니다.**

축별 **보행 개시 임계값**을 정밀 측정한 결과:

| 축 | 무반응 | 최초 반응 | 추종 품질 |
|---|---|---|---|
| `vx` 전진 | 0.1, 0.2, 0.3 | **0.5** (0.4 미측정) | 0.5→0.493 (1.4%), 1.0→0.971 (2.9%) — 우수 |
| `vx` 후진 | −0.3 | **−0.4** | −0.4→−0.284 (29%), −0.5→−0.389 (22%) — 부정확 |
| `vy` 측방 | 0.3, 0.35, 0.4, 0.45 | **0.5** | 0.5→0.460 (8%) |
| `wz` 제자리 회전 | 0.5, 0.6, 0.8 | **1.0** | 1.0→0.617 (**38%**) — 매우 부정확 |
| `wz` 보행 중 | — | **0.4 에서 동작** | 0.4→0.358 (10.5%) — 우수 |

**핵심 통찰: 데드밴드는 축별이 아니라 "보행 개시" 자체에 걸려 있습니다.**
`wz=0.4` 는 단독으로는 무반응이지만 `vx=0.4` 와 함께 주면 10% 오차로 잘 추종합니다.
즉 **일단 걷기 시작하면 모든 축이 잘 추종**하고, 서 있는 상태에서 보행을 개시하는 데만
큰 명령이 필요합니다.

**Nav2 통합에 대한 함의 (Phase 5 에 반영):**

1. **제자리 회전을 쓰지 말 것.** `wz` 단독은 1.0(학습 범위의 최대치)에서야 겨우 움직이고
   그마저 38% 오차입니다. 반면 전진 중 선회는 정확합니다 → **원호 주행 기반 컨트롤러**가 맞습니다.
2. **측방 이동(`vy`)은 사실상 사용 불가.** 임계값 0.5 가 학습 범위의 최대치라
   "0 아니면 0.5" 의 이진 동작입니다 → **Nav2 관점에서 비홀로노믹으로 취급**합니다.
3. 목표 근처의 미세 명령이 조용히 무시되어 "Nav2 는 명령을 보내는데 로봇은 정지" 교착 발생
   → 최소 속도를 임계값 위로 설정 + goal tolerance 를 넉넉히.
4. ~~`velocity_smoother` 램프가 정책을 서기 상태에 가둔다~~ → **반증됨.**
   램프 길이 0 / 0.5 / 2.0초 모두 동일한 결과 → 명령 인가 방식과 무관.
   `velocity_smoother` 는 그대로 써도 됩니다.
5. `backup` 복구 행동은 기본 크기(−0.15 m/s 등)로 동작하지 않습니다 → 비활성화 또는 크기 상향.

### 함정: Isaac Sim 의 종료 코드와 stdout 은 신뢰할 수 없음

`SimulationApp.close()` 가 `os._exit()` 를 호출해 **stdout 버퍼와 종료 코드를 모두 날립니다.**
첫 실행에서 예외가 났는데도 exit 0 에 출력이 전혀 없었습니다.
→ 라인 버퍼링 강제 + 결과를 JSON 파일로 이중 기록하도록 수정했습니다.
**이후 모든 Isaac 스크립트는 exit code 가 아니라 결과 파일로 판정합니다.**

---

## G1 기본 장착 센서 — 공식 URDF 실장 위치

`unitree_ros/robots/g1_description/g1_29dof_rev_1_0.urdf` (사용 중인 USD 와 동일 리비전)에서
확인한 값을 `isaac/sensors.py` 에 그대로 옮겼습니다.

| 센서 | 부모 | xyz [m] | rpy [rad] | 비고 |
|---|---|---|---|---|
| **D435i** (RGB-D) | `torso_link` | (0.0576235, 0.01753, 0.42987) | (0, **0.83078**, 0) | **아래 47.6°** |
| **MID-360** (3D LiDAR) | `torso_link` | (0.0002835, 0.00003, 0.41618) | (0, 0.04014, 0) | 아래 2.3° (거의 수평) |
| IMU (torso) | `torso_link` | (−0.03959, −0.00224, 0.14792) | (0,0,0) | |
| IMU (pelvis) | `pelvis` | (0.04525, 0, −0.08339) | (0,0,0) | 정책이 쓰는 IMU |

### ⚠ D435i 는 수평 아래로 47.6° 기울어져 있음

가슴에 달려 **발밑 지형을 보는 배치**입니다. 광축이 수평에서 47.6° 아래를 향하므로
원거리 전방 시야가 크게 제한됩니다 → Nav2 local costmap 의 유효 관측 거리와
`obstacle_max_range` 를 여기에 맞춰 보수적으로 잡아야 합니다.

### ⚠ MID-360 3D LiDAR 가 기본 장착되어 있음

기존 `rtabmap.db` 가 RGB-D 전용이라 카메라만 있는 것으로 가정했었으나, 공식 description 에는
**거의 수평(2.3°)으로 장착된 360° 3D LiDAR 가 포함**되어 있습니다. 내비게이션 관점에서는
47.6° 아래를 보는 카메라보다 훨씬 유리합니다 (전방위 커버리지, 원거리 장애물).
RTAB-Map 은 LiDAR 입력도 지원하므로 측위 방식을 바꾸지 않고 센서만 추가할 수 있습니다.
→ **채택했습니다.** 프레임워크 변경 없이 덧붙는 수준임을 확인했습니다:
Isaac 쪽은 센서 prim + writer + TF 하나씩, 제어 루프는 무변경.

#### MID-360 프로파일은 자체 제작 근사입니다

Isaac Sim 5.1 에 Livox 프로파일이 **동봉되어 있지 않아** `isaac/lidar_configs/Livox_MID360.json`
을 직접 만들었습니다.

| 항목 | 실기 MID-360 | 근사 프로파일 |
|---|---|---|
| 수평 FOV | 360° | 360° |
| 수직 FOV | 59° (−7~+52°) | 동일 (40채널, 1.51° 간격) |
| 포인트레이트 | 200,000 pts/s | 200,000 pts/s (발행 기준) |
| 스캔 주기 | 10 Hz | 프로파일 200 Hz / **발행 10 Hz** (아래 참조) |
| 최대 거리 | 40 m @10% | 40 m |
| **스캔 패턴** | **비반복 로제트** | **rotary 다중빔** ← 다름 |

Isaac 의 RTX lidar 는 rotary 모델만 지원하므로 **스캔 패턴은 재현되지 않습니다.**
커버리지·포인트레이트는 동등하니 매핑·장애물 검출에는 충분하지만,
비반복 스캔의 시간 누적 특성에 의존하는 알고리즘 평가에는 이 차이를 감안해야 합니다.

#### 센서 발행률을 실기에 맞춰야 함 (실측으로 발견)

매 물리 스텝(200 Hz)마다 렌더링하면 센서 writer 가 **렌더 프레임마다 발행**하므로
라이다가 실기의 4배 가까운 속도로 나갑니다. `--render-hz`(기본 10) 로 렌더를
데시메이션해 실기 사양에 맞췄습니다.

```
[render] 센서 렌더 10 Hz (물리 20 스텝마다)
/mid360/points  10 Hz(시뮬시간), width=41659 pts, frame=mid360_link
TF base_link→mid360_link  translation (−0.002, −0.003, 0.460), pitch 2.59°
                          → URDF 실장값(z=0.416 + torso 오프셋, 2.3°)과 부합
```

#### ⚠⚠ 라이다가 **두 번** 조용히 다른 센서가 되어 있었음 (2026-08-13 발견)

위의 `width=41659 pts` 는 그럴듯해 보였지만 실제로는 MID-360 이 아니었습니다.
`explored.db` 에 저장된 스캔을 디코딩해서야 드러났습니다.

**(1) 프로파일이 아예 적용되지 않았음.** isaac.log 에 경고 한 줄만 남습니다:

```
[Warning] Config 'Livox_MID360' not found for OmniLidar at .../mid360
```

Isaac Sim 5.x 의 `IsaacSensorCreateRtxLidar` 는 JSON 프로파일을 더 이상 보지 않고
하드코딩된 USD 에셋 목록(HESAI/Ouster/SICK 등 23종)만 인식합니다. `profileBaseFolder`
설정은 extension.toml 에 **"for (deprecated) camera-based Lidar"** 라고 명시돼 있습니다.
못 찾으면 예외가 아니라 **기본 라이다(Example_Rotary)로 대체**됩니다.

**(2) 메시지 하나에 1/20 회전만 담겼음.** Isaac 은 렌더된 프레임마다 라이다를
**`rendering_dt` 만큼만** 회전시킵니다. 이 프로젝트는 `rendering_dt = physics_dt = 5 ms`
이므로(제어 주기 유지 때문에 필수) 10 Hz 스캔이면 tick 당 1.8° 밖에 안 돕니다.
시뮬 시간은 100 ms 흘렀는데 센서는 5 ms 만 도는 불일치입니다.

| | 의도 | 실제 (실측) | 수정 후 (실측) |
|---|---|---|---|
| 고도각 | −7 ~ +52° | **−15 ~ +10°** (Example_Rotary) | −7.02 ~ +52.02° |
| 메시지당 방위각 | 360° | **24°** | 360° (로봇 장착 시 311°, 팔 가림) |
| 메시지당 점 개수 | 20,000 | ~16,000 (24° 조각) | 20,000 |

**맵에 미친 영향**: 노드마다 좁은 부채꼴만 들어가 장애물 셀이 있는 노드가
**550 중 120개(21.8%)** 뿐이었습니다 — 벽이 거의 없는 격자. 수정 후 같은 창고에서
164 중 115개(70.1%)로 회복했습니다.

**수정**

1. `ros_lidar.py`: 생성 시 `force_camera_prim=True` (JSON 프로파일이 살아 있는 유일한 경로)
   + 생성 직후 `sensorModelConfig` 를 검사해 **대체되면 즉시 예외**. 조용한 실패 금지.
2. `Livox_MID360.json`: `scanRateBaseHz` 10 → **200 = 1/rendering_dt**,
   `reportRateBaseHz` 5000 → 100000, `fireTimeNs` 1/20 로 축소.
   렌더 tick 당 정확히 1 회전을 돌게 맞춘 것이며, 발행은 렌더 주기(10 Hz)로 나가므로
   **출력은 실기와 동일한 20,000점/스캔 @10 Hz** 입니다.
   대신 한 스캔이 시간에 걸쳐 훑히지 않고 **순간 스냅샷**이 됩니다
   (`MotionBVH` 가 어차피 꺼져 있어 원래도 왜곡은 없었습니다).
   **`physics_dt` 를 바꾸면 이 값도 같이 바꿔야 합니다.**

**회귀 검사** — 같은 실수를 다시 조용히 하지 않도록 두 개를 추가했습니다:

```bash
python isaac/check_lidar_profile.py     # 닫힌 방에서 센서 원형 검사 (360°, −7~+52°)
python3 scripts/check_lidar_topic.py    # 발행된 /mid360/points 검사 (조각 발행 감지)
```

### 좌표 규약 변환

URDF 링크는 ROS 규약(X 전방, Y 좌, Z 상), USD Camera 는 −Z 시선/+Y 상 규약입니다.
연결 회전은 `(w,x,y,z) = (0.5, 0.5, −0.5, −0.5)` 이며 회전행렬의 det=1·정규직교를 확인했습니다.

---

## Phase 2 — ROS 2 인터페이스 부착 ✅ 통과

구현: `isaac/ros_io.py` (제어 신호), `isaac/ros_camera.py` (영상 스트림),
`isaac/sensors.py` (센서 실장), `isaac/g1_nav_sim.py` (메인)

### 구성

| 방향 | 토픽 | 방식 | 실측 |
|---|---|---|---|
| 구독 | `/cmd_vel` | rclpy (내장 cp311) | 922 msg 수신 확인 |
| 발행 | `/clock` | rclpy | 물리 스텝마다 |
| 발행 | `/odom` | rclpy | 정책 스텝(50Hz)마다 |
| 발행 | `/tf` | rclpy | odom→base_link→base_footprint, base_link→d435_link→optical |
| 발행 | `/camera/color/image_raw` | replicator writer | 640×480 rgb8 |
| 발행 | `/camera/depth/image_rect_raw` | replicator writer | 640×480 **32FC1** |
| 발행 | `/camera/{color,depth}/camera_info` | replicator writer | fx=fy=**462.14**, cx=320, cy=240 |
| 발행 | `/mid360/points` | replicator writer | PointCloud2, 10 Hz, ~41k pts/msg |

`camera_info` 의 fx 는 HFOV 69.4°/640px 이론값 `320/tan(34.7°) = 462.1` 과 일치합니다.

### 종단 검증 결과

```
전진: /cmd_vel vx=0.5  →  (0.009, 0.000) → (17.545, −4.307)   18 m 주행
정지: 명령 중단        →  속도 ≈ 0                            워치독(0.5s) 정상
선회: vx=0.4, wz=0.4   →  yaw −143° 회전하며 이동              원호 주행 정상
TF:   odom→base_footprint 의 z = 0                            지면 투영 정상
```

### 실측으로 드러난 것

**1. `base_link → d435_link` 는 정적 TF 가 아님**
카메라는 `torso_link` 에 붙어 있고 허리 3축(yaw/roll/pitch)이 **구동 관절**입니다.
정적 TF 로 두면 허리가 움직일 때 포인트클라우드가 어긋나므로 USD 월드 변환에서
매 스텝 계산해 발행합니다.

**2. ROS 2 브리지 확장을 명시적으로 켜야 함**
`enable_extension("isaacsim.ros2.bridge")` 없이는 replicator writer 가 레지스트리에
등록되지 않아 `No writer with name 'LdrColorSDROS2PublishImage'` 로 실패합니다.
(rclpy 만 쓸 때는 확장 없이도 동작해서 늦게 발견됨)

**3. `camera_info` 는 intrinsic 을 직접 넘겨야 함**
writer 에 `k/r/p/width/height` 를 주지 않으면 **width=height=0, k=[] 인 빈 메시지**가
정상적으로 발행됩니다. RTAB-Map 은 이걸 받으면 RGB-D 를 처리하지 못합니다.
`read_camera_info(rp_path)` → `(CameraInfo, Usd.Prim)` **튜플** 을 반환하니 언팩 주의.

**4. 직진 시 미세한 요 바이어스**
`vx=0.5` 직진 명령에 wz ≈ −0.014 rad/s 의 바이어스가 있어 장거리에서 완만하게 휩니다
(18 m 주행에 y −4.3 m). Nav2 의 피드백 루프가 흡수하지만, 오도메트리만으로
장거리 추측항법을 하면 누적됩니다.

**남은 작업**: 모든 ROS 노드에 `use_sim_time:=true` 적용은 Phase 3 launch 파일에서 설정.

---

## Phase 3 — 시뮬 맵 작성 (RTAB-Map mapping) ✅ 통과

씬: **Isaac Sim `full_warehouse.usd`** (S3 에셋). 구현: `g1_localization/launch/g1_mapping.launch.py`,
`scripts/drive_patrol.py`

### 결과

```
DB   ros2_ws/src/g1_localization/maps/sim_warehouse.db  (439 MB, rtabmap 0.23.7)
노드 260개        (실기 DB 235개와 비슷한 규모)
링크 187개        이웃 140 / 루프클로저 39 / 국소공간 8
데이터 260행      image=260  depth=260  scan=260   ← 라이다 포함
격자 348×472 @0.05m = 17.4 × 23.6 m
```

rtabmap 버전이 실기 DB(0.23.7)와 **동일**해 나중에 그대로 교체 가능합니다.

### ⚠ 결정적 함정 — 센서 스탬프와 TF 스탬프가 서로 다른 시계였음

처음에는 로봇이 6 m 를 걸어도 **RTAB-Map 노드가 1개에서 늘지 않았습니다.**
원인은 타임스탬프였습니다:

```
WARN getTransform() (odom -> base_footprint) Lookup would require extrapolation
     into the future. Requested time 146.06 but the latest data is at 130.41
```

- 카메라/라이다 replicator writer 는 **Isaac 내부 시뮬레이션 시간**(`IsaacReadSimulationTime`)으로 스탬프를 찍음
- 반면 브리지는 `/clock`·TF·odom 을 **`step × PHYSICS_DT` 로 자체 계산**
- 두 시계가 어긋나 격차가 15초 이상으로 계속 벌어짐 → RTAB-Map 이 모든 프레임의 TF 조회에 실패

**수정: 모든 스탬프를 `world.current_time`(Isaac 내부 시뮬레이션 시간)으로 통일.**

| | 수정 전 | 수정 후 |
|---|---|---|
| TF 외삽 경고 | 수백 건 | **0** |
| 작업 메모리(WM) | 1 (정지) | **71 → 260 노드** |
| 스탬프 격차 | 15초, 계속 증가 | 0.155초, 일정 |

에러 없이 조용히 실패하는 유형이라, 이후 센서를 추가할 때도 **스탬프 출처를 반드시
Isaac 시뮬레이션 시간으로 통일**해야 합니다.

### 순찰 주행은 정책 특성을 반영해야 함

`scripts/drive_patrol.py` 는 Phase 1 실측을 그대로 반영합니다:
전진 0.5(추종 오차 1.4%), 선회는 `vx=0.4 + wz=0.4` 원호(제자리 회전 금지).
직진 구간이 길면 창고 벽에 막혀 정지하므로 5초(≈2.5 m)로 잡았습니다.

---

## Phase 4 — 측위 (RTAB-Map localization) ✅ 통과

구현: `g1_localization/launch/g1_localization.launch.py`
(매핑과 동일 파라미터 + `Mem/IncrementalMemory=false`, `Mem/InitWMWithAllNodes=true`)

### 결과 — 1바퀴 주행 후 측위 오차

```
map →base_footprint  x=−1.676  y=+3.215  yaw=−80.2°
odom→base_footprint  x=−1.569  y=+3.267  yaw=−79.2°     ← 시뮬 실제값

오차 0.118 m, 1.04°   → 기준(0.15 m) 통과
```

실기 DB 로 교체할 때::

    ros2 launch g1_localization g1_localization.launch.py \
        database_path:=~/Downloads/rtabmap.db use_lidar:=false

---

## Phase 3 — 시뮬 맵 작성 (원래 계획)

기존 `rtabmap.db`의 파라미터 세트를 그대로 적용해 **시뮬 씬의 새 DB**를 만듭니다.

```
Reg/Strategy=0  Reg/Force3DoF=false  Kp/DetectorStrategy=8
Grid/3D=true  Grid/CellSize=0.05  Grid/Sensor=1  Grid/RangeMax=5.0
Optimizer/Strategy=2  Rtabmap/DetectionRate=1  Vis/MinInliers=20
Mem/IncrementalMemory=true
```

- 입력: `/camera/color/*` + `/camera/depth/*` + `odom`(Phase 2)
- 텔레옵으로 씬을 한 바퀴 돌며 루프 클로저 확보
- 산출물: `ros2_ws/src/g1_localization/maps/sim_scene.db` + `/map` 2D grid
- 비교용으로 Isaac의 **Occupancy Map Generator**(`isaacsim.asset.gen.omap`)로도 ground-truth
  격자맵을 뽑아둡니다 (Nav2 static layer 대안 + 정확도 평가 기준)

**검증**: 저장된 DB의 노드 수·루프클로저 수 확인, 투영 격자맵이 omap ground truth와 일치.

---

## Phase 4 — Localization (RTAB-Map localization 모드)

- 동일 DB를 `Mem/IncrementalMemory=false`, `Mem/InitWMWithAllNodes=true`로 로드
- rtabmap이 `map→odom` 보정 발행 → Nav2 static/global 코스트맵과 정합
- odom 소스 2단계 전략:
  1. **1단계**: 시뮬 ground-truth odom (Nav2 문제와 분리해 디버깅)
  2. **2단계**: `rgbd_odometry`(RTAB-Map 시각 오도메트리)로 교체 → 실기와 동일 구성
     - 보행 진동으로 VO가 끊길 수 있음 → `Odom/ResetCountdown`, `Vis/MinInliers` 완화,
       IMU를 `Odom/GuessMotion` 보조로 투입

**검증**: 로봇을 임의 위치에 재배치 → 수 초 내 재측위. `map→base_footprint` 오차 < 0.15 m.

---

## Phase 5 — Nav2 통합

`g1_navigation/params/g1_nav2_params.yaml` 주요 설정:

### 상태: 🔶 탐사(매핑 모드)는 안정, 측위 모드 자율주행은 미완

#### 자율 탐사 — `scripts/explore_frontier.py`, `demo.sh explore`

측위 모드에서 목표·경로가 계속 바뀌던 문제의 **근본 원인이 맵 커버리지**임이 확인됐고,
매핑 모드 탐사로 전환하니 해결됐습니다.

| 실행 | 도달 | 주행 실패 | 사전차단 | 비고 |
|---|---|---|---|---|
| 측위 모드 자율주행 | 1 | 다수 | — | map→odom 점프로 목표가 계속 바뀜 |
| 탐사 1차 | 6 | 0 | — | 시뮬 살아있던 구간 |
| 탐사 2차 | 12 | 18 | — | 벽 너머 프론티어를 실제 시도하다 20초씩 실패 |
| 탐사 3차 (후진 도입) | 23 | 16 | 1 | 후진은 매번 실패했으나 복구 시퀀스 효과로 도달률 상승 |
| **탐사 4차 (조정 후)** | **7** | **0** | **10** | 사전 검사로 도달 불가 목표를 미리 제외 |

**핵심은 주행 실패 0.** 도달 불가 프론티어를 `compute_path_to_pose` 로 미리 걸러내니
시도한 목표는 전부 성공했습니다.

#### 벽 고착 탈출 — BackUp 복구

제자리 회전과 후진을 **둘 다** 막아두면 벽에 코를 박은 로봇이 영영 못 빠져나옵니다.
후진은 정확도가 나쁘지만(−0.4→−0.284, −0.5→−0.389; 22~29% 오차) **탈출에는 충분**하므로
복구 행동으로만 되살렸습니다 (일반 주행은 `allow_reversing: false` 유지).

실패 원인 두 가지를 로그로 특정해 조정했습니다:

```
Exceeded time allowance          → 거리 0.8→0.5 m, 시간 12→20초
Pose Goes Off Grid → Collision   → simulate_ahead_time 2.0→1.0, 국소맵 8→10 m
```

`backup_speed` 는 0.5 를 넘겨도 무의미합니다 — 정책 학습 범위가 `vx∈[-0.5,1.0]` 이라
`set_command` 에서 클램프됩니다. Nav2 기본값 0.025 로는 보행 개시 임계값 아래라
**정책이 명령을 아예 무시**하므로 반드시 0.4 이상으로 올려야 합니다.

> ⚠ 조정 후 실행에서는 갇히는 상황 자체가 없어 **후진 탈출의 실제 동작은 미검증**입니다.
> 실주행에서 `Running backup` 뒤에 `backup failed` 가 없으면 성공입니다.

#### 사용자 관찰로 잡은 문제 4건 (모두 원인 특정 + 수정 완료)

실제로 화면을 보며 지적해 주신 증상들이 전부 실제 버그·설정 문제였습니다.

| 관찰 | 원인 | 조치 | 검증 |
|---|---|---|---|
| 공터에 장애물이 생김 | 코스트맵 `inflation_radius: 0.75` (robot_radius 0.30 과 합쳐 1.05 m 여유 요구) | 0.45 로 축소 | RViz 1번 레이어만 보면 정상 |
| 후반부 전역 경로 이탈 | 맵 재조립 시 `map→odom` 점프 (최대 9.4 m) | 매핑 모드 `OptimizeFromGraphEnd: true` | **점프 12회 → 0회** |
| 목표는 전방인데 계속 후진 | 탈출 노드의 블로킹 루프 | 상태 기계 전환 | 단위 검증 3항목 통과 |
| 벽 고착 시 못 빠져나옴 | 후진·제자리회전 둘 다 금지 | 탈출 전용 노드 | **실제 발동 2회, 이후 속도 0.6대 복귀** |

##### 탈출 노드에서 낸 버그 두 개 (기록용)

1. **블로킹 루프** — 타이머 콜백 안에서 `while now < end: publish; spin_once()` 를 돌렸습니다.
   단일 스레드 실행기에서는 콜백 내부 `spin_once` 가 재진입에 막혀 `/clock` 을 처리하지
   못하고, sim time 이 멈춘 채 루프가 끝나지 않아 **후진 명령만 무한 발행**됐습니다.
   → 상태 기계(idle → back → turn → idle)로 교체. 틱마다 한 단계씩만 진행.
2. **안전장치 무력화** — 1번을 막으려 넣은 감시가 `if self._phase_start:` 로 판정했는데
   **sim time 은 0 에서 시작**하므로 `0.0` 이 falsy 가 되어 발동하지 않았습니다.
   → `None` 센티널로 수정. 단위 검증에서 발견.

##### 고착 판정 기준을 세 번 고침

| 시도 | 기준 | 결과 |
|---|---|---|
| 1 | `/cmd_vel` 에 명령이 있는데 안 움직임 | 발동 0회 — RPP 는 충돌 감지 시 **명령 자체를 안 냄** |
| 2 | 순간 속도 < 0.06 | 발동 0회 — 보행 흔들림으로 순간 속도가 임계값을 넘나듦 (실측: 20초 정지인데 카운터가 0.2초에서 리셋) |
| 3 | **4초 창에서 이동 거리 < 0.25 m** | 흔들림에 영향받지 않음. 실제 검증 대기 |

#### 미확인 — 사전 검사가 탐사 범위를 줄일 가능성

조정 후 맵이 174 MB 로 직전(1.1 GB)보다 작습니다. 사전 검사가 지나치게 엄격해
갈 수 있는 곳까지 막았을 수 있습니다. 완화 수단: `--no-precheck`, `--empty-retries` 증가.

---

### (이전 상태) 측위 모드 자율주행: 1회 성공, 안정성 미해결

```
성공 사례 — 시작 (−9.45, −4.20) → 목표 (−12.05, −5.70), 거리 3.0 m
           최종 (−11.73, −5.81), 오차 0.34 m, 액션 상태 4 (SUCCEEDED)
```

계획 → RPP 제어 → `/cmd_vel` → 보행 정책 → 실제 주행까지 전 경로가 동작합니다.
다만 **연속 5회 목표 검증은 통과하지 못했습니다** (아래 미해결 이슈).

#### ⚠ 결정적 함정 — yaw 목표 체커와 제자리 회전 금지가 모순

두 번의 실패가 같은 패턴이었습니다:

| 시도 | 목표 거리 | 실제 주행 | 최종 오차 |
|---|---|---|---|
| 1차 | 3.0 m | 6.5 m | 3.52 m |
| 2차 | 3.0 m | 6.1 m | 3.78 m |

둘 다 **목표를 통과해 계속 전진**했습니다. 감속 파라미터를 고쳐도 동일 → 감속이 아니라
**"도달 판정 자체가 나지 않는" 문제**였습니다.

원인: `use_rotate_to_heading: false` (제자리 회전 불가) 인데 `yaw_goal_tolerance: 0.50`
(방위 정렬 요구). 로봇이 목표 방위로 정렬할 수단이 없으니 조건이 영원히 충족되지 않고,
`SimpleGoalChecker` 가 성공을 반환하지 않아 컨트롤러가 계속 경로를 따라갑니다.

**수정: `yaw_goal_tolerance: 3.15` (≈π, 방위 무시).**
이 정책으로 최종 방위를 지정하려면 제자리 회전이 가능한 별도 수단이 필요합니다.

#### ⚠ 과잉 적용했던 파라미터

`min_approach_linear_velocity: 0.40` 도 틀렸습니다. 데드밴드는 **보행 개시**에만 걸리고
이미 걷는 중에는 작은 명령도 추종되는데, 감속 자체를 막아버렸습니다 → `0.10` 으로 완화.
보행 개시 부스트는 `GaitCommandShim` 이 담당합니다.

#### 🔴 미해결 — RTAB-Map 측위가 자주 끊김

`map→odom` TF 가 사라져 Nav2 가 계획을 못 하는 상황이 반복됩니다:

1. **정지 시**: 로봇이 멈추면 새 측위가 안 잡혀 TF 발행이 멎습니다
   (`Localization was good, but waiting for another one to be more accurate`).
   → `scripts/bootstrap_localization.py` 로 조금 걷게 해 복구 (0.1~0.8초)
2. **맵 밖으로 나갔을 때**: 시각 매칭 실패 (`Not enough inliers 0/20`)
3. 연속 목표 검증에서는 40초를 걸어도 복구되지 않는 경우가 있었습니다

이것이 현재 가장 큰 걸림돌입니다. 후보 대응:
`RGBD/MaxOdomCacheSize=0` (즉시 발행, 정확도 손해) / 맵 커버리지 추가 확장 /
휠 오도메트리 신뢰도를 높여 측위 공백을 버티게 하기.

---

**Phase 1 실측이 컨트롤러 선택을 뒤집었습니다: MPPI(홀로노믹) → RPP(원호 주행).**

| 항목 | 값 | 근거 (Phase 1 실측) |
|---|---|---|
| `robot_base_frame` | `base_footprint` | 보행 진동 배제 |
| `robot_radius` | 0.30 | G1 어깨폭 + 여유 |
| controller | **RPP** (Regulated Pure Pursuit) | 제자리 회전 불가·측방 불가 → **원호 주행**이 정책 특성과 일치 |
| `desired_linear_vel` | 0.6 | 임계값(0.3 실패/0.5 성공) 위 + 추종 우수 구간 |
| `minimum_speed` | 0.4 | 이 아래로 떨어지면 보행이 멈춤 |
| `vx_max` / `vx_min` | 0.8 / **0.0** | 후진은 22~29% 오차 → 주행에 사용 안 함 |
| `vy_max` | **0.0** | 측방은 "0 아니면 0.5" 이진 동작 → 비홀로노믹 취급 |
| `wz_max` | 0.8 | 보행 중 선회는 10% 오차로 양호 |
| `use_rotate_to_heading` | **false** | 제자리 회전은 wz≥1.0 에서 38% 오차 → 금지 |
| `allow_reversing` | **false** | 후진 추종 부정확 |
| recovery: `spin` / `backup` | **비활성** | 둘 다 정책이 제대로 수행 못 함 |
| `xy_goal_tolerance` | 0.35 | 미세 명령 무시 특성상 정밀 정지 불가 |
| `ax_max` | 0.5 | 급가속 시 정책 추종 실패 |
| planner | NavFn 또는 Smac 2D | 초기엔 단순하게 |
| `velocity_smoother` | 사용 | 램프가 무해함을 실측 확인 |
| local costmap | voxel layer (depth 기반) | 2D 라이다가 없음 |
| AMCL | **비활성** | 측위는 RTAB-Map 담당 |

> 제자리 회전을 못 쓰므로 **좁은 공간에서의 방향 전환이 제약**됩니다. 실주행에서 문제가 되면
> 두 가지 선택지: (a) 480차원 최신 정책(후보 B)이 저속 회전을 더 잘하는지 확인,
> (b) 제자리 회전 전용 정책을 별도 학습. Phase 5 에서 판단합니다.

**검증**: RViz에서 2D Goal Pose로 10 m 주행 성공, 장애물 회피, 도착 오차 < 0.35 m.
연속 5회 goal 왕복 무낙상.

---

## ⚠ 루프 클로저 오검출 — 실기 DB 파라미터를 그대로 쓰면 안 됨 (2026-08-13)

`/odom` 이 PhysX 참값이므로 **루프 클로저의 오차 = 곧 오검출**입니다. 이 성질로
`explored.db` 의 링크를 전부 검산한 결과:

| | 수락된 클로저 | 오검출 | 병진 오차 | 그래프 거부 |
|---|---|---|---|---|
| 수정 전 (`Reg/Strategy=0`) | 15 | **13** | 3 ~ 18 m | **81 회** |
| 수정 후 (`Reg/Strategy=2`) | 9 | 0 | **0.024 ~ 0.075 m** | 0 회 |

원인은 실기 DB 에서 승계한 `Reg/Strategy=0`(순수 비주얼) + `Vis/MinInliers=20` 입니다.
반복 패턴 창고를 **바닥만 보는 카메라**(D435i 아래 47.6°)로 보면 시각 특징이 서로
구별되지 않습니다. 오검출이 한 번 그래프에 박히면 RTAB-Map 이 `RGBD/OptimizeMaxError`
초과를 감지해 **이후 모든 클로저를 거부**하고, 맵은 뒤틀린 채 복구되지 않습니다.
`RGBD/OptimizeFromGraphEnd=true` 라서 로봇 좌표는 고정된 채 **맵 쪽이 뒤틀리므로**
`map→odom` 점프(0.000 m)만 보면 정상으로 보입니다 — 이래서 오래 못 찾았습니다.

**수정** (매핑·측위 launch 양쪽 동일):

```
Reg/Strategy           0 → 2      비주얼 정합 후 라이다 ICP 로 검증
Reg/Force3DoF      false → true   평지 창고 + 보행 로봇
Vis/MinInliers        20 → 35
Icp/CorrespondenceRatio  → 0.3    ← 엉뚱한 장소는 여기서 탈락
Icp/MaxCorrespondenceDistance 0.1 → 0.3
Icp/MaxTranslation / MaxRotation → 1.0 m / 0.5 rad
```

**남은 것**: 수정 후 클로저 9개 중 4개가 병진은 3 cm 이내인데 **요각이 5.6~6.7°**
어긋납니다. 선회 속도 0.4 rad/s(23°/s) × rtabmap 이 보고하는 지연 0.1~0.3 s 와
자릿수가 맞아, 센서 스탬프와 odom 스탬프의 시차가 남아 있는 것으로 보입니다.
(Phase 3 에서 한 번 잡았던 문제와 같은 계열 — 다음 확인 대상.)

**기존 DB 는 전부 폐기 대상입니다.** `sim_warehouse.db`, `explored.db` 를 포함해
2026-08-13 이전에 만든 맵은 24° 조각 라이다로 만들어졌습니다. `demo.sh` 기본
(측위) 모드가 `sim_warehouse.db` 를 쓰므로 **재매핑 전에는 측위 모드 결과를
신뢰하면 안 됩니다.**

## ⚠ "맵이 커지면 깨진다" — 크기 제한이 아니라 평행 통로 오검출 (2026-08-19)

**크기 제한은 걸려 있지 않습니다.** DB 에 저장된 실제 파라미터 확인:

```
Rtabmap/MemoryThr    0   (WM 노드 수 무제한)
Rtabmap/TimeThr      0   (처리시간 제한 없음)
GridGlobal/MaxNodes  0   (격자 노드 무제한)
```

### 실제로 무슨 일이 일어나는가

최적화된 포즈를 참값(odom = PhysX)과 대조하면 **어느 순간 갑자기** 깨집니다:

| 노드 구간 | 평균 오차 | 최대 오차 |
|---|---|---|
| 1 ~ 323 | **0.000 m** | **0.000 m** |
| 324 ~ 445 | 0.11 m | 0.71 m |
| 450 ~ 523 | 2.90 m | 4.80 m |
| 524 ~ 600 | 7.11 m | 14.76 m |
| 755 ~ 813 | 21.63 m | **26.31 m** |

첫 루프 클로저 전까지는 오차가 **정확히 0** 입니다(odom 이 참값이므로 당연). 즉 맵을
망가뜨리는 것은 오직 루프 클로저입니다.

### 범인: 평행 통로

오검출 클로저가 이은 두 지점을 참값으로 찍으면:

```
660(-12.95, 21.99) <-> 409(  1.84, 22.09)   실제거리 14.79 m
762(-18.35, 18.45) <-> 480( -3.26, 18.18)   실제거리 15.10 m
766(-18.14, 15.90) <-> 502( -3.16, 16.09)   실제거리 14.98 m
```

**y 는 같고 x 만 15 m 차이** — full_warehouse 의 평행 통로 간격입니다. 두 번째 통로에
들어서는 순간 맵이 첫 통로 위로 접혀 얹힙니다. 맵이 커질수록 그런 통로를 더 만나므로
"크기 제한"처럼 보인 것입니다.

### 2026-08-13 수정(Reg/Strategy=2 + ICP 검증)으로는 못 막았습니다

수정 파라미터로 돌린 3460 노드 실행에서도 **수락 클로저 23 개 중 12 개가 오검출**,
전부 14.85 m 였습니다(라이다는 정상 — 방위각 312°, 고도 −7~+6.6° 확인).

- **ICP 가 못 거릅니다.** 평행 통로는 외형뿐 아니라 3D 형상까지 같아 ICP 가 깨끗하게 정합됩니다.
- **`RGBD/OptimizeMaxError` 도 못 거릅니다.** 오검출 12 개가 전부 같은 14.85 m 오프셋으로
  **서로 일관**되어 그래프에 모순이 생기지 않습니다 (거부 1 회에 그침).
  자기모순적인 오검출만 잡는 장치라 "통째로 접힌" 해석은 통과시킵니다.

### 해법: 후보 탐색을 근방으로 제한

검증 단계가 아니라 **후보 생성 단계**를 막아야 합니다.

```
Rtabmap/LoopThr  0.11 → 2.0     외형 기반 클로저 가설을 전부 기각
Kp/MaxFeatures   500  (유지)     어휘는 계속 만들어 DB 에 남김
```

가설값은 확률이라 1 을 넘을 수 없으므로 2.0 은 "절대 성립 안 함"입니다. 남는 클로저는
`RGBD/ProximityBySpace`(반경 `RGBD/LocalRadius`=10 m + ICP 검증)뿐이라 15 m 떨어진
통로는 후보에 오르지 못합니다. **매핑 launch 에만 적용**했습니다 — 측위 모드는 전역
재측위가 필요합니다.

#### ⚠ `Kp/MaxFeatures` 를 음수로 막으면 안 됩니다 (한 번 밟은 지뢰)

처음엔 `Kp/MaxFeatures=-1`(추출 안 함)로 막았습니다. 오검출은 사라졌지만 **맵이
측위에 못 쓰게 됐습니다.** RTAB-Map 은 시그니처의 특징을 **정합에도 재사용**하므로
`Reg/Strategy=2` 의 시각 정합까지 같이 죽습니다.

| 맵 | 노드 | 단어 | 루프 클로저 |
|---|---|---|---|
| `sim_warehouse.db` (Phase 4, 측위오차 0.118 m) | 233 | 87,547 | — |
| `guided.db` (`Kp/MaxFeatures=-1` 로 작성) | 1,418 | **0** | **0** |
| `guided_loc.db` (아래 방법으로 복구) | 1,418 | **502,756** | 7 (전부 근접, 오검출 0) |

**복구는 재주행 없이 됩니다.** `Mem/BinDataKept=true` 라 RGB·depth·scan·calibration 이
전부 남아 있어 저장된 이미지에서 어휘만 다시 뽑으면 됩니다:

```bash
rtabmap-reprocess --Kp/MaxFeatures 500 --Rtabmap/LoopThr 2.0 \
    --Reg/Strategy 2 --Reg/Force3DoF true --Vis/MinInliers 35 \
    --Icp/CorrespondenceRatio 0.3 --Icp/MaxCorrespondenceDistance 0.3 \
    guided.db guided_loc.db
```

실측: 노드 위치 차이 **최대 0.000 mm**(지오메트리 완전 보존), 격자 셀 동일,
외형 클로저 0 개 / 근접 클로저 7 개 전부 정상. 이 조합이 "어휘는 남기고 통로 접힘은
막는다"가 실제로 성립함을 보여줍니다.

#### 측위 쪽에 남은 위험: `RGBD/MaxOdomCacheSize: 0`

RTAB-Map 문서상 이 파라미터는 *"이전에 측위했던 곳과 매우 비슷한 장소로 순간이동하지
않도록 측위 변환을 검증"* 합니다 — **정확히 평행 통로 대비 안전장치**인데 부트스트랩
교착 때문에 0(비활성)으로 두었습니다. 매핑에서 겪은 15 m 오검출이 측위에서는 로봇 위치가
튀는 형태로 나타납니다. `bootstrap_localization.py` 가 로봇을 밀어주므로 되살릴 여지가
있으나 **아직 검증 안 했습니다.**

대안으로 `slam_toolbox`(설치돼 있음)도 같은 원리입니다: 루프 클로저 탐색을 현재 추정
위치 반경으로 제한하므로 15 m 오검출이 구조적으로 불가능합니다. 다만 2D 전용이라
3D 격자와 실기 RGB-D DB 이식 계획(Phase 6)을 포기해야 합니다.

### 검사 도구

```bash
python3 scripts/check_map_quality.py <db>            # 오검출 개수 + 장애물 셀 비율
python3 scripts/check_map_quality.py --poses <db>    # 맵 왜곡량까지 (수 분)
```

### 맵 작성은 `guided` 모드를 씁니다

```bash
bash scripts/demo.sh guided     # 매핑 모드 + Nav2, RViz 2D Goal Pose 로 직접 목표 지정
```

`nav` 모드는 **측위 모드**라 저장된 맵을 읽기만 하고 맵이 자라지 않습니다. 맵을 만들면서
목표를 찍으려면 매핑 모드 + Nav2 조합이 필요해 `guided` 를 추가했습니다.

자율 탐사(`explore`)와 비교한 장점은 **경로를 사람이 정한다**는 것뿐이지만, 프론티어
탐사의 실패 모드(도달 불가 목표 시도, 벽 고착, 블랙리스트 누적)를 피할 수 있어 맵을
한 번 제대로 뜨는 데는 이쪽이 확실합니다. 다만 **평행 통로 오검출을 막아주지는 않습니다**
— 그건 위의 `Kp/MaxFeatures=-1` 이 담당합니다.

## Phase 6 — 실기 이식 준비 (설계만, 구현은 후속)

시뮬에서 확정한 **토픽/프레임/파라미터를 그대로 유지**하고 소스만 교체합니다.

| 구성요소 | 시뮬 | 실기 G1 |
|---|---|---|
| RGB-D | Isaac 카메라 640×480 | RealSense D435i 1280×720 (기존 DB와 동일) |
| odom | GT → rgbd_odometry | rgbd_odometry (+ IMU), 필요 시 MID-360 LIO |
| `/cmd_vel` 수신 | Python 정책 러너 | `mjlab_PDW/deploy` C++ FSM (`Velocity` 상태) |
| 정책 | 동일 `policy.onnx` | 동일 (이미 배포 검증본) |
| map | `sim_scene.db` | 기존 `rtabmap.db` |

- 실기 `/cmd_vel` → deploy FSM 브리지 노드가 유일한 신규 작업
- **안전**: `/cmd_vel` 워치독(0.3 s 무수신 시 정지), E-stop, 속도 클램프 이중화

---

## 결정 사항 요약

1. **Isaac Lab 대신 Isaac Sim 직접 사용** — 단일 로봇 인터랙티브 구동에 env 매니저는 과함
2. **정책은 02_mjPDW `g1_velocity` (default)** — deploy v0을 명세 기준, 최신 run을 구동 기준
3. **측위는 RTAB-Map으로 통일** — 기존 매핑 파라미터 승계, 시뮬 DB만 새로 작성
4. **AMCL·pointcloud_to_laserscan 미사용** — RGB-D 3D 파이프라인이 기존 자산과 일치
5. ~~컨트롤러는 MPPI (홀로노믹)~~ → **RPP (원호 주행)** — Phase 1 실측으로 뒤집힘.
   측방 이동과 제자리 회전이 실용 범위에서 동작하지 않아 G1을 비홀로노믹으로 취급합니다.
6. **`base_footprint` 기준 내비게이션** — humanoid 특유의 pelvis 진동 차단

## 주요 리스크

| 리스크 | 확률 | 완화책 |
|---|---|---|
| 관절 순서/obs 레이아웃 오매핑 | 높음 | Phase 1에서 격리 검증, 이름 기반 매핑 로그 |
| MuJoCo 학습 정책의 PhysX 전이 실패 | 중 | 03_IsaacPDW Isaac baseline 정책으로 폴백 |
| py3.11/3.10 DDS 미통신 | 중 | Phase 0에서 최우선 검증, FastDDS 프로파일 |
| 보행 진동으로 VO 소실 | 중 | GT odom 우선 → 단계적 전환, IMU 보조 |
| RGB-D만으로 좁은 통로 회피 실패 | 중 | 카메라 FOV 확대 또는 MID-360 라이다 추가 |

## 다음 액션

1. ✅ `05_Arch` 스캐폴딩 + `scripts/{isaac_env,ros_env}.sh` + `dds_ping.py`
2. ✅ Isaac 측 내장 rclpy 동작 검증 (py3.11, DDS 루프백 OK)
3. ⏳ **(사용자) 일반 터미널에서** `bash ~/Project/05_Arch/scripts/install_ros2.sh`
4. ⏳ Phase 0 마무리 — 프로세스 간 DDS 통신 확인
5. ⏳ Phase 1 — `G1PolicyController` 구현 (ROS 없이 보행 검증)
