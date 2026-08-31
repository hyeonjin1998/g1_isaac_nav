# 05_Arch — Unitree G1 Localization & Navigation

Isaac Sim 안의 Unitree G1(29DOF)에 RTAB-Map 측위와 Nav2 자율주행을 붙인 스택.
시뮬에서 완성한 뒤 토픽/프레임/파라미터를 그대로 실기로 옮기는 것을 목표로 합니다.

| Phase | 내용 | 상태 |
|---|---|---|
| 0 | ROS 2 Humble 설치, py3.11↔py3.10 DDS 상호운용 | ✅ |
| 1 | 보행 정책 이식 (MuJoCo→PhysX), 속도 특성 측정 | ✅ |
| 2 | ROS 2 I/O (`/cmd_vel`, `/odom`, TF, D435i, MID-360) | ✅ |
| 3 | RTAB-Map 매핑 (warehouse) | ✅ |
| 4 | RTAB-Map 측위 (오차 0.118 m) | ✅ |
| 5 | Nav2 자율주행 | 🔶 주행 성공, 측위 안정성 미해결 |

- 단계별 설계 근거와 실측값: [`docs/plan.md`](docs/plan.md)
- 환경 함정 모음: [`docs/setup_notes.md`](docs/setup_notes.md)
- 미포함 외부 에셋: [`THIRD_PARTY.md`](THIRD_PARTY.md)

---

# 설치

## 요구사항

| | 요구 | 개발 머신 (검증 환경) |
|---|---|---|
| OS | **Ubuntu 22.04 (jammy)** — ROS 2 Humble 고정 | Ubuntu 22.04.5 LTS |
| GPU | **NVIDIA RTX 계열 필수** | RTX 5080 (16 GB) |
| 드라이버 | 관련 Isaac Sim 5.1 요구사항 충족 | 570.211.01 |
| 디스크 | 30 GB+ (Isaac 에셋 캐시 + 맵 DB) | — |

RTX가 필수인 이유: MID-360 라이다를 **RTX Lidar**로 시뮬레이션하므로 레이트레이싱
코어가 없으면 포인트클라우드가 나오지 않습니다.

Ubuntu 24.04에서는 동작하지 않습니다 — ROS 2 Humble이 jammy 전용입니다.

---

## 0. 사전 준비 — G1 USD 모델

**이 저장소에 포함돼 있지 않습니다.** Unitree 배포판에서 받아야 합니다.

```bash
git clone https://github.com/unitreerobotics/unitree_ros.git
# 또는 Unitree가 배포하는 unitree_model 패키지
```

`g1_29dof_rev_1_0.usd` 를 확보한 뒤, 다음 중 아무 방법이나 쓰면 됩니다.

```bash
# 방법 A — 환경변수 (권장)
export G1_USD=/path/to/g1_29dof_rev_1_0.usd

# 방법 B — 저장소 안에 배치 (assets/ 는 .gitignore 됨)
mkdir -p assets/unitree_model/G1/29dof/usd/
cp -r .../g1_29dof_rev_1_0 assets/unitree_model/G1/29dof/usd/

# 방법 C — 실행할 때마다 지정
python isaac/g1_nav_sim.py --usd /path/to/g1_29dof_rev_1_0.usd
```

창고/사무실 환경 USD는 Isaac Sim이 에셋 서버에서 자동으로 받으므로 준비 불필요합니다.

---

## 1. 저장소 클론

```bash
git clone <이 저장소 URL> ~/05_Arch
cd ~/05_Arch
```

경로는 어디든 상관없습니다. 스크립트가 자기 위치에서 저장소 루트를 찾습니다.

---

## 2. Isaac Sim 5.1.0 (프로세스 A, Python 3.11)

```bash
python3.11 -m venv ~/isaacsim_venv
source ~/isaacsim_venv/bin/activate
pip install --upgrade pip
pip install 'isaacsim[all,extscache]==5.1.0' --extra-index-url https://pypi.nvidia.com

# 이 프로젝트가 추가로 쓰는 것 (ONNX 정책 로드)
pip install onnx pyyaml
deactivate
```

venv 경로가 `~/IsaacLab/env_isaaclab` 가 아니면 알려줘야 합니다:

```bash
export ISAAC_VENV=~/isaacsim_venv     # ~/.bashrc 에 넣어두면 편합니다
```

확인:

```bash
source scripts/isaac_env.sh
python -c "import isaacsim, rclpy; print(rclpy.__file__)"
# .../isaacsim/exts/isaacsim.ros2.bridge/humble/rclpy/rclpy/__init__.py 가 나와야 합니다
deactivate
```

> 첫 실행은 셰이더 컴파일로 **10분 이상** 걸립니다. 멈춘 게 아닙니다.

---

## 3. ROS 2 Humble + Nav2 + RTAB-Map (프로세스 B, Python 3.10)

```bash
bash scripts/install_ros2.sh      # sudo 비밀번호를 물어봅니다
```

`ros-humble-desktop`, `navigation2`, `rtabmap-ros`, `rmw-fastrtps-cpp`,
`colcon` 을 설치합니다. 이미 ROS 2가 있으면 누락분만 채우고 끝납니다.

---

## 4. 워크스페이스 빌드

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install
cd ..
```

> **`--symlink-install` 를 반드시 쓰세요.** 이게 없으면 colcon이 `maps/` 디렉터리를
> `install/` 로 **복사**합니다. 맵 DB가 쌓이면 GB 단위가 그대로 복제됩니다.

---

## 5. 설치 검증

아래를 순서대로 통과하면 세팅이 끝난 겁니다. 각 단계는 독립 실행 가능합니다.

### 5-1. 두 Python 사이의 DDS 통신

```bash
# 터미널 A
source scripts/isaac_env.sh
python scripts/dds_ping.py pub

# 터미널 B
source scripts/ros_env.sh
python3 scripts/dds_ping.py sub
```

B에서 메시지가 보이면 통과. **안 보이면 여기서 멈추고 해결하세요** — 이후 모든 게
이 통신 위에 있습니다. `RMW_IMPLEMENTATION` / `ROS_DOMAIN_ID` / `ROS_LOCALHOST_ONLY`
불일치가 대부분의 원인이고, 토픽은 조용히 안 보일 뿐 오류가 나지 않습니다.

### 5-2. 보행 정책 (ROS 없이)

```bash
source scripts/isaac_env.sh
python isaac/g1_walk_test.py
```

G1이 넘어지지 않고 걸으면 통과.

### 5-3. 라이다 프로파일

```bash
source scripts/isaac_env.sh
python isaac/check_lidar_profile.py
```

`PASS` — 방위각 360°, 고도 −7~+52°.

### 5-4. 센서 토픽

```bash
# 터미널 A
source scripts/isaac_env.sh
python isaac/g1_nav_sim.py --scene simple_room --camera --lidar

# 터미널 B
source scripts/ros_env.sh
python3 scripts/check_lidar_topic.py
```

`PASS` — 10 Hz, 메시지당 ≥1000점, 방위각 ≥300°.

### 5-5. 전체 스택

```bash
bash scripts/demo.sh manual
```

RViz가 뜨고 `/cmd_vel` 로 로봇이 움직이면 설치 완료입니다.

---

# 맵 만들기

맵 DB는 저장소에 없습니다(파일당 수백 MB~GB). 직접 만듭니다.

```bash
bash scripts/demo.sh guided
```

RViz에서 **2D Goal Pose** 로 가고 싶은 지점을 찍으면 로봇이 그리로 이동하면서
맵을 넓힙니다. 자율 탐사(`explore`)보다 이쪽이 결과가 안정적입니다.

만들어진 DB는 `ros2_ws/src/g1_localization/maps/` 에 저장됩니다. 다 만들었으면
품질을 확인하세요:

```bash
source scripts/ros_env.sh
python3 scripts/check_map_quality.py ros2_ws/src/g1_localization/maps/<이름>.db
```

`장소 오검출 0` 이고 `장애물 셀 ≥50%` 면 쓸 만한 맵입니다.

> **주의 — 창고 씬의 평행 통로.** full_warehouse의 통로들은 14.8~15.1 m 간격으로
> 시각·기하 양쪽 모두 동일합니다. RTAB-Map이 이걸 같은 장소로 오인하면 맵 전체가
> 접힙니다. 현재 매핑 설정은 `Rtabmap/LoopThr: 2.0` 으로 외형 기반 루프 클로저를
> 전부 기각하고 근접 클로저만 쓰도록 돼 있습니다. 자세한 내용은
> [`docs/plan.md`](docs/plan.md) 참조.

---

# 실행

```bash
bash scripts/demo.sh          # 저장된 맵으로 측위 + RViz + 순찰 주행
bash scripts/demo.sh guided   # 맵을 만들며 2D Goal Pose 로 직접 몰기  ← 맵 작성 권장
bash scripts/demo.sh mapping  # 맵을 새로 만들며 관찰 (순찰 주행)
bash scripts/demo.sh explore  # 자율 탐사 (프론티어 자동 선정)
bash scripts/demo.sh nav      # 저장된 맵으로 자율주행 (맵은 자라지 않음)
bash scripts/demo.sh manual   # 직접 /cmd_vel 로 조종
```

씬 변경:

```bash
SCENE=warehouse   bash scripts/demo.sh guided   # 선반 적은 창고
SCENE=simple_room bash scripts/demo.sh guided   # 작은 실내 (가장 빠름)
SCENE=office      bash scripts/demo.sh guided   # 사무실 (통로 많음)
```

## 수동 실행 (3 터미널)

```bash
# A: 시뮬
source scripts/isaac_env.sh
python isaac/g1_nav_sim.py --scene full_warehouse --camera --lidar

# B: 측위
source scripts/ros_env.sh
ros2 launch g1_localization g1_localization.launch.py \
    database_path:=$PWD/ros2_ws/src/g1_localization/maps/<이름>.db

# C: 내비게이션
source scripts/ros_env.sh
ros2 launch g1_navigation g1_navigation.launch.py
python3 scripts/bootstrap_localization.py   # map 프레임 확보 (필수)
```

---

# 두 개의 프로세스, 두 개의 Python

이 프로젝트의 핵심 제약입니다. **절대 섞지 마세요.**

| | 프로세스 A (Isaac Sim) | 프로세스 B (ROS 2) |
|---|---|---|
| 환경 | `source scripts/isaac_env.sh` | `source scripts/ros_env.sh` |
| Python | 3.11 (venv) | 3.10 (시스템) |
| rclpy | Isaac 번들 (cp311) | `/opt/ros/humble` (cp310) |
| `/opt/ros/humble/setup.bash` | **source 금지** | 필수 |
| venv activate | 필수 | **금지** |

두 프로세스는 DDS 로만 통신합니다. 아래 세 값이 양쪽에서 같아야 하고,
다르면 **오류 없이 토픽만 안 보입니다**.

```
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ROS_DOMAIN_ID=0
ROS_LOCALHOST_ONLY=1
```

두 env 스크립트 모두 반대쪽 환경이 감지되면 경고를 냅니다. 경고가 보이면
새 터미널에서 다시 시작하세요.

## 환경변수 정리

| 변수 | 기본값 | 언제 필요한가 |
|---|---|---|
| `ISAAC_VENV` | `~/IsaacLab/env_isaaclab` | Isaac Sim venv 경로가 다를 때 |
| `G1_USD` | 후보 경로 자동 탐색 | G1 USD 위치를 못 찾을 때 |
| `G1_ARCH_ROOT` | `ros_env.sh` 가 자동 설정 | 보통 불필요 |
| `SCENE` | `full_warehouse` | `demo.sh` 씬 변경 |

---

# 구성

```
isaac/                       프로세스 A (Python 3.11)
├── g1_nav_sim.py            메인 — 씬 + 정책 + ROS I/O
├── g1_policy.py             ONNX 정책 런타임 (시뮬 비의존)
├── g1_walk_test.py          Phase 1 격리 검증
├── check_lidar_profile.py   RTX 라이다 프로파일 검증
├── sensors.py               G1 공식 URDF 실장 위치 (D435i, MID-360, IMU)
├── ros_io.py                /cmd_vel 구독, /odom·TF 발행, 보행개시 shim
├── ros_camera.py            D435i RGB-D + PointCloud2
├── ros_lidar.py             MID-360 (Livox 근사 프로파일)
├── lidar_configs/           RTX 라이다 JSON 프로파일
└── policy/velocity_v0/      보행 정책 가중치 (THIRD_PARTY.md 참조)

ros2_ws/src/                 프로세스 B (Python 3.10)
├── g1_localization/         RTAB-Map 매핑/측위 launch + 맵 DB(미커밋)
├── g1_navigation/           Nav2 파라미터 + 커스텀 BT
└── g1_bringup/              RViz 설정

scripts/                     환경·실행·검증
├── isaac_env.sh / ros_env.sh    두 환경 (섞지 말 것)
├── install_ros2.sh              ROS 2 설치
├── demo.sh                      통합 실행
├── dds_ping.py                  py3.11↔3.10 통신 검증
├── check_lidar_topic.py         발행 클라우드 검증
├── check_map_quality.py         맵 DB 품질 검사
├── bootstrap_localization.py    map 프레임 확보
├── stuck_escape.py              고착 탈출 (후진)
└── explore_frontier.py          자율 탐사
```

---

# 보행 정책의 제약 (Nav2 설정 근거)

Phase 1 실측. 바퀴 로봇 기본값을 쓰면 로봇이 멈춰 있거나 넘어집니다.

| 축 | 무반응 | 최초 반응 | 비고 |
|---|---|---|---|
| `vx` 전진 | ~0.3 | **0.5** | 추종 오차 1.4% — 우수 |
| `vx` 후진 | −0.3 | −0.4 | 29% 오차 → 탈출 시에만 사용 |
| `vy` 측방 | ~0.45 | **0.5** | 학습 최대치 → 비홀로노믹 취급 |
| `wz` 제자리 회전 | ~0.8 | **1.0** | 38% 오차 → **금지** |
| `wz` 보행 중 | — | 0.4 | 10% 오차 — 우수 |

데드밴드는 축별이 아니라 **보행 개시**에 걸립니다. 일단 걷기 시작하면 작은 명령도
잘 추종합니다.

→ 컨트롤러는 **RPP(원호 주행)**, `use_rotate_to_heading: false`,
`yaw_goal_tolerance: 3.15`(방위 무시), spin 복구 비활성화.

---

# 알려진 미해결 이슈

**RTAB-Map 측위가 끊김** — 로봇이 정지하거나 맵 밖으로 나가면 `map→odom` 발행이
멎어 Nav2가 계획을 못 합니다. `scripts/bootstrap_localization.py` 로 대개
복구되지만 항상은 아닙니다.

관련해서 `g1_localization.launch.py` 의 `RGBD/MaxOdomCacheSize: 0` 는
순간이동 방지 검사를 끈 상태입니다. 부트스트랩 교착을 피하려는 선택이지만,
평행 통로에서 오검출이 나면 로봇이 15 m 튈 수 있습니다. 대응 후보는
[`docs/plan.md`](docs/plan.md) Phase 5 절 참조.

# 문제 해결

| 증상 | 원인 / 조치 |
|---|---|
| 토픽이 조용히 안 보임 | 두 셸의 `RMW_IMPLEMENTATION`/`ROS_DOMAIN_ID`/`ROS_LOCALHOST_ONLY` 불일치. `scripts/dds_ping.py` 로 격리 |
| `import rclpy` 실패 (venv) | 그 셸에서 `/opt/ros/humble/setup.bash` 를 source 했음. 새 터미널에서 시작 |
| ROS 노드가 conda python을 잡음 | `ros_env.sh` 가 PATH에서 conda를 밀어냅니다. 경고 메시지 확인 |
| G1 USD 를 못 찾음 | `export G1_USD=/path/to/g1_29dof_rev_1_0.usd` |
| 라이다 포인트 0개 | RTX GPU 아님, 또는 라이다 프로파일 미적용. `check_lidar_profile.py` |
| 맵이 커지면 접힘 | 평행 통로 오검출. 위 '맵 만들기' 주의 참조 |
| `colcon build` 가 매우 느림/큼 | `--symlink-install` 누락 → `maps/` 복사 중 |
