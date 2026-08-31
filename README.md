# 05_Arch — Unitree G1 Localization & Navigation

Isaac Sim 안의 Unitree G1(29DOF) 휴머노이드에 **RTAB-Map 측위**와 **Nav2 자율주행**을
붙인 스택입니다. 시뮬에서 완성한 뒤 그대로 실기로 옮기는 것을 목표로 합니다.

현재: 매핑·측위(오차 0.118 m)·자율주행까지 동작. 측위 안정성은 개선 중입니다.

---

## 준비물

| | 필요한 것 |
|---|---|
| OS | **Ubuntu 22.04** (ROS 2 Humble 전용, 24.04 불가) |
| GPU | **NVIDIA RTX 계열** — 라이다를 레이트레이싱으로 시뮬레이션하므로 필수 |
| 디스크 | 30 GB 이상 |

검증 환경: Ubuntu 22.04.5 / RTX 5080 / 드라이버 570.211

---

## 설치 — 5단계

### 1. 이 저장소 받기

```bash
git clone <이 저장소 URL> ~/05_Arch
cd ~/05_Arch
```

경로는 어디든 됩니다. 스크립트가 알아서 찾습니다.

### 2. Isaac Sim 5.1.0 설치

Python 3.11 가상환경에 pip로 설치합니다.

> 공식 안내: [설치 방법](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_python.html) ·
> [시스템 요구사항](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html)

```bash
python3.11 -m venv ~/isaacsim_venv
source ~/isaacsim_venv/bin/activate
pip install --upgrade pip
pip install 'isaacsim[all,extscache]==5.1.0' --extra-index-url https://pypi.nvidia.com
pip install onnx pyyaml          # 이 프로젝트가 추가로 쓰는 것
deactivate
```

가상환경을 `~/isaacsim_venv` 가 아닌 곳에 만들었다면 위치를 알려주세요:

```bash
echo 'export ISAAC_VENV=~/내가/만든/venv' >> ~/.bashrc
```

### 3. ROS 2 Humble + Nav2 + RTAB-Map 설치

준비된 스크립트를 쓰면 한 번에 끝납니다. (sudo 비밀번호를 물어봅니다)

```bash
bash scripts/install_ros2.sh
```

직접 하고 싶거나 문제가 생기면 각 공식 문서를 참고하세요.

> [ROS 2 Humble 설치](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) ·
> [Nav2](https://docs.nav2.org/) · [RTAB-Map ROS](https://github.com/introlab/rtabmap_ros)

### 4. G1 로봇 모델 받기

로봇 3D 모델은 **Unitree가 따로 배포**하므로 이 저장소에 없습니다.
Hugging Face에서 받으세요.

> [unitreerobotics/unitree_model](https://huggingface.co/datasets/unitreerobotics/unitree_model)

```bash
git clone https://huggingface.co/datasets/unitreerobotics/unitree_model ~/unitree_model
```

받은 뒤 위치를 알려줍니다:

```bash
echo 'export G1_USD=~/unitree_model/G1/29dof/usd/g1_29dof_rev_1_0/g1_29dof_rev_1_0.usd' >> ~/.bashrc
source ~/.bashrc
```

창고·사무실 같은 배경 환경은 Isaac Sim이 알아서 받아오므로 준비할 게 없습니다.

### 5. 워크스페이스 빌드

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws && colcon build --symlink-install && cd ..
```

> `--symlink-install` 을 꼭 붙이세요. 없으면 맵 파일(GB 단위)까지 통째로 복사됩니다.

---

## 잘 됐는지 확인

위에서 아래로 순서대로 해보세요. **막히면 그 단계에서 멈추고 해결**하는 게 빠릅니다.

| | 터미널 A (Isaac) | 터미널 B (ROS 2) | 통과 기준 |
|---|---|---|---|
| 1 | `python scripts/dds_ping.py pub` | `python3 scripts/dds_ping.py sub` | B에 메시지가 뜬다 |
| 2 | `python isaac/g1_walk_test.py` | — | 로봇이 안 넘어지고 걷는다 |
| 3 | `python isaac/check_lidar_profile.py` | — | `PASS` |
| 4 | `python isaac/g1_nav_sim.py --scene simple_room --camera --lidar` | `python3 scripts/check_lidar_topic.py` | `PASS` |

각 터미널은 먼저 환경을 불러와야 합니다.

```bash
source scripts/isaac_env.sh     # 터미널 A 용
source scripts/ros_env.sh       # 터미널 B 용
```

마지막으로 전체를 한 번에:

```bash
bash scripts/demo.sh manual
```

RViz가 뜨고 로봇이 움직이면 설치 완료입니다.

> Isaac Sim 첫 실행은 셰이더 컴파일 때문에 **10분 이상** 걸립니다. 멈춘 게 아닙니다.

---

## 실행

```bash
bash scripts/demo.sh guided   # 맵을 만들며 직접 몰기 (맵 작성은 이걸 권장)
bash scripts/demo.sh          # 저장된 맵으로 측위 + 순찰
bash scripts/demo.sh nav      # 저장된 맵으로 자율주행
bash scripts/demo.sh explore  # 자율 탐사로 맵 작성
bash scripts/demo.sh manual   # 직접 조종
```

배경 바꾸기: `SCENE=simple_room bash scripts/demo.sh guided`
(`full_warehouse` 기본, 그 외 `warehouse` / `simple_room` / `office`)

### 맵 만들기

맵 파일은 개당 수백 MB~GB라 저장소에 없습니다. 직접 만드세요.

```bash
bash scripts/demo.sh guided
```

RViz에서 **2D Goal Pose** 로 가고 싶은 곳을 찍으면 로봇이 이동하면서 맵을 넓힙니다.
다 만든 뒤 품질 확인:

```bash
source scripts/ros_env.sh
python3 scripts/check_map_quality.py ros2_ws/src/g1_localization/maps/<이름>.db
```

---

## 딱 하나만 기억할 것 — 터미널 두 개를 섞지 마세요

Isaac Sim은 Python 3.11, ROS 2는 Python 3.10을 씁니다. **한 터미널에서 둘 다
불러오면 깨집니다.** 두 프로그램은 네트워크(DDS)로만 대화합니다.

| | 터미널 A | 터미널 B |
|---|---|---|
| 불러올 것 | `source scripts/isaac_env.sh` | `source scripts/ros_env.sh` |
| 쓰는 것 | Isaac Sim | Nav2 / RTAB-Map / RViz |
| 금지 | ROS 2 setup.bash | Isaac 가상환경 |

잘못 섞으면 두 스크립트가 경고를 냅니다. 경고가 보이면 새 터미널에서 시작하세요.
**증상은 대개 "오류는 없는데 토픽이 안 보임"** 입니다.

---

## 더 읽을거리

| 문서 | 내용 |
|---|---|
| [`docs/plan.md`](docs/plan.md) | 단계별 설계 근거와 실측값 — 왜 이 파라미터인지 |
| [`docs/setup_notes.md`](docs/setup_notes.md) | 환경 설정 중 겪은 함정과 해결 |
| [`THIRD_PARTY.md`](THIRD_PARTY.md) | 미포함 외부 에셋과 출처 |

문제가 생기면 `docs/setup_notes.md` 를 먼저 보세요. 자주 겪는 것들이 정리돼 있습니다.

### 알아두면 좋은 것 둘

**로봇이 바퀴가 아닙니다.** 제자리 회전은 오차 38%라 Nav2에서 막아뒀고, 후진도
탈출할 때만 씁니다. 그래서 컨트롤러가 원호 주행(RPP)입니다. 자세한 실측값은
`docs/plan.md` 참조.

**창고 맵이 가끔 접힙니다.** full_warehouse의 통로들이 15 m 간격으로 똑같이 생겨서
RTAB-Map이 같은 곳으로 착각합니다. 현재 설정으로 억제해 뒀지만 완전히 해결되진
않았습니다.
