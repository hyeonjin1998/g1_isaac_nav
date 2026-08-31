# 환경 설정 기록 및 트러블슈팅

## 검증 완료 (2026-08-12)

### ✅ Isaac Sim 내장 rclpy가 venv(Python 3.11)에서 동작

이 프로젝트 아키텍처의 핵심 전제였고, 실제로 확인했습니다.

```
$ source scripts/isaac_env.sh
$ python -c "import rclpy; print(rclpy.__file__)"
.../isaacsim/exts/isaacsim.ros2.bridge/humble/rclpy/rclpy/__init__.py
```

- Python **3.11.14** 에서 `import rclpy` 성공
- `std_msgs` / `nav_msgs` / `sensor_msgs` / `geometry_msgs` / `tf2_msgs` / `rosgraph_msgs` 임포트 성공
- 노드 + 퍼블리셔 생성 성공 (`rmw_fastrtps_cpp`)
- **DDS 루프백 pub/sub 왕복 성공** → rmw 계층까지 정상 동작

의미: 시스템 ROS 2(py3.10)와 Isaac Sim(py3.11)을 별도 프로세스로 띄우고 DDS로 통신시키는
설계가 성립합니다. Isaac 쪽에서 `/cmd_vel` 구독과 `/odom` 발행을 Python으로 직접 처리할 수 있습니다.

### ⏳ 남은 검증: cp311 ↔ cp310 프로세스 간 통신

ROS 2 설치 후 `scripts/dds_ping.py` 로 확인합니다. DDS 는 와이어 프로토콜 레벨에서
호환되므로 Python 버전 차이는 무관해야 하지만, 실측 전까지는 가정으로 둡니다.

---

## ROS 2 설치

`scripts/install_ros2.sh` 를 **일반 터미널에서** 실행하세요 (sudo 비밀번호 입력 필요).

### 확인된 사실

- 이 머신에 ROS 2 apt 저장소가 **등록돼 있지 않았습니다**
  (`apt-cache policy ros-humble-desktop` 이 빈 출력).
  저장소 등록 없이 `apt install ros-humble-*` 를 실행하면 "package not found" 로 실패합니다.
- 등록 방식은 **`ros2-apt-source` .deb** 입니다.
  구버전 문서의 `ros-archive-keyring.gpg` + `sources.list.d/ros2.list` 방식은 키 만료로 폐기됨.
- 해석된 값 (2026-08-12 기준):
  `ROS_APT_SOURCE_VERSION=1.2.0`, `codename=jammy`
  → `ros2-apt-source_1.2.0.jammy_all.deb` (URL HTTP 200 확인)

---

## 셸 규칙 — 두 환경을 절대 섞지 말 것

| | 프로세스 A (Isaac Sim) | 프로세스 B (ROS 2) |
|---|---|---|
| 환경 | `source scripts/isaac_env.sh` | `source scripts/ros_env.sh` |
| Python | 3.11 (venv) | 3.10 (시스템) |
| rclpy | Isaac 번들 (cp311) | `/opt/ros/humble` (cp310) |
| `/opt/ros/humble/setup.bash` | **source 금지** | source 필수 |
| venv activate | 필수 | **금지** |

두 스크립트 모두 반대쪽 환경이 감지되면 경고를 출력합니다. 경고가 보이면 새 터미널에서 시작하세요.

공통으로 일치해야 하는 값 (불일치 시 토픽이 조용히 안 보입니다):

```
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ROS_DOMAIN_ID=0
ROS_LOCALHOST_ONLY=1
```

---

## Isaac Sim 관련 함정 (실측)

### `SimulationApp.close()` 가 종료 코드와 stdout 을 삼킨다

`close()` 내부에서 `os._exit()` 가 호출돼 **파이썬 stdout 버퍼가 flush 되지 않고, 종료 코드도
항상 0** 이 됩니다. 첫 Phase 1 실행에서 스크립트 출력이 한 줄도 없는데 exit 0 이 나왔습니다.

대응 (`isaac/g1_walk_test.py` 에 적용):
```python
sys.stdout.reconfigure(line_buffering=True)
...
except Exception:
    traceback.print_exc()          # close() 전에 직접 출력
    Path(out).write_text(...)      # 결과를 파일로 이중 기록
finally:
    sys.stdout.flush(); simulation_app.close()
```
**Isaac 스크립트의 성패는 exit code 가 아니라 결과 JSON 으로 판정할 것.**

### `rendering_dt ≠ physics_dt` 면 제어 주기가 조용히 틀어진다 ⚠ 가장 위험

`World(physics_dt=1/200, rendering_dt=1/50)` 로 두고 `world.step(render=True)` 를 호출하면
내부적으로 `app.update()` 가 불려 **물리가 `rendering_dt/physics_dt = 4` 스텝씩 진행**됩니다.
루프는 이를 1스텝으로 세므로 **정책 제어 주기가 4배 느려집니다 (50Hz → 12.5Hz).**

증상: 로봇이 조금 비틀거리다가 발산해 넘어짐. 에러 메시지는 전혀 없음.

```
render 매 스텝(GUI)   → t=4.57s 낙상, 0.97m 만에 발산
  steps=228 → 루프 가정 시간 1.14s
  Isaac 실제 물리 시간         4.57s   ← 정확히 4.0배
```

헤드리스에서 20스텝마다 렌더할 때는 오차가 15% 라 증상이 약했고
(앞서 본 타임스탬프 12% 불일치도 같은 원인), **GUI 에서만 4배가 그대로 드러났습니다.**

**수정: `rendering_dt = physics_dt` 로 통일.** 그러면 `step(render=True)` 든 `False` 든
물리가 정확히 1스텝씩 진행됩니다. 렌더 주기는 `step % render_decim == 0` 으로 따로 조절합니다.

```
수정 후: [diag] 제어 주기 정상 (ratio=1.001, 실효 50.0Hz)
```

조용히 실패하는 유형이라 `g1_nav_sim.py` 에 **상시 감시**를 넣었습니다 —
step 2000 에서 `world.current_time / (step × physics_dt)` 가 1.0 에서 2% 이상 벗어나면 경고합니다.

### `pkill -f <패턴>` 이 자기 자신을 죽인다

`pkill -f` / `pgrep -f` 는 **전체 명령줄**을 매칭하므로, 그 명령을 감싼 셸의 명령줄에
패턴 문자열이 들어 있으면 **자기 자신을 죽이거나 자기 자신을 발견**합니다.

```bash
# 나쁨: 이 줄 자체가 "rtabmap" 을 포함 → 셸이 자살하고 launch 는 실행조차 안 됨
pkill -9 -f rtabmap; ros2 launch g1_localization g1_localization.launch.py

# 나쁨: 존재하지 않는 프로세스를 계속 "발견"함 (매번 PID 가 바뀜)
pgrep -f rtabmap
```

증상이 **빈 출력 + 종료코드 1/144** 라 원인을 엉뚱한 곳에서 찾게 됩니다.

```bash
# 좋음: 브래킷 트릭으로 자기 매칭 제외
ps -eo pid,args | grep '[r]tabmap_slam'
# 좋음: 정리와 실행을 별도 호출로 분리
```

### RTAB-Map 맵 재조립이 `map→odom` 을 튀게 한다 (매핑 모드)

증상: **후반부로 갈수록 전역 경로를 못 따르고, 앞이 비었는데 로봇이 멈춤.**

`scripts/diag_drift.py` 로 측정한 결과:

```
map→odom 점프 12회, 최대 9.392 m  (9.392 / 2.644 / 1.582 / 0.648 / 0.376 …)
전방점유 0%        ← 앞은 실제로 비어 있었음 (유령 장애물이 아님)
Maps update 급증   0.003s → 0.17~0.23s, 19회
```

점프 시각과 `Maps update` 급증 시각이 일치합니다. 맵이 커질 때 **전역 재조립**이 일어나고,
그때 로봇의 map 좌표가 통째로 이동해 Nav2 의 전역 경로가 무효가 됩니다.

**중간에 두 번 틀린 진단을 했고 데이터가 반증했습니다:**

| 가설 | 반증 |
|---|---|
| 루프 클로저가 원인 → 조이자 | 클로저 **수락 0건 / 거부 37건** — 이미 전부 거부 중 |
| 유령 장애물이 원인 | **전방점유 0%** — 앞은 비어 있었음 |

**수정: 매핑 모드에서 `RGBD/OptimizeFromGraphEnd: true`.**
그래프 최적화의 고정점을 첫 노드(맵 원점) 대신 **마지막 노드(로봇 현재 위치)** 로 잡습니다.
로봇 주변 좌표가 고정되고 맵 내용이 대신 이동하므로 경로 추종이 깨지지 않습니다.

측위 모드(`g1_localization.launch.py`)는 저장된 맵의 좌표계가 고정되어야 하므로 **false 유지**.

### 시뮬 라이다는 로봇 자신을 보지 않는다 (실기와 다름)

"공터에 장애물이 생긴다" 의 원인을 찾으며 확인한 것들:

```
MID-360 self-hit   : 78,752점 중 1 m 이내 0개
D435i 지면 오검출  : 307,200점 전부 z < 0.12 m 로 필터링, 장애물 마킹 0
맵의 고립 장애물   : 전체 점유의 0.5% (최대 0.01 m²)
```

실제 원인은 **코스트맵 `inflation_radius: 0.75`** 였습니다 (robot_radius 0.30 과 합쳐
1.05 m 여유를 요구 → 통로가 통째로 고비용). 0.45 로 낮춰 해결했습니다.

⚠ **실기에서는 다릅니다.** MID-360 은 360° 스캔이라 실기에서는 머리 좌우 구조물을
반드시 봅니다. Isaac RTX 라이다가 자기 메시를 레이캐스트에서 제외하기 때문에 시뮬에서만
안 보이는 것입니다. 실기 이식 시 `obstacle_min_range` 상향 또는 방위각 마스킹이 필요합니다.

### PhysX articulation DOF 순서 ≠ USD 선언 순서

USD 파일의 관절 선언 순서는 Unitree SDK 순서(좌각→우각→허리→좌팔→우팔)와 같지만,
런타임 `robot.dof_names` 는 **운동학 트리 BFS 순서**입니다:

```
0 left_hip_pitch  1 right_hip_pitch  2 waist_yaw    ← pelvis 직계 자식
3 left_hip_roll   4 right_hip_roll   5 waist_roll   ← 그 다음 레벨
```

29개 중 **27개가 어긋납니다.** 인덱스로 매핑하면 정책이 엉뚱한 관절을 구동하고, 증상은
그냥 "넘어짐"이라 원인 추적이 어렵습니다. 항상 `build_joint_index_map()` 으로 이름 기반 매핑.

### USD python 바인딩(pxr)을 Isaac Sim 없이 쓰기

`SimulationApp` 없이 USD 를 열어보려면 번들 USD 라이브러리 경로를 직접 지정해야 합니다:
```bash
L=<site-packages>/isaacsim/extscache/omni.usd.libs-*/
PY=/home/hyeonjin/.local/share/uv/python/cpython-3.11.14-linux-x86_64-gnu/lib
PYTHONPATH=$L LD_LIBRARY_PATH=$PY:$L/bin python -c "from pxr import Usd"
```
`libpython3.11.so.1.0` 를 못 찾는다는 오류가 나면 `$PY` 누락입니다.

---

## 알려진 함정

**`/opt/ros/humble` 을 source 한 셸에서 Isaac Sim 실행**
→ py3.10 rclpy 가 `PYTHONPATH` 에 얹혀 venv 의 3.11 인터프리터에서 `_rclpy_pybind11` 로딩 실패.
증상: `ImportError: undefined symbol` 또는 `ModuleNotFoundError`. 새 터미널에서 시작할 것.

**`ROS_LOCALHOST_ONLY=1` 인데 토픽이 안 보임**
→ 양쪽 터미널 모두에 설정됐는지 확인. 한쪽만 설정하면 discovery 가 실패합니다.

**Isaac Sim 이 시스템 ROS 2 라이브러리를 먼저 잡는 경우**
→ `LD_LIBRARY_PATH` 앞쪽에 `${ISAAC_ROS_BRIDGE}/lib` 이 오도록 `isaac_env.sh` 가 설정합니다.
`ldd` 로 실제 로딩 경로를 확인할 수 있습니다.
