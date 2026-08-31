# 외부 구성요소

이 저장소에 **포함되지 않은** 것들입니다. 아래에서 직접 받으세요
(설치 절차는 [README](README.md) 참조).

| 항목 | 받는 곳 |
|---|---|
| Unitree G1 29DOF 모델 | [huggingface.co/datasets/unitreerobotics/unitree_model](https://huggingface.co/datasets/unitreerobotics/unitree_model) |
| Isaac Sim 5.1.0 | [NVIDIA 공식 문서](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_python.html) |
| ROS 2 Humble | [docs.ros.org](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) |
| Nav2 | [docs.nav2.org](https://docs.nav2.org/) |
| RTAB-Map | [github.com/introlab/rtabmap_ros](https://github.com/introlab/rtabmap_ros) |
| 배경 환경 USD (창고·사무실) | Isaac Sim이 실행 시 자동 다운로드 |

## 포함된 것 — 보행 정책 가중치

`isaac/policy/velocity_v0/` (`policy.onnx`, `policy.onnx.data`, `deploy.yaml`) 는
[unitreerobotics/unitree_rl_mjlab](https://github.com/unitreerobotics/unitree_rl_mjlab)
의 기본 velocity 정책입니다.

**해당 저장소에는 LICENSE 파일이 없습니다.** 명시적 라이선스가 없으면 기본적으로
재배포 권한이 없으므로, 이 저장소를 **공개로 전환하기 전에 upstream에 확인하거나
가중치를 제거**해야 합니다. 제거할 경우 `.gitignore` 에 `isaac/policy/` 를 추가하고
README에 다운로드 안내를 넣으면 됩니다.

정책은 교체 가능합니다 — `deploy.yaml`(관측·행동 스펙)과 ONNX 파일만 규격에 맞으면
다른 velocity 정책으로도 스택이 동작합니다.
