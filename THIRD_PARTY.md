# 서드파티 구성요소

이 저장소는 아래 항목들을 **포함하지 않습니다**. 각 배포처에서 직접 받아
README `0. 사전 준비` 의 경로에 두거나 환경변수로 지정하세요.

| 항목 | 출처 | 이 저장소에서의 위치 |
|---|---|---|
| Unitree G1 29DOF USD 모델 | Unitree 공식 배포 (`unitree_ros` / `unitree_model`) | `$G1_USD` 또는 `assets/unitree_model/...` |
| Isaac Sim 기본 환경 USD (`Simple_Warehouse` 등) | NVIDIA Isaac Sim 에셋 서버 | 런타임에 자동 다운로드 |
| Isaac Sim 5.1.0 | NVIDIA (pip) | 별도 venv |
| ROS 2 Humble / Nav2 / RTAB-Map | 각 upstream (apt) | 시스템 설치 |

## 보행 정책 가중치

`isaac/policy/velocity_v0/` (`policy.onnx`, `policy.onnx.data`, `deploy.yaml`) 는
upstream **`unitree_rl_mjlab`** 의 기본 velocity 정책에서 가져온 것입니다.

이 저장소가 비공개인 동안은 문제되지 않지만, **공개 전환 시 upstream 라이선스를
확인하고 그에 맞는 고지 또는 제거가 필요합니다.** 제거하는 경우 README 에
다운로드 안내를 넣고 `.gitignore` 에 `isaac/policy/` 를 추가하세요.

정책 자체는 `isaac/g1_policy.py`(ONNX 런타임)와 `deploy.yaml`(관측/행동 스펙)만
있으면 교체 가능합니다 — 다른 velocity 정책을 써도 스택은 동작합니다.
