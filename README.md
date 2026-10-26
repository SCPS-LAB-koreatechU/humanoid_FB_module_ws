# humanoid_sdk

SCPS Lab 휴머노이드(Berkeley Humanoid Lite 상반신 기반)용 소프트웨어 모음입니다.

## 구성

| 경로 | 내용 |
|---|---|
| [`bhl_module_manager/`](bhl_module_manager/README.md) | 모듈 관리자 ROS 2 패키지. 부품(모듈) 명세 YAML을 바탕으로 장착, 분리 요청을 검증하고 URDF(TF 트리), 관절 목록, 능력 목록을 다시 구성합니다. 코어는 순수 Python이라 ROS 없이도 실행됩니다 |
| [`experiments/software_verification/`](experiments/software_verification/README.md) | 모듈 관리자의 소프트웨어 수준 검증. 구성 A~E 시나리오, 능력 목록, URDF 변경량, 팔 기구학 비교, 처리 시간을 계산하고 그림을 만듭니다 |

## 현재 상태

- 파이썬 코어: `test/test_core.py` 전체 통과 (BHL 원본 URDF 비교 포함)
- ROS 2 노드: rclpy 스텁으로 요청 처리 로직만 확인했고, 실제 ROS 2 환경과 실제 로봇에서는 아직 실행하지 않았습니다
- `experiments/`의 결과는 모두 소프트웨어 수준입니다 (실제 부품 탈착, 실제 ROS 2 통신 미포함)
- 보드(RDK X5) 제어 코드는 이 저장소가 아니라 연구실 `berkeley-humanoid-lite` 저장소의 `feature/onboard-x5` 브랜치에 있습니다

## 빠른 시작

```bash
pip install numpy pyyaml pytest
git clone https://github.com/HybridRobotics/Berkeley-Humanoid-Lite-Assets      # BHL 원본 URDF와 메쉬
export BHL_URDF=$PWD/Berkeley-Humanoid-Lite-Assets/data/robots/berkeley_humanoid/berkeley_humanoid_lite/urdf/berkeley_humanoid_lite.urdf
cd bhl_module_manager
python3 -m pytest -q test                  # 코어 검증
python3 scripts/demo_plug.py --out out     # 구성 A→E 시나리오 (ROS 불필요)
```

ROS 2 실행과 모듈 명세 작성법은 [bhl_module_manager/README.md](bhl_module_manager/README.md),
검증 데이터와 그림을 다시 만드는 방법은 [experiments/software_verification/README.md](experiments/software_verification/README.md)에 있습니다.
