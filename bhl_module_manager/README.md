# bhl_module_manager

모듈 명세(YAML) 하나로 플러그인, 플러그아웃 때 **TF 트리(URDF), 관절 목록, 능력 목록**을 다시 구성하는 ROS 2 패키지입니다.
코어는 순수 Python(numpy, PyYAML)이라 ROS 없이도 돌아갑니다.

## 핵심 규칙

- **포트 좌표계의 z축 = 결합 방향.** 모듈이 장착되면 부모 포트 {P}와 모듈 기준 좌표계 {0}이 일치(항등 결합)하므로 구성 전체가 하나의 기구학 트리가 됩니다.
- **모듈 내부 기구학:** 관절이 있는 모듈은 표준 DH(θ = q + θ⁰), 나머지는 고정 변환. 팔 DH는 BHL URDF에서 유도했고 원본 URDF 순기구학과 일치합니다.
- **순기구학:** T_B^tool = T_B^P A₁(θ₁)⋯A_n(θ_n) T_F^C T_C^tool
- **능력 = 트리 연결성 + 호스트 요구 + 전제 능력.** 예: `hand_over(R_hand)`는 오른손 TCP가 장착되어 있고 `locate_hand(person)`(머리 카메라)이 가능할 때만 등록됩니다.
- **머리 = 목 모듈(요, 틸트 2자유도) + ZED 2i.** 몸통 포트 N(서보 홀더 윗면)의 z축이 요 축이자 결합 방향이라 목 모듈의 DH {0}이 N과 같습니다. 카메라는 목 모듈 브래킷 윗판의 `camera_1_4_20` 포트에 붙습니다.

## 구조

```
modules/        모듈 명세 (FB 기술서): torso_bhl, arm_bhl, connector_d84, hand_dexhand_v1, gripper_placeholder, neck_pan_tilt_scs, head_zed2i
hosts/          연산 노드 능력 (control_pc: cuda 있음, rdk_x5: cuda 없음)
configs/        구성 A~E (장착 목록)
bhl_module_manager/
  descriptor.py 명세 읽기와 검증
  assembly.py   장착, 분리(하위 트리 제거), 검증, 순기구학
  capability.py 능력 평가, "빼빼로 건네줘" 계획
  urdf_gen.py   URDF 생성(robot_state_publisher용), URDF 순기구학
  node.py       ROS 2 노드
launch/         robot_state_publisher + module_manager
scripts/demo_plug.py   ROS 없이 A→E 시나리오 실행
test/test_core.py      검증 테스트
examples/urdf/  구성 A~E에서 생성한 URDF 예시
```

## 새 모듈 추가 (코어 수정 없음)

`modules/`에 YAML 파일 하나를 넣습니다. 도메인을 나누는 기준은 하나입니다: **구조가 정하는 값은 기계, 구동기나 소프트웨어가 정하는 값은 제어.**

| 도메인 | 필드 | 쓰임 |
|---|---|---|
| 기계 | `mates_with` | 꽂힐 수 있는 포트 타입 (불일치면 장착 거부) |
| 기계 | `kind` (`base`, `arm`, `actuator`, `adapter`, `end_effector`, `sensor`, `tool`) | 기록, 시각화용 분류 |
| 기계 | `kinematics` (`dh_standard`의 a, α, d, θ⁰와 관절별 `range` = 기구 범위, 또는 없음), `frames`, `ports` | 트리 합성, 다음 모듈이 꽂힐 포트 |
| 기계 | `mass_kg` | 포트 하중 한도 검사 (값이 있을 때) |
| 전기 | `bus`, `ids`, `host_requirements` | 같은 호스트, 같은 버스 ID 충돌 검사, 호스트 기능(cuda, usb3, can) 검사 |
| 제어 | `rate_hz`, `effort`, `velocity`, `soft_limits`, `zero_offset` | 소프트 한계는 기구 범위 안에 있어야 함(밖이면 명세 거부), URDF `limit`에는 소프트 한계와 토크, 속도가 들어감, 영점은 드라이버용 |
| 기능 | `skills[].requires` (`frames`, `capabilities`, `host`) | 능력 목록 자동 등록 |

예전처럼 DH 행에 `limit`을 쓰거나 `kinematics`에 `effort`, `velocity`를 두면 명세 오류로 알려 줍니다(기계와 제어 값이 섞이지 않게). θ⁰는 DH 좌표계를 잡는 약속이라 기계, "서보 위치값 512 = q 0" 같은 영점 대응은 캘리브레이션이라 제어입니다.

예시 (흡착 그리퍼, test_core.py의 확장성 테스트와 같음):

```yaml
id: suction_cup
name: 흡착 그리퍼
kind: end_effector
sides: [R, L]
fb:
  mechanical:
    mates_with: hand_d84
    frames: {tcp: {xyz: [0, 0, 95]}}
  functional:
    skills:
      - {name: "pick({side}_suction)", requires: {frames: [tcp]}}
      - {name: "hand_over({side}_suction)", requires: {frames: [tcp], capabilities: ["locate_hand(person)"]}}
```

주의: YAML에서 `on:` 키는 불리언으로 읽히므로 DH 프레임 별칭은 `F: {at: dh5}`처럼 `at`을 씁니다.

## 실행

ROS 없이:

```bash
pip install numpy pyyaml pytest
python3 scripts/demo_plug.py --out out                 # A→E 시나리오, out/urdf/*.urdf, out/plug_log.json
BHL_URDF=/path/berkeley_humanoid_lite.urdf python3 -m pytest -q test   # 원본 URDF 비교 포함
```

ROS 2 (Humble 기준):

```bash
cd ~/ros2_ws/src && cp -r bhl_module_manager . && cd .. && colcon build --packages-select bhl_module_manager && source install/setup.bash
ros2 launch bhl_module_manager module_manager.launch.py \
    bhl_urdf:=/path/berkeley_humanoid_lite.urdf mesh_prefix:=package://<BHL 설명 패키지>/meshes
# 분리, 장착 요청
ros2 topic pub --once /modules/request std_msgs/msg/String "{data: '{\"op\": \"detach\", \"id\": \"r_hand\"}'}"
ros2 topic pub --once /modules/request std_msgs/msg/String \
  "{data: '{\"op\": \"attach\", \"port\": \"r_conn.hand\", \"module\": \"gripper_placeholder\", \"id\": \"r_grip\"}'}"
ros2 topic echo /modules/response
ros2 topic echo /capabilities          # 사용 가능 능력, 불가 사유, "빼빼로 건네줘" 계획
```

RViz에서는 RobotModel(Description Topic: /robot_description)과 TF를 켜면 장착, 분리에 따라 모델과 좌표계가 바뀝니다.

## 머리: 목 모듈 (요, 틸트)과 ZED 2i

실제 로봇 사진(2026-10-23) 기준 구성입니다. 앞쪽 상단 2020 바 뒷면 T슬롯에 3D 프린트 서보 홀더를 고정하고, 홀더 안에 요(팬) 서보를 세워 넣어 귀를 홀더 윗면에 나사로 고정합니다. 요 혼 위 ㄷ자 채널 브래킷에 틸트 서보를 눕혀 달고, 틸트 혼(오른쪽)과 아이들러(왼쪽)가 ㄷ자 알루미늄 브래킷을 받칩니다. ZED 2i는 그 윗판에 바닥 1/4"-20 나사로 체결합니다.

| i | 관절 (hw) | a | α | d | θ⁰ | 기구 범위 (기계) | 소프트 한계 (제어) | 비고 |
|---|---|---|---|---|---|---|---|---|
| 1 | `head_pan_joint` | 0 | −90° | 28 | 0 | −100~100° | −90~90° | z₀ = 위 (포트 N의 z), q > 0이면 왼쪽을 봄 |
| 2 | `head_tilt_joint` | 0 | 90° | 0 | 0 | −70~100° | −40~60° | z₁ = +y (요를 따라 회전), q > 0이면 아래를 봄 |

- 기구 범위: 서보 회전 범위 200°(SCS15 매뉴얼)의 가운데를 q = 0으로 가정, 틸트 뒤쪽은 모델에서 브래킷 윗판이 팬 브래킷 뒤 벽에 닿기 직전(약 −71°)까지. 소프트 한계는 임시 사용 범위입니다.

- T_B^Cam = T_B^N A₁(θ_p) A₂(θ_t) T_{2}^M T_M^Cam, T_{2}^M = Tz(36) (브래킷 윗판), T_M^Cam = Tz(15) (zed_camera_center)
- T_B^N = Trans(38, 0, 848) (몸통 상단 2020 바 윗면 830 mm 기준 홀더 윗면 +18 mm, 앞쪽 바 뒷면에서 12 mm 뒤)
- 요 축과 틸트 축이 만난다고 가정했습니다(a₁ = 0). 실제로 어긋나 있으면 a₁ 또는 d₂만 바꾸면 됩니다.
- 카메라만 풀면(`detach head`) 목 관절은 남고 `look_at(point)`, `locate_hand(person)`이 빠집니다. 목 모듈을 빼면(`detach neck`) 카메라까지 하위 트리째 빠집니다.
- `head_pan_joint`, `head_tilt_joint` 값은 SCS 서보 드라이버 노드가 /joint_states로 발행해야 합니다(이 패키지에는 없음). 팔 관절과 같은 방식입니다.

## 설계 메모

- **attach_mode = dynamic (기본):** 포트→모듈 결합 관절을 범위 0인 회전 관절로 두고 노드가 /joint_states로 0을 발행합니다. tf2는 정적 변환(/tf_static)을 지우지 못해서, 결합을 fixed로 두면 분리한 모듈의 프레임이 수신 측 버퍼에 계속 남습니다. dynamic이면 분리 즉시 발행이 멈추므로 현재 시각 기준 조회가 실패합니다. 그래도 분리 여부의 최종 판단은 /modules/state와 /capabilities를 기준으로 합니다.
- **ZED 2i:** 이 패키지는 목 모듈 브래킷 윗판(결합면)에서 `zed_camera_link`(바닥 나사 구멍)까지만 발행합니다. 그 아래(camera_center, 좌우 카메라, optical frame)는 zed_wrapper가 발행하므로 카메라 이름을 `zed`로 두면 트리가 이어집니다.
- **/joint_states 혼합:** 결합 관절은 하드웨어 관절 상태와 같은 토픽에 따로 발행됩니다. robot_state_publisher는 메시지마다 들어온 관절만 갱신하므로 문제없지만, joint_state_publisher_gui는 시작할 때의 모델만 알고 있으니 구성을 바꾸면 다시 켜야 합니다.
- **MoveIt:** 실행 중 모델 교체를 지원하지 않으므로 구성이 바뀌면 move_group을 다시 띄우는 방식이 필요합니다.

## 검증 상태

- DH 모델과 BHL 원본 URDF: 팔마다 무작위 500자세, 위치 0.01 mm, 자세 0.01° 이내 (DH 표 반올림 영향)
- 생성 URDF와 내부 모델: 무작위 100자세에서 모든 프레임 일치 (fixed, dynamic 두 모드)
- 생성 URDF의 BHL 메쉬 위치가 원본 URDF와 일치
- 포트 타입 불일치, 좌우 불일치, CUDA 없는 호스트에 ZED 장착 거부, 하위 트리 제거, 구성 A~E 계획 결과, 새 모듈 명세 추가
- 목 모듈: 무작위 200자세에서 DH 결과가 "N의 z축 회전(요) 다음 따라 도는 y축 회전(틸트)"과 일치, ZED를 몸통 포트에 바로 꽂으면 거부, 카메라만 분리하면 `look_at(point)`만 빠지고 목 관절은 유지
- 생성 URDF의 목 모듈 상자 형상이 Rhino 모델(사진 기준 추정 형상)과 같은 자세에서 일치
- 관절 한계 도메인 분리: 기구 범위(기계)와 소프트 한계(제어)를 따로 읽고, 소프트 한계가 기구 범위를 벗어나거나 예전 `limit` 키, 기계 쪽 `effort`를 쓰면 명세 오류. URDF `limit`에는 소프트 한계와 제어의 토크, 속도가 들어감
- ROS 2 노드: rclpy 스텁으로 요청 처리, robot_description 갱신 호출, 상태 발행 로직만 확인했고 실제 ROS 2 환경에서는 아직 실행하지 않았습니다.

## 비어 있는 값

DexHand, 커넥터, 그리퍼, 목 모듈 질량과 액추에이터 CAN ID, SCS 서보 ID는 비워 두었습니다(해당 검사는 값이 있을 때만 동작). 손 TCP는 메쉬 손바닥 중심 근사값이고, 그리퍼는 임시 형상입니다. 목 모듈의 서보 모델(SCS15로 추정), 홀더, 브래킷 치수, 기구 범위는 사진을 보고 잡은 추정값이라 실측 후 `modules/neck_pan_tilt_scs.yaml`의 DH 값과 `range`(기계), `soft_limits`, `zero_offset`(제어)을 고치면 됩니다.
