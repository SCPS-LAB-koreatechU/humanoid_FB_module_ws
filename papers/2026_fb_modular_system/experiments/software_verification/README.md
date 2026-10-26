# 모듈 관리자 소프트웨어 수준 검증

`bhl_module_manager`의 파이썬 코어로 장착, 분리 요청을 처리하고 그 결과를 수치와 그림으로 남깁니다.
포럼 논문 「복합 로봇 시스템을 위한 기능 블록을 이용한 모듈형 시스템」의 적용 사례(구성 A~E)와 보조 그림을 이 폴더에서 다시 계산할 수 있습니다.

**실제 로봇, 실제 부품 탈착, 실제 ROS 2 실행은 포함하지 않습니다.** 모두 소프트웨어 수준의 결과입니다.

## 실행

```bash
pip install numpy pyyaml matplotlib pillow fonttools playwright
python3 tools/build_fonts.py /usr/share/fonts/opentype/noto fonts    # 그림용 한글 글꼴 (Ubuntu: apt install fonts-noto-cjk)
python3 collect.py ../../bhl_module_manager $BHL_URDF data           # 수치 (약 10초)
python3 figs.py data renders out                                     # 그림 S1~S7
python3 overview.py . out/overview.pdf                               # 그림과 설명을 모은 A4 PDF (playwright의 Chromium 사용)
```

`BHL_URDF`는 [Berkeley-Humanoid-Lite-Assets](https://github.com/HybridRobotics/Berkeley-Humanoid-Lite-Assets)의
`data/robots/berkeley_humanoid/berkeley_humanoid_lite/urdf/berkeley_humanoid_lite.urdf` 경로입니다.
`fonts/`와 `out/`은 생성물이라 저장소에 올리지 않습니다.

## 시나리오

A(기본) → B(오른손을 그리퍼로 교체) → A → C(오른팔 분리) → A → D(양손 분리) → A → E(목과 카메라 분리)
→ 잘못된 장착 요청 3건 거부 후 A → E′(카메라만 분리). 각 단계에서 같은 명령 "빼빼로 건네줘"의 실행 계획을 다시 세웁니다.

## 파일

| 경로 | 내용 |
|---|---|
| `collect.py` | 시나리오 실행, 능력 목록, 거부 사례, 구성별 URDF와 TF 변화, 팔 기구학 비교, 처리 시간 측정 |
| `figs.py` | 그림 S1~S7. 그림마다 SVG(글자를 도형으로 변환), PDF(글꼴 포함), PNG(300 dpi), 수치 CSV |
| `overview.py` | 그림 전체와 설명을 A4 PDF 한 부로 묶음 |
| `tools/build_fonts.py` | Noto Sans CJK KR에서 그림용 글꼴(VizSansKR) 생성 |
| `renders/` | 그림 S1에 쓰는 3D 장면 (모듈 명세로 조립한 BHL 상반신 모델, 구성 A~E). 별도 뷰어로 만든 결과 이미지만 있음 |
| `data/timeline.json`, `timeline.csv` | 요청마다 TF 프레임 수, 구동 관절 수, 가능 능력 수, 실행 계획 |
| `data/capability_matrix.csv`, `capabilities.json` | 구성별 가능 능력과 불가 사유 |
| `data/urdf_changes.csv`, `urdf_*.urdf` | 구성별 생성 URDF와 구성 A 대비 바뀐 줄 수 |
| `data/kinematics_error.csv` | 팔마다 관절 범위 안 무작위 2,000자세에서 생성 기구학과 원본 URDF의 손끝 위치, 자세 오차 |
| `data/processing_time_ms.csv` | 요청 단계별 처리 시간 (단계마다 300회, 워밍업 20회 제외) |
| `data/rejections.json` | 거부된 장착 요청과 사유 (포트 타입, 호스트 CUDA, 포트 점유, 좌우) |
| `data/summary.json` | 위 결과의 요약과 실행 환경 |

## 수치를 쓸 때 주의할 점

- 처리 시간은 실행 환경마다 다릅니다. 저장된 값(요청 1회 중앙값 약 3.4 ms, ROS 통신 제외)은 클라우드 작업 환경에서 잰 값이므로,
  논문이나 발표에는 실제 제어 PC에서 다시 잰 값을 써야 합니다. 나머지 결과는 다시 실행해도 같습니다.
- 팔 기구학 오차(위치 최대 약 7 µm)는 모듈 명세에 DH 값을 반올림해 적은 데 따른 차이입니다. 반올림 전 유도 값으로는 최대 0.0004 mm였습니다.
- ZED 2i를 RDK X5에 배치하는 요청의 거부는 실제 실행 결과가 아니라 호스트 명세(CUDA 없음)에 대한 규칙 검사입니다.
