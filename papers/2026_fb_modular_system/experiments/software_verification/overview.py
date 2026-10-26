"""One PDF with every figure and its caption (A4), for reading and sharing. usage: overview.py <viz dir> <out pdf>"""
import html, json, os, sys
from playwright.sync_api import sync_playwright

V, OUT_PDF = sys.argv[1], sys.argv[2]
S = json.load(open(os.path.join(V, "data", "summary.json")))
k = S["kinematics"]; t = S["processing_time"]["요청 1회 전체"]; u = {r[0]: r for r in S["urdf_changes"]}

FIGS = [
    ("S1_구성별_처리결과", "그림 S1. 부품 구성 A~E에서 같은 명령 “빼빼로 건네줘”의 처리 결과",
     "구성에 맞는 기능 블록으로 연결되거나, 실행할 수 없으면 사유와 함께 거부된다. 3D 장면은 모듈 명세로 조립한 BHL 상반신 모델이며, "
     "반투명으로 표시한 것이 분리된 모듈이다. 논문의 표 1과 같은 결과다."),
    ("S2_능력목록_행렬", "그림 S2. 구성별 능력 목록과 실행 경로",
     "부품이 빠지면 그 부품이 제공하던 능력과, 그 능력을 전제로 하는 능력이 함께 빠진다. 예를 들어 목과 카메라가 빠진 E에서는 손이 남아 있어도 "
     "받는 사람 손 위치를 알 수 없으므로 hand_over가 불가해진다. 명령은 남은 능력 가운데 정해진 우선순위로 실행 경로가 정해진다."),
    ("S3_재구성_타임라인", "그림 S3. 장착, 분리 요청에 따른 자동 재구성",
     "요청마다 TF 프레임, 구동 관절, 가능 능력 수가 다시 계산된다. 잘못된 장착 요청 3건은 상태를 바꾸지 않고 거부되었다."),
    ("S4_팔기구학_원본URDF_비교", "그림 S4. 생성된 팔 기구학과 BHL 원본 URDF의 차이",
     f"팔마다 관절 범위 안 무작위 2,000자세에서 손끝 좌표계를 비교했다. 위치 오차는 최대 {k['R']['max_um']:.1f} µm(오른팔), "
     f"{k['L']['max_um']:.1f} µm(왼팔), 자세 오차는 최대 {max(k['R']['max_rot_deg'], k['L']['max_rot_deg']):.4f}°이며, 명세에 DH 값을 "
     "반올림해 적은 데 따른 차이다. 반올림 전 유도 값으로는 최대 0.0004 mm였다."),
    ("S5_요청_처리시간", "그림 S5. 장착, 분리 요청 1회의 처리 시간",
     f"파이썬 코어만 측정했다(ROS 통신 제외). 요청 1회 중앙값은 {t['median_ms']:.1f} ms이고 대부분이 URDF 생성 시간이다. "
     "클라우드 작업 환경에서 잰 값이라, 논문에 쓰려면 실제 제어 PC에서 다시 재야 한다."),
    ("S6_URDF_변경량", "그림 S6. 구성 변경 시 URDF에서 바뀌는 양",
     "자동 생성된 URDF끼리 비교한 값이다. 정적 URDF를 쓰는 방식이라면 이만큼을 사람이 직접 고쳐야 하지만, 제안 방식에서는 장착, 분리 요청 1회로 "
     "반영되며 사용자 코드와 명세 파일은 수정하지 않는다."),
    ("S7_장착검증_거부사례", "그림 S7. 장착 요청 검증에서 거부된 사례",
     "기계 도메인(포트 타입, 점유, 좌우)과 전기 도메인(호스트 요구) 명세로 장착 전에 걸러진다. ZED 2i의 RDK X5 배치 거부는 실제 실행이 아니라 "
     "호스트 명세(CUDA 없음)에 대한 규칙 검사다."),
]

rows = [
    ("같은 명령의 처리 결과 (A~E)", "오른손 FB, 그리퍼 FB, 왼손 FB, 거부(파지 수단 없음), 거부(카메라 없음)"),
    ("팔 기구학과 원본 URDF 차이", f"위치 최대 {k['R']['max_um']:.1f} µm, 자세 최대 {max(k['R']['max_rot_deg'], k['L']['max_rot_deg']):.4f}° (팔마다 2,000자세)"),
    ("요청 1회 처리 시간", f"중앙값 {t['median_ms']:.1f} ms, 95% {t['p95_ms']:.1f} ms (파이썬 코어, ROS 통신 제외)"),
    ("구성 변경 시 URDF 변경량", ", ".join(f"{c} {int(r[1]) + int(r[2])}줄" for c, r in u.items())),
    ("장착 검증 거부", "4건 (포트 타입, 호스트 CUDA, 포트 점유와 타입, 좌우)"),
]

fonts = "".join(f'@font-face{{font-family:"{fam}";src:url("../fonts/{f}.ttf") format("truetype")}}'
                for fam, f in (("VR", "VizSansKR"), ("VM", "VizSansKRMedium"), ("VB", "VizSansKRBold")))
css = fonts + """
@page { size: A4; margin: 16mm 16mm 16mm 16mm }
* { box-sizing: border-box }
body { margin: 0; font: 9.5pt/1.6 VR; color: #0b0b0b; -webkit-print-color-adjust: exact }
h1 { font: 16pt/1.3 VB; margin: 0 0 2mm }
.sub { color: #52514e; font-size: 9pt; margin: 0 0 5mm }
.note { border-left: 2.5px solid #ec835a; background: #fdf3ee; padding: 3mm 4mm; margin: 0 0 5mm; font-size: 9pt }
.note b { font-family: VB; font-weight: normal }
table { border-collapse: collapse; width: 100%; font-size: 9pt; margin: 0 0 4mm }
th, td { text-align: left; padding: 1.6mm 2mm; border-bottom: 0.5px solid #e1e0d9; vertical-align: top }
th { font-family: VM; font-weight: normal; color: #52514e; width: 34% }
.fig { break-inside: avoid; margin: 0 0 7mm }
.fig img { width: 100%; display: block; margin: 0 0 2mm }
.cap b { font-family: VB; font-weight: normal }
.cap { font-size: 9pt; color: #0b0b0b }
.files { color: #52514e; font-size: 8.5pt }
.pb { break-before: page }
"""
body = [f"<h1>모듈 관리자 소프트웨어 검증 결과 시각화</h1>",
        "<p class='sub'>복합 로봇 시스템을 위한 기능 블록을 이용한 모듈형 시스템 (VIPFORUM 2026) 보조 자료, 2026년 10월 26일</p>",
        "<div class='note'><b>모두 소프트웨어 수준 검증입니다.</b> 모듈 관리자(bhl_module_manager)의 파이썬 코어로 장착, 분리 요청을 처리한 결과이며, "
        "실제 로봇, 실제 부품 탈착, 실제 ROS 2 실행은 포함하지 않습니다. 같은 코드와 스크립트로 누구나 다시 계산할 수 있습니다.</div>",
        "<table>" + "".join(f"<tr><th>{html.escape(a)}</th><td>{html.escape(b)}</td></tr>" for a, b in rows) + "</table>"]
for i, (name, title, cap) in enumerate(FIGS):
    body.append(f"<div class='fig{' pb' if i in (1, 3, 5) else ''}'><img src='{name}.svg'>"
                f"<div class='cap'><b>{html.escape(title)}.</b> {html.escape(cap)}</div></div>")
body.append("<p class='files'>파일: 그림마다 SVG(글자를 도형으로 변환), PDF(글꼴 포함), PNG(300 dpi)와 수치 CSV. 데이터 재계산: collect.py "
            "(모듈 관리자 패키지와 BHL 원본 URDF 경로를 인자로 받음).</p>")
doc = f"<!doctype html><html lang='ko'><head><meta charset='utf-8'><title>모듈 관리자 소프트웨어 검증 시각화</title><style>{css}</style></head><body>{''.join(body)}</body></html>"
path = os.path.join(V, "out", "overview.html")
open(path, "w").write(doc)
with sync_playwright() as p:
    b = p.chromium.launch(); pg = b.new_page()
    pg.goto("file://" + os.path.abspath(path)); pg.wait_for_timeout(800)
    pg.evaluate("document.fonts.ready")
    pg.pdf(path=OUT_PDF, format="A4", print_background=True, prefer_css_page_size=True)
    b.close()
print("written", OUT_PDF)
