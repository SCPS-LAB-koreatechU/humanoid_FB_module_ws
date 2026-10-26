"""Figures for the software-level verification of the module manager (static, for the paper and slides).

usage: python3 figs.py <data dir> <renders dir> <out dir>
Each figure is written as SVG (text as paths), PDF (fonts embedded) and PNG (300 dpi), with its numbers in a CSV.
Palette (dataviz reference, validated): blue #2a78d6 = available / executed; status serious #ec835a with a cross icon =
rejected (same meaning as the orange 거부 처리 in figure 1 of the paper); grays for unavailable and absent.
"""
import csv, json, os, sys
from collections import defaultdict
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
from matplotlib.patches import Circle, FancyBboxPatch, Rectangle
from matplotlib.path import Path
from PIL import Image, ImageChops

DATA, RENDERS, OUT = sys.argv[1], sys.argv[2], sys.argv[3]
os.makedirs(OUT, exist_ok=True)
HERE = os.path.dirname(os.path.abspath(__file__))
for f in ("VizSansKR", "VizSansKRMedium", "VizSansKRBold"):
    fm.fontManager.addfont(os.path.join(HERE, "fonts", f + ".ttf"))
R = fm.FontProperties(family="VizSansKR")
M = fm.FontProperties(family="VizSansKR Medium", weight=500)
B = fm.FontProperties(family="VizSansKR Bold", weight=700)
MONO = fm.FontProperties(family="DejaVu Sans Mono")

INK, INK2, MUTED = "#0b0b0b", "#52514e", "#898781"
GRID, AXIS = "#e1e0d9", "#c3c2b7"
BLUE, BLUE_LIGHT = "#2a78d6", "#cde2fb"
REJECT = "#ec835a"                       # status: serious
plt.rcParams.update({
    "font.family": "VizSansKR", "font.size": 8, "axes.unicode_minus": True,
    "svg.fonttype": "path", "pdf.fonttype": 42,
    "axes.edgecolor": AXIS, "axes.linewidth": 0.6, "axes.labelcolor": INK2, "axes.titlesize": 9,
    "xtick.color": INK2, "ytick.color": INK2, "xtick.labelcolor": INK2, "ytick.labelcolor": INK2,
    "xtick.major.width": 0.6, "ytick.major.width": 0.6, "xtick.major.size": 2.5, "ytick.major.size": 0,
    "grid.color": GRID, "grid.linewidth": 0.6, "figure.facecolor": "white", "savefig.facecolor": "white",
})
MM = 1 / 25.4


def save(fig, name, png_dpi=300):
    for ext in ("svg", "pdf"):
        fig.savefig(os.path.join(OUT, f"{name}.{ext}"))
    fig.savefig(os.path.join(OUT, f"{name}.png"), dpi=png_dpi)
    plt.close(fig)
    print("saved", name)


CHECK = Path([(-0.46, 0.02), (-0.14, -0.30), (0.46, 0.30)], [Path.MOVETO, Path.LINETO, Path.LINETO])


def status_icon(ax, x, y, ok, size=7.5, transform=None):
    """status mark: blue circle with a check (executed) or orange circle with a cross (rejected); size in points"""
    tr = transform or ax.transData
    ax.plot([x], [y], marker="o", ms=size, mfc=BLUE if ok else REJECT, mec="none", transform=tr, zorder=5, clip_on=False)
    if ok:
        ax.plot([x], [y], marker=CHECK, ms=size * 0.58, mfc="none", mec="white", mew=size * 0.15, transform=tr, zorder=6,
                clip_on=False, solid_capstyle="round", solid_joinstyle="round")
    else:
        ax.plot([x], [y], marker="x", ms=size * 0.40, mfc="none", mec="white", mew=size * 0.15, transform=tr, zorder=6,
                clip_on=False, solid_capstyle="round")


def clean(ax, left=True, bottom=True):
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    ax.spines["left"].set_visible(left)
    ax.spines["bottom"].set_visible(bottom)


caps = json.load(open(os.path.join(DATA, "capabilities.json")))
timeline = json.load(open(os.path.join(DATA, "timeline.json")))
summary = json.load(open(os.path.join(DATA, "summary.json")))
CFG_NAME = {"A": "기본 구성", "B": "오른손 → 그리퍼", "C": "오른팔 분리", "D": "양손 분리", "E": "목, 카메라 분리",
            "E′": "카메라만 분리"}
PLAN = {"hand_over(R_hand)": (True, "오른손 FB로 수행"), "hand_over(R_gripper)": (True, "그리퍼 FB로 수행"),
        "hand_over(L_hand)": (True, "왼손 FB로 수행"), "reject(no_grasp)": (False, "거부: 파지 수단 없음"),
        "reject(no_vision)": (False, "거부: 카메라 없음")}


# ===================================================================== S1. configurations A-E with the outcome
def crop_box(im, thr=10):
    d = ImageChops.difference(im.convert("RGB"), Image.new("RGB", im.size, "white")).convert("L").point(lambda v: 255 if v > thr else 0)
    return d.getbbox()


def fig_s1():
    keys = ["A", "B", "C", "D", "E"]
    ims = [Image.open(os.path.join(RENDERS, f"f_{k}.png")).convert("RGB") for k in keys]
    bbs = [crop_box(im) for im in ims]
    ub = (min(b[0] for b in bbs) - 20, min(b[1] for b in bbs) - 20, max(b[2] for b in bbs) + 20, max(b[3] for b in bbs) + 20)
    crops = [np.asarray(im.crop(ub)) for im in ims]
    h, w = crops[0].shape[:2]
    W = 170 * MM; panel_w = W / 5; img_h = panel_w * h / w; text_h = 17 * MM

    def build(interp):
        fig = plt.figure(figsize=(W, img_h + text_h))
        for i, (k, c) in enumerate(zip(keys, crops)):
            ax = fig.add_axes([i / 5, text_h / (img_h + text_h), 1 / 5, img_h / (img_h + text_h)])
            ax.imshow(c, interpolation=interp); ax.axis("off")
            t = fig.add_axes([i / 5, 0, 1 / 5, text_h / (img_h + text_h)]); t.set_xlim(0, 100); t.set_ylim(0, 100); t.axis("off")
            t.text(50, 84, f"({k}) {CFG_NAME[k]}", ha="center", va="center", fontproperties=B, fontsize=8.5, color=INK)
            ok, txt = PLAN[caps[k]["plan"]]
            tw = len(txt) * 6.1
            x0 = 50 - (tw + 10) / 2
            status_icon(t, x0 + 3.2, 50, ok, size=8.5)
            t.text(x0 + 10, 50, txt, ha="left", va="center", fontproperties=M, fontsize=7.6, color=INK)
            t.text(50, 17, f"가능 능력 {len(caps[k]['available'])}개", ha="center", va="center", fontproperties=R, fontsize=7, color=MUTED)
        return fig

    name = "S1_구성별_처리결과"
    fig = build("none")                      # vector files keep the renders at their own resolution
    for ext in ("svg", "pdf"):
        fig.savefig(os.path.join(OUT, f"{name}.{ext}"))
    plt.close(fig)
    fig = build("lanczos")
    fig.savefig(os.path.join(OUT, f"{name}.png"), dpi=300)
    plt.close(fig)
    print("saved", name, crops[0].shape)
    with open(os.path.join(OUT, "S1_구성별_처리결과.csv"), "w", newline="", encoding="utf-8-sig") as f:
        w_ = csv.writer(f); w_.writerow(["구성", "부품 변경", "빼빼로 건네줘 결과", "가능 능력 수"])
        for k in keys:
            w_.writerow([k, CFG_NAME[k], PLAN[caps[k]["plan"]][1], len(caps[k]["available"])])


# ===================================================================== S2. capability matrix
GROUPS = [("인식", ["detect(object)", "locate_hand(person)", "look_at(point)"]),
          ("오른쪽", ["wave(R_arm)", "open_hand(R_hand)", "handshake(R_arm, R_hand)", "grasp(R_hand)", "hand_over(R_hand)",
                   "grasp(R_gripper)", "hand_over(R_gripper)"]),
          ("왼쪽", ["wave(L_arm)", "open_hand(L_hand)", "handshake(L_arm, L_hand)", "grasp(L_hand)", "hand_over(L_hand)"])]
COLS = ["A", "B", "C", "D", "E", "E′"]


def fig_s2():
    rows = [(g, s) for g, ss in GROUPS for s in ss]
    n = len(rows)
    W, H = 170 * MM, 122 * MM
    fig = plt.figure(figsize=(W, H))
    ax = fig.add_axes([0, 0, 1, 1]); ax.set_xlim(0, 170); ax.set_ylim(122, 0); ax.axis("off")
    x_group, x_skill, x0, dx = 2, 15, 69, 16.6
    y_head, y0, dy = 8, 24, 4.55
    cx = [x0 + dx * (i + 0.5) for i in range(len(COLS))]
    for i, c in enumerate(COLS):
        ax.text(cx[i], y_head - 2.6, c, ha="center", va="center", fontproperties=B, fontsize=9, color=INK)
        name = {"A": "기본\n구성", "B": "오른손 →\n그리퍼", "C": "오른팔\n분리", "D": "양손\n분리", "E": "목, 카메라\n분리",
                "E′": "카메라만\n분리"}[c]
        ax.text(cx[i], y_head + 4.4, name, ha="center", va="center", fontproperties=R, fontsize=6.6, color=INK2, linespacing=1.15)
    ax.plot([x_group, 168], [y0 - 4.2, y0 - 4.2], color=AXIS, lw=0.6)
    y = y0
    csv_rows = []
    for gi, (g, ss) in enumerate(GROUPS):
        if gi:
            ax.plot([x_group, 168], [y - dy / 2 - 0.7, y - dy / 2 - 0.7], color=GRID, lw=0.6)
            y += 1.4
        ax.text(x_group, y + (len(ss) - 1) * dy / 2, g, ha="left", va="center", fontproperties=M, fontsize=7.4, color=INK2)
        for s in ss:
            ax.text(x_skill, y, s, ha="left", va="center", fontproperties=MONO, fontsize=6.6, color=INK)
            row = [g, s]
            for i, c in enumerate(COLS):
                st = caps[c]
                chosen = st["plan"] == s
                if s in st["available"]:
                    if chosen:
                        ax.add_patch(Circle((cx[i], y), 2.05, facecolor="none", edgecolor=BLUE, lw=1.0))
                    ax.add_patch(Circle((cx[i], y), 1.25, facecolor=BLUE, edgecolor="white", lw=0.8))
                    row.append("선택" if chosen else "가능")
                elif s in st["unavailable"]:
                    ax.add_patch(Circle((cx[i], y), 1.15, facecolor="white", edgecolor=MUTED, lw=0.8))
                    row.append("조건 미충족: " + st["unavailable"][s])
                else:
                    ax.plot([cx[i] - 1.2, cx[i] + 1.2], [y, y], color=AXIS, lw=0.8, solid_capstyle="round")
                    row.append("모듈 없음")
            csv_rows.append(row)
            y += dy
    # outcome row
    y += 1.6
    ax.plot([x_group, 168], [y - 2.6, y - 2.6], color=AXIS, lw=0.6)
    ax.text(x_group, y + 1.2, "“빼빼로 건네줘”", ha="left", va="center", fontproperties=B, fontsize=7.4, color=INK)
    ax.text(x_group, y + 5.4, "실행 결과", ha="left", va="center", fontproperties=R, fontsize=6.8, color=INK2)
    for i, c in enumerate(COLS):
        ok, txt = PLAN[caps[c]["plan"]]
        status_icon(ax, cx[i], y + 0.6, ok, size=8.0)
        short = txt.replace(" FB로 수행", " FB").replace("거부: ", "")
        ax.text(cx[i], y + 5.2, short, ha="center", va="center", fontproperties=M, fontsize=6.6, color=INK)
    y += 10.5
    ax.text(x_group, y, "가능 능력 수", ha="left", va="center", fontproperties=R, fontsize=6.8, color=INK2)
    for i, c in enumerate(COLS):
        ax.text(cx[i], y, f"{len(caps[c]['available'])}", ha="center", va="center", fontproperties=M, fontsize=7.4, color=INK)
    # legend
    y += 8.5
    lx = x_group
    items = [("dot", "가능"), ("ring", "선택된 실행 경로"), ("hollow", "모듈은 있으나 선행 능력 없음"), ("dash", "모듈 없음")]
    for kind, label in items:
        if kind == "dot":
            ax.add_patch(Circle((lx + 1.3, y), 1.25, facecolor=BLUE, edgecolor="white", lw=0.8))
        elif kind == "ring":
            ax.add_patch(Circle((lx + 1.3, y), 2.05, facecolor="none", edgecolor=BLUE, lw=1.0))
            ax.add_patch(Circle((lx + 1.3, y), 1.25, facecolor=BLUE, edgecolor="white", lw=0.8))
        elif kind == "hollow":
            ax.add_patch(Circle((lx + 1.3, y), 1.15, facecolor="white", edgecolor=MUTED, lw=0.8))
        else:
            ax.plot([lx + 0.1, lx + 2.5], [y, y], color=AXIS, lw=0.8, solid_capstyle="round")
        t = ax.text(lx + 4.2, y, label, ha="left", va="center", fontproperties=R, fontsize=6.8, color=INK2)
        lx += 4.2 + len(label) * 2.3 + 6
    save(fig, "S2_능력목록_행렬")
    with open(os.path.join(OUT, "S2_능력목록_행렬.csv"), "w", newline="", encoding="utf-8-sig") as f:
        w_ = csv.writer(f); w_.writerow(["묶음", "능력"] + COLS); w_.writerows(csv_rows)


# ===================================================================== S3. reconfiguration timeline (small multiples)
def fig_s3():
    xs = np.arange(len(timeline))
    series = [("frames", "TF 프레임 수", (18, 36), [20, 24, 28, 32]), ("joints", "구동 관절 수", (5.5, 13.8), [6, 8, 10, 12]),
              ("capabilities", "가능 능력 수", (3, 15.5), [4, 8, 12])]
    W, H = 170 * MM, 112 * MM
    fig = plt.figure(figsize=(W, H))
    left, right, top, bottom = 0.085, 0.985, 0.94, 0.215
    ph = (top - bottom) / 3
    axes = []
    for k, (key, title, ylim, ticks) in enumerate(series):
        ax = fig.add_axes([left, top - (k + 1) * ph + 0.02, right - left, ph - 0.04])
        v = np.array([r[key] for r in timeline])
        ax.step(xs, v, where="mid", color=BLUE, lw=1.5, solid_joinstyle="round", zorder=2)
        ax.plot(xs, v, "o", ms=4.6, mfc=BLUE, mec="white", mew=1.1, zorder=3)
        base = v[0]
        span = ylim[1] - ylim[0]
        for x, val in zip(xs, v):
            if val != base:
                ax.text(x, val + span * 0.07, f"{val}", ha="center", va="bottom", fontproperties=M, fontsize=7, color=INK)
        ax.text(xs[0], base + span * 0.07, f"{base}", ha="center", va="bottom", fontproperties=R, fontsize=6.8, color=INK2)
        ax.set_ylim(*ylim); ax.set_yticks(ticks); ax.set_xlim(-0.5, len(xs) - 0.2)
        ax.grid(axis="y"); ax.set_axisbelow(True)
        clean(ax, left=False, bottom=(k == 2))
        ax.tick_params(axis="x", length=0 if k < 2 else 2.5, labelbottom=(k == 2))
        for lab in ax.get_yticklabels():
            lab.set_fontproperties(R); lab.set_fontsize(6.8)
        ax.text(-0.45, ylim[1] + (ylim[1] - ylim[0]) * 0.02, title, ha="left", va="bottom", fontproperties=M, fontsize=7.6, color=INK)
        axes.append(ax)
    # rejected attempts at step 8 (no state change)
    ax0 = axes[0]
    i8 = next(r["i"] for r in timeline if r["rejected"])
    for j in range(3):
        status_icon(ax0, i8 - 0.2 + j * 0.2, 25.2, False, size=6.2)
    ax0.text(i8, 23.4, "잘못된 장착\n3건 거부", ha="center", va="top", fontproperties=R, fontsize=6.4, color=INK2, linespacing=1.1)
    labels = []
    for r in timeline:
        name = {"기본 구성": "기본", "복귀": "복귀", "목, 카메라 분리": "목, 카메라\n분리", "잘못된 장착 3건 거부 후 복귀": "거부 후\n복귀",
                "카메라만 분리": "카메라만\n분리", "오른손 → 그리퍼": "오른손 →\n그리퍼", "오른팔 분리": "오른팔\n분리", "양손 분리": "양손\n분리"}[r["label"]]
        labels.append(f"{r['config']}\n{name}")
    ax = axes[-1]
    ax.set_xticks(xs); ax.set_xticklabels(labels)
    for i, lab in enumerate(ax.get_xticklabels()):
        lab.set_fontproperties(R); lab.set_fontsize(6.7); lab.set_linespacing(1.15)
        lab.set_color(INK if timeline[i]["config"] != "A" else INK2)
    fig.text(left, 0.02, "가로축: 장착, 분리 요청 순서 (구성 A에서 시작해 각 구성으로 바꾼 뒤 A로 복귀). 모든 변경은 장착, 분리 요청만으로 처리했고 사용자 코드와 명세 파일은 수정하지 않음.",
             ha="left", va="bottom", fontproperties=R, fontsize=6.5, color=MUTED)
    save(fig, "S3_재구성_타임라인")
    with open(os.path.join(DATA, "timeline.csv"), encoding="utf-8-sig") as f, open(os.path.join(OUT, "S3_재구성_타임라인.csv"), "w", encoding="utf-8-sig") as g:
        g.write(f.read())


# ===================================================================== S4. arm kinematics vs the original URDF
def fig_s4():
    rows = list(csv.reader(open(os.path.join(DATA, "kinematics_error.csv"), encoding="utf-8-sig")))[1:]
    data = defaultdict(list); rot = defaultdict(list)
    for side, _, p, r in rows:
        data[side].append(float(p)); rot[side].append(float(r))
    W, H = 170 * MM, 68 * MM
    fig = plt.figure(figsize=(W, H))
    for k, (side, name) in enumerate((("R", "오른팔"), ("L", "왼팔"))):
        ax = fig.add_axes([0.075 + k * 0.49, 0.27, 0.40, 0.58])
        v = np.sort(np.array(data[side])); y = np.arange(1, len(v) + 1) / len(v) * 100
        ax.step(v, y, where="post", color=BLUE, lw=1.5)
        ax.fill_between(v, y, step="post", color=BLUE, alpha=0.10, lw=0)
        mx, med = v[-1], np.median(v)
        ax.plot([mx], [100], "o", ms=4.6, mfc=BLUE, mec="white", mew=1.1, zorder=4)
        ax.text(7.95, 86, f"최대 {mx:.1f} µm", ha="right", va="top", fontproperties=M, fontsize=7, color=INK)
        ax.plot([mx, mx], [100, 88], color=AXIS, lw=0.6)
        ax.plot([med, med], [0, 50], color=AXIS, lw=0.6); ax.plot([0, med], [50, 50], color=AXIS, lw=0.6)
        ax.text(med + 0.12, 46, f"중앙값 {med:.1f} µm", ha="left", va="top", fontproperties=R, fontsize=6.6, color=INK2)
        ax.set_xlim(0, 8); ax.set_ylim(0, 104); ax.set_xticks([0, 2, 4, 6, 8]); ax.set_yticks([0, 25, 50, 75, 100])
        ax.set_yticklabels(["0", "25", "50", "75", "100%"])
        ax.grid(axis="both"); ax.set_axisbelow(True); clean(ax)
        for lab in ax.get_xticklabels() + ax.get_yticklabels():
            lab.set_fontproperties(R); lab.set_fontsize(6.8)
        ax.set_xlabel("손끝 좌표계 위치 오차 (µm)", fontproperties=R, fontsize=7, color=INK2, labelpad=3)
        ax.text(0, 112, f"({'a' if k == 0 else 'b'}) {name}", ha="left", va="bottom", fontproperties=B, fontsize=8, color=INK,
                transform=ax.transData)
        ax.text(8, 112, f"자세 {len(v):,}개, 자세 오차 최대 {max(rot[side]):.4f}°", ha="right", va="bottom", fontproperties=R,
                fontsize=6.6, color=INK2, transform=ax.transData)
    fig.text(0.075, 0.03, "세로축: 오차가 해당 값 이하인 자세의 누적 비율. 관절 기구 범위 안 무작위 자세에서, 모듈 명세의 DH(반올림한 값)로 생성한 기구학과\nBHL 원본 URDF 순기구학을 비교함.",
             ha="left", va="bottom", fontproperties=R, fontsize=6.4, color=MUTED)
    save(fig, "S4_팔기구학_원본URDF_비교")
    with open(os.path.join(OUT, "S4_팔기구학_원본URDF_비교.csv"), "w", newline="", encoding="utf-8-sig") as f:
        w_ = csv.writer(f); w_.writerow(["팔", "자세 수", "위치 오차 중앙값 (µm)", "위치 오차 95% (µm)", "위치 오차 최대 (µm)", "자세 오차 최대 (도)"])
        for side, name in (("R", "오른팔"), ("L", "왼팔")):
            v = np.array(data[side]); w_.writerow([name, len(v), round(float(np.median(v)), 3), round(float(np.percentile(v, 95)), 3),
                                                   round(float(v.max()), 3), round(max(rot[side]), 5)])


# ===================================================================== S5. processing time per request
def fig_s5():
    rows = list(csv.reader(open(os.path.join(DATA, "processing_time_ms.csv"), encoding="utf-8-sig")))[1:]
    d = defaultdict(list)
    for k, v in rows:
        d[k].append(float(v))
    order = [("장착 (명세 검증)", "장착 요청 검증"), ("분리 (검증, 하위 트리 제거)", "분리 (하위 트리 제거)"),
             ("능력 목록 재평가", "능력 목록 재평가"), ("URDF 생성 (BHL 메쉬 포함)", "URDF 생성 (메쉬 포함)"),
             ("요청 1회 전체", "요청 1회 전체")]
    W, H = 170 * MM, 60 * MM
    fig = plt.figure(figsize=(W, H))
    ax = fig.add_axes([0.2, 0.26, 0.68, 0.68])
    out = []
    for i, (key, name) in enumerate(order):
        v = np.array(d[key]); p5, p50, p95 = np.percentile(v, [5, 50, 95])
        y = len(order) - 1 - i
        total = key == "요청 1회 전체"
        ax.plot([p5, p95], [y, y], color=BLUE, lw=1.5, solid_capstyle="round", alpha=1.0)
        ax.plot([p50], [y], "o", ms=5.2, mfc=BLUE, mec="white", mew=0.7, zorder=3)
        lab = f"{p50:.3f} ms" if p50 < 0.1 else f"{p50:.1f} ms"
        ax.text(p95 * 1.35, y, lab, ha="left", va="center", fontproperties=M if total else R, fontsize=7.2 if total else 6.8, color=INK)
        out.append([name, len(v), p5, p50, p95, v.max()])
    ax.set_xscale("log"); ax.set_xlim(0.004, 40)
    ax.set_xticks([0.01, 0.1, 1, 10]); ax.set_xticklabels(["0.01", "0.1", "1", "10 ms"])
    ax.set_yticks(range(len(order))); ax.set_yticklabels([n for _, n in order][::-1])
    for i, lab in enumerate(ax.get_yticklabels()):
        lab.set_fontproperties(B if i == 0 else R); lab.set_fontsize(7.2); lab.set_color(INK if i == 0 else INK2)
    for lab in ax.get_xticklabels():
        lab.set_fontproperties(R); lab.set_fontsize(6.8)
    ax.set_ylim(-0.6, len(order) - 0.4)
    ax.grid(axis="x", which="major"); ax.set_axisbelow(True); clean(ax, left=False)
    ax.tick_params(axis="y", length=0)
    env = summary["environment"]
    fig.text(0.2, 0.03, f"점: 중앙값, 선: 5~95% 범위 (항목별 {min(len(x) for x in d.values()):,}~{max(len(x) for x in d.values()):,}회). 파이썬 코어만 측정, "
             f"ROS 통신과 robot_state_publisher 반영 시간은 제외.\n측정 환경: Intel Xeon 2.1 GHz, Python {env['python']} (클라우드 작업 환경, 실제 제어 PC 값은 별도 측정 필요).",
             ha="left", va="bottom", fontproperties=R, fontsize=6.3, color=MUTED, linespacing=1.4)
    ax.minorticks_off()
    save(fig, "S5_요청_처리시간")
    with open(os.path.join(OUT, "S5_요청_처리시간.csv"), "w", newline="", encoding="utf-8-sig") as f:
        w_ = csv.writer(f); w_.writerow(["항목", "횟수", "5% (ms)", "중앙값 (ms)", "95% (ms)", "최대 (ms)"])
        w_.writerows([[a, b] + [round(x, 4) for x in rest] for a, b, *rest in out])


# ===================================================================== S6. what the change means for the URDF
def fig_s6():
    rows = list(csv.reader(open(os.path.join(DATA, "urdf_changes.csv"), encoding="utf-8-sig")))
    head, rows = rows[0], rows[1:]
    W, H = 170 * MM, 66 * MM
    fig = plt.figure(figsize=(W, H))
    ax = fig.add_axes([0.205, 0.27, 0.47, 0.62])
    ax.set_xlim(0, 124); ax.set_ylim(-0.6, len(rows) - 0.4)
    from matplotlib.transforms import blended_transform_factory
    side = blended_transform_factory(fig.transFigure, ax.transData)
    for i, r in enumerate(rows):
        c, plus, minus, la, lr, ja, jr = r[0], int(r[1]), int(r[2]), int(r[3]), int(r[4]), int(r[5]), int(r[6])
        y = len(rows) - 1 - i; total = plus + minus; h = 0.44
        ax.add_patch(FancyBboxPatch((0, y - h / 2), total, h, boxstyle="round,pad=0,rounding_size=1.1",
                                    mutation_aspect=h / 2.2, facecolor=BLUE, edgecolor="none"))
        ax.add_patch(Rectangle((0, y - h / 2), min(total, 4), h, facecolor=BLUE, edgecolor="none"))
        ax.text(total + 1.6, y, f"{total}줄", ha="left", va="center", fontproperties=M, fontsize=7.4, color=INK)
        detail = (f"링크, 조인트 {la}개씩 교체 (추가 {plus}, 삭제 {minus}줄)" if plus and minus
                  else f"링크 {lr}개, 조인트 {jr}개 삭제")
        ax.text(0.70, y, detail, ha="left", va="center", fontproperties=R, fontsize=6.9, color=INK2, transform=side, clip_on=False)
    fig.text(0.70, 0.93, "URDF 구조 변화", ha="left", va="center", fontproperties=M, fontsize=7.2, color=INK2)
    ax.set_yticks(range(len(rows)))
    ax.set_yticklabels([f"({r[0]}) {CFG_NAME[r[0]]}" for r in rows][::-1])
    for lab in ax.get_yticklabels():
        lab.set_fontproperties(R); lab.set_fontsize(7.2); lab.set_color(INK)
    ax.set_xticks([0, 25, 50, 75, 100])
    for lab in ax.get_xticklabels():
        lab.set_fontproperties(R); lab.set_fontsize(6.8)
    ax.grid(axis="x"); ax.set_axisbelow(True); clean(ax, left=False); ax.tick_params(axis="y", length=0)
    ax.set_xlabel("A 구성 URDF 대비 바뀌는 줄 수 (A: 301줄, 링크 44개, 조인트 43개)", fontproperties=R, fontsize=7, color=INK2, labelpad=3)
    fig.text(0.205, 0.03, "자동 생성된 URDF끼리 비교한 값으로, 정적 URDF를 쓰면 사람이 직접 고쳐야 하는 양에 해당함.\n"
             "제안 방식에서는 장착, 분리 요청 1회로 반영되며 사용자 코드와 명세 파일은 수정하지 않음.",
             ha="left", va="bottom", fontproperties=R, fontsize=6.3, color=MUTED, linespacing=1.4)
    save(fig, "S6_URDF_변경량")
    with open(os.path.join(DATA, "urdf_changes.csv"), encoding="utf-8-sig") as f, open(os.path.join(OUT, "S6_URDF_변경량.csv"), "w", encoding="utf-8-sig") as g:
        g.write(f.read())


# ===================================================================== S7. rejected attach requests
def fig_s7():
    cases = [("ZED 2i를 몸통의 목 포트(torso.N)에", "바로 결합", "기계: 포트 타입", "포트 타입 불일치",
              "torso.N은 head_mount, ZED 2i는 camera_1_4_20 포트에만 결합"),
             ("ZED 2i 드라이버를 RDK X5에 배치", "", "전기: 호스트 요구", "호스트 요구 불충족",
              "RDK X5에 CUDA 없음 (ZED 2i 드라이버는 CUDA 필요)"),
             ("DexHand를 왼쪽 어깨 포트(torso.S_L)에", "결합", "기계: 포트 점유, 타입", "포트 사용 중, 포트 타입 불일치",
              "torso.S_L에는 l_arm이 장착됨, 손은 hand_d84 포트에만 결합"),
             ("왼팔 명세를 오른쪽 어깨 포트(torso.S_R)에", "결합", "기계: 좌우", "좌우 불일치", "포트는 오른쪽(R), 모듈 명세는 왼쪽(L)")]
    W, H = 170 * MM, 64 * MM
    fig = plt.figure(figsize=(W, H))
    ax = fig.add_axes([0, 0, 1, 1]); ax.set_xlim(0, 170); ax.set_ylim(64, 0); ax.axis("off")
    xs = [2, 64, 94]
    for x, h in zip(xs, ["장착 요청", "검사 도메인", "결과"]):
        ax.text(x, 5, h, ha="left", va="center", fontproperties=M, fontsize=7.4, color=INK2)
    ax.plot([2, 168], [9, 9], color=AXIS, lw=0.6)
    y, dy = 15.5, 12.4
    for i, (a1, a2, b, c1, c2) in enumerate(cases):
        ax.text(xs[0], y - 2.1, a1, ha="left", va="center", fontproperties=R, fontsize=7.2, color=INK)
        if a2:
            ax.text(xs[0], y + 2.1, a2, ha="left", va="center", fontproperties=R, fontsize=7.2, color=INK)
        ax.text(xs[1], y - 2.1, b, ha="left", va="center", fontproperties=R, fontsize=7.2, color=INK2)
        status_icon(ax, xs[2] + 1.6, y - 2.1, False, size=7.5)
        ax.text(xs[2] + 4.8, y - 2.1, "거부: " + c1, ha="left", va="center", fontproperties=M, fontsize=7.2, color=INK)
        ax.text(xs[2] + 4.8, y + 2.1, c2, ha="left", va="center", fontproperties=R, fontsize=6.6, color=INK2)
        if i < len(cases) - 1:
            ax.plot([2, 168], [y + dy / 2, y + dy / 2], color=GRID, lw=0.6)
        y += dy
    ax.plot([2, 168], [y - dy / 2, y - dy / 2], color=AXIS, lw=0.6)
    save(fig, "S7_장착검증_거부사례")
    with open(os.path.join(OUT, "S7_장착검증_거부사례.csv"), "w", newline="", encoding="utf-8-sig") as f:
        w_ = csv.writer(f); w_.writerow(["장착 요청", "검사 도메인", "거부 사유", "세부"])
        w_.writerows([[a1 + (" " + a2 if a2 else ""), b, c1, c2] for a1, a2, b, c1, c2 in cases])


if __name__ == "__main__":
    which = sys.argv[4:] or ["s1", "s2", "s3", "s4", "s5", "s6", "s7"]
    for k in which:
        globals()["fig_" + k]()
