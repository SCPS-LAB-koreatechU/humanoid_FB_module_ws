"""Software-level verification data for the module manager (bhl_module_manager), written to viz/data.

usage: python3 collect.py <package dir> <BHL urdf> <out dir>
Everything here runs the package's pure-Python core (no ROS): the plug scenario A -> E, the capability
matrix, rejected attach attempts, URDF and TF changes per configuration, arm kinematics against the
original URDF, and processing time per request.
"""
import csv, difflib, gc, json, math, os, platform, statistics, sys, time
import numpy as np

PKG, BHL_URDF, OUT = sys.argv[1], sys.argv[2], sys.argv[3]
sys.path.insert(0, PKG)
from bhl_module_manager import (Assembly, build, diff, evaluate, load_hosts, load_library, parse_urdf, plan_hand_over,
                                to_urdf, urdf_fk, se3)

os.makedirs(OUT, exist_ok=True)
LIB = load_library(os.path.join(PKG, "modules"))
HOSTS = load_hosts(os.path.join(PKG, "hosts", "hosts.yaml"))
CFG_A = os.path.join(PKG, "configs", "A_full.yaml")


def full():
    return build(CFG_A, LIB, HOSTS)[0]


def frames(asm):
    return {n for n, _, _ in asm.tree()}


def write_csv(name, header, rows):
    with open(os.path.join(OUT, name), "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f); w.writerow(header); w.writerows(rows)


# ------------------------------------------------------------------ 1. plug scenario (same steps as demo_plug.py)
HAND = ("hand_dexhand_v1",)
STEPS = [
    ("A", "기본 구성", []),
    ("B", "오른손 → 그리퍼", [("detach", "r_hand"), ("attach", "r_conn.hand", "gripper_placeholder", "r_grip")]),
    ("A", "복귀", [("detach", "r_grip"), ("attach", "r_conn.hand", "hand_dexhand_v1", "r_hand", None, "control_pc")]),
    ("C", "오른팔 분리", [("detach", "r_arm")]),
    ("A", "복귀", [("attach", "torso.S_R", "arm_bhl", "r_arm", None, "control_pc"),
                  ("attach", "r_arm.flange", "connector_d84", "r_conn"),
                  ("attach", "r_conn.hand", "hand_dexhand_v1", "r_hand", None, "control_pc")]),
    ("D", "양손 분리", [("detach", "r_hand"), ("detach", "l_hand")]),
    ("A", "복귀", [("attach", "r_conn.hand", "hand_dexhand_v1", "r_hand", None, "control_pc"),
                  ("attach", "l_conn.hand", "hand_dexhand_v1", "l_hand", None, "control_pc")]),
    ("E", "목, 카메라 분리", [("detach", "neck")]),
    ("A", "잘못된 장착 3건 거부 후 복귀", [
        ("attach", "torso.N", "head_zed2i", "head", None, "control_pc"),
        ("attach", "torso.N", "neck_pan_tilt_scs", "neck", None, "control_pc"),
        ("attach", "neck.mount", "head_zed2i", "head", None, "rdk_x5"),
        ("attach", "torso.S_L", "hand_dexhand_v1", "x_hand", None, "control_pc"),
        ("attach", "neck.mount", "head_zed2i", "head", None, "control_pc")]),
    ("E′", "카메라만 분리", [("detach", "head")]),
]
asm = full()
caps = evaluate(asm)
timeline, rejections, states = [], [], {}
for i, (cfg, label, actions) in enumerate(STEPS):
    before = frames(asm)
    acts = []
    for op, *a in actions:
        r = asm.attach(*a) if op == "attach" else asm.detach(*a)
        acts.append({"op": op, "args": [x for x in a if x is not None], "ok": bool(r.ok), "msg": r.events if r.ok else r.reasons})
        if not r.ok:
            rejections.append({"step": i, "attempt": f"{a[1]} → {a[0]}" + (f" (호스트 {a[4]})" if len(a) > 4 and a[4] else ""),
                               "reasons": r.reasons})
    new = evaluate(asm)
    added, removed = diff(caps, new)
    caps = new
    after = frames(asm)
    plan = plan_hand_over(caps)
    rec = {"i": i, "config": cfg, "label": label, "actions": acts, "frames": len(after), "joints": len(asm.joint_names()),
           "capabilities": len(caps["available"]), "frames_added": len(after - before), "frames_removed": len(before - after),
           "caps_added": added, "caps_removed": removed, "plan": plan["call"], "plan_ok": plan["ok"],
           "rejected": sum(1 for x in acts if not x["ok"])}
    timeline.append(rec)
    if cfg not in states:
        states[cfg] = {"available": sorted(caps["available"]), "unavailable": caps["unavailable"], "plan": plan["call"],
                       "urdf": to_urdf(asm), "frames": sorted(after), "joints": asm.joint_names()}
json.dump(timeline, open(os.path.join(OUT, "timeline.json"), "w"), ensure_ascii=False, indent=1)
write_csv("timeline.csv", ["순서", "구성", "변경", "TF 프레임 수", "관절 수", "가능 능력 수", "프레임 추가", "프레임 제거", "거부된 장착", "빼빼로 건네줘 결과"],
          [[r["i"], r["config"], r["label"], r["frames"], r["joints"], r["capabilities"], r["frames_added"], r["frames_removed"],
            r["rejected"], r["plan"]] for r in timeline])

# side mismatch (from the test suite) completes the rejection cases
a2 = Assembly(LIB, HOSTS, "torso_bhl")
r = a2.attach("torso.S_R", "arm_bhl", "a", side="L", host="control_pc")
rejections.append({"step": None, "attempt": "arm_bhl(L) → torso.S_R", "reasons": r.reasons})
json.dump(rejections, open(os.path.join(OUT, "rejections.json"), "w"), ensure_ascii=False, indent=1)

# each configuration on its own, straight from A, so element order in the URDF stays comparable
SINGLE = {"A": [], "B": [("detach", "r_hand"), ("attach", "r_conn.hand", "gripper_placeholder", "r_grip")],
          "C": [("detach", "r_arm")], "D": [("detach", "r_hand"), ("detach", "l_hand")],
          "E": [("detach", "neck")], "E′": [("detach", "head")]}
for c, acts in SINGLE.items():
    a1 = full()
    for op, *a in acts:
        r = a1.attach(*a) if op == "attach" else a1.detach(*a)
        assert r.ok, (c, r.reasons)
    cp = evaluate(a1)
    assert sorted(cp["available"]) == states[c]["available"], c        # same capabilities as in the scenario
    states[c] = {"available": sorted(cp["available"]), "unavailable": cp["unavailable"], "plan": plan_hand_over(cp)["call"],
                 "urdf": to_urdf(a1), "frames": sorted(frames(a1)), "joints": a1.joint_names()}

# ------------------------------------------------------------------ 2. capability matrix
cols = ["A", "B", "C", "D", "E", "E′"]
skills = sorted({s for c in cols for s in states[c]["available"]} | {s for c in cols for s in states[c]["unavailable"]})
matrix = []
for s in skills:
    row = [s]
    for c in cols:
        st = states[c]
        row.append("가능" if s in st["available"] else ("불가: " + st["unavailable"][s] if s in st["unavailable"] else "모듈 없음"))
    matrix.append(row)
write_csv("capability_matrix.csv", ["능력"] + cols, matrix)
json.dump({c: {k: states[c][k] for k in ("available", "unavailable", "plan", "joints")} for c in cols},
          open(os.path.join(OUT, "capabilities.json"), "w"), ensure_ascii=False, indent=1)

# ------------------------------------------------------------------ 3. what changes in the URDF per configuration
def urdf_counts(text):
    m = parse_urdf(text)
    return len(m["links"]), len(m["joints"]), len([j for j in m["joints"] if j["type"] != "fixed" and not j["name"].startswith("attach__")])

base_urdf = states["A"]["urdf"]
urdf_rows = []
for c in cols[1:]:
    t = states[c]["urdf"]
    d = list(difflib.unified_diff(base_urdf.splitlines(), t.splitlines(), lineterm="", n=0))
    plus = sum(1 for x in d if x.startswith("+") and not x.startswith("+++"))
    minus = sum(1 for x in d if x.startswith("-") and not x.startswith("---"))
    la, ja, aa = urdf_counts(base_urdf); lc, jc, ac = urdf_counts(t)
    fa, fc = set(states["A"]["frames"]), set(states[c]["frames"])
    ma, mc = parse_urdf(base_urdf), parse_urdf(t)
    LA, LC = set(ma["links"]), set(mc["links"])
    JA, JC = {j["name"] for j in ma["joints"]}, {j["name"] for j in mc["joints"]}
    urdf_rows.append([c, plus, minus, len(LC - LA), len(LA - LC), len(JC - JA), len(JA - JC), aa, ac, len(fc - fa), len(fa - fc)])
write_csv("urdf_changes.csv", ["구성", "URDF 추가 줄", "URDF 삭제 줄", "링크 추가", "링크 제거", "조인트 추가", "조인트 제거",
                               "구동 관절 A", "구동 관절", "TF 프레임 추가", "TF 프레임 제거"], urdf_rows)
for c in cols:
    open(os.path.join(OUT, f"urdf_{c.replace('′', 'p')}.urdf"), "w").write(states[c]["urdf"])

# ------------------------------------------------------------------ 4. arm kinematics vs the original URDF
model = parse_urdf(BHL_URDF)
asmA = full()
rot_deg = lambda R: math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(R) - 1) / 2))))
rng = np.random.default_rng(0)
kin_rows, kin_sum = [], {}
for side, arm, hand in (("R", "r_arm", "arm_right_hand_link"), ("L", "l_arm", "arm_left_hand_link")):
    joints = asmA.instances[arm].mtype.joints_for(side)
    T0 = urdf_fk(model)[hand].copy(); T0[:3, 3] *= 1000
    X = se3.inv(T0) @ asmA.poses()[f"{arm}/F"]
    pe, re_ = [], []
    for k in range(2000):
        q = {j.hw: rng.uniform(math.radians(j.range[0]), math.radians(j.range[1])) for j in joints}
        Tb = urdf_fk(model, q)[hand].copy(); Tb[:3, 3] *= 1000
        D = se3.inv(Tb @ X) @ asmA.poses(q)[f"{arm}/F"]
        p, rr = float(np.linalg.norm(D[:3, 3])), rot_deg(D[:3, :3])
        pe.append(p); re_.append(rr); kin_rows.append([side, k, p * 1000, rr])
    kin_sum[side] = {"n": 2000, "max_um": max(pe) * 1000, "median_um": statistics.median(pe) * 1000,
                     "p95_um": float(np.percentile(pe, 95)) * 1000, "max_rot_deg": max(re_)}
write_csv("kinematics_error.csv", ["팔", "자세 번호", "위치 오차 (µm)", "자세 오차 (도)"], kin_rows)

# ------------------------------------------------------------------ 5. processing time per request (core only, no ROS)
def timed(fn):
    t = time.perf_counter_ns(); r = fn(); return r, (time.perf_counter_ns() - t) / 1e6

bench = full()
samples = {"분리 (검증, 하위 트리 제거)": [], "장착 (명세 검증)": [], "능력 목록 재평가": [], "URDF 생성 (BHL 메쉬 포함)": [], "요청 1회 전체": []}
REPS, WARM = 300, 20
gc.collect()
for k in range(REPS + WARM):
    for op in (lambda: bench.detach("r_hand"), lambda: bench.attach("r_conn.hand", "gripper_placeholder", "r_grip"),
               lambda: bench.detach("r_grip"), lambda: bench.attach("r_conn.hand", "hand_dexhand_v1", "r_hand", None, "control_pc")):
        t0 = time.perf_counter_ns()
        r, t_op = timed(op)
        assert r.ok, r.reasons
        c, t_ev = timed(lambda: evaluate(bench))
        _, t_pl = timed(lambda: plan_hand_over(c))
        _, t_ur = timed(lambda: to_urdf(bench, bhl_urdf=BHL_URDF))
        total = (time.perf_counter_ns() - t0) / 1e6
        if k >= WARM:
            samples["분리 (검증, 하위 트리 제거)" if "detach" in op.__code__.co_names else "장착 (명세 검증)"].append(t_op)
            samples["능력 목록 재평가"].append(t_ev + t_pl)
            samples["URDF 생성 (BHL 메쉬 포함)"].append(t_ur)
            samples["요청 1회 전체"].append(total)
write_csv("processing_time_ms.csv", ["항목", "시간 (ms)"], [[k, v] for k, vs in samples.items() for v in vs])
time_sum = {k: {"n": len(v), "median_ms": statistics.median(v), "p95_ms": float(np.percentile(v, 95)), "max_ms": max(v)}
            for k, v in samples.items()}

env = {"python": platform.python_version(), "machine": platform.machine(),
       "cpu": next((l.split(":", 1)[1].strip() for l in open("/proc/cpuinfo") if l.startswith("model name")), "?"),
       "numpy": np.__version__}
summary = {"kinematics": kin_sum, "processing_time": time_sum, "environment": env,
           "urdf_changes": urdf_rows, "rejections": rejections, "plans": {c: states[c]["plan"] for c in cols}}
json.dump(summary, open(os.path.join(OUT, "summary.json"), "w"), ensure_ascii=False, indent=1)
print(json.dumps({k: summary[k] for k in ("kinematics", "processing_time", "environment", "plans")}, ensure_ascii=False, indent=1))
print("urdf changes", urdf_rows)
print("rejections", json.dumps(rejections, ensure_ascii=False, indent=1))
