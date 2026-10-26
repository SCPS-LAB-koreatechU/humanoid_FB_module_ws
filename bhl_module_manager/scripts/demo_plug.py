#!/usr/bin/env python3
"""Plug-in / plug-out scenario A -> E without ROS.

For every step it prints the validation result, the TF tree change and the capability change,
plans "빼빼로 건네줘", and writes the URDF that robot_state_publisher would receive.

  python3 scripts/demo_plug.py --out out [--bhl-urdf path/to/berkeley_humanoid_lite.urdf --mesh-prefix package://...]
"""
import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from bhl_module_manager import build, diff, evaluate, plan_hand_over, to_urdf  # noqa: E402
from bhl_module_manager.config import default_paths  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="out")
    ap.add_argument("--bhl-urdf", default=None, help="original BHL URDF, used for arm visual meshes")
    ap.add_argument("--mesh-prefix", default=None, help="replaces the directory of BHL mesh filenames")
    ap.add_argument("--attach-mode", default="dynamic", choices=["dynamic", "fixed"])
    args = ap.parse_args()
    os.makedirs(os.path.join(args.out, "urdf"), exist_ok=True)
    _, _, cfg_dir = default_paths()
    asm, _ = build(os.path.join(cfg_dir, "A_full.yaml"))
    log = []
    caps = evaluate(asm)

    def tree_names():
        return {n for n, _, _ in asm.tree()}

    def step(title, actions, save=None):
        nonlocal caps
        print(f"\n=== {title}")
        before_frames = tree_names()
        rec = {"step": title, "actions": []}
        for op, *a in actions:
            r = asm.attach(*a) if op == "attach" else asm.detach(*a)
            tag = "OK " if r.ok else "거부"
            msg = r.events if r.ok else r.reasons
            print(f"  [{tag}] {op} {a[0]}{(' ← ' + a[1]) if op == 'attach' else ''}: {'; '.join(msg)}")
            rec["actions"].append({"op": op, "args": a, "ok": r.ok, "msg": msg})
        new_caps = evaluate(asm)
        added, removed = diff(caps, new_caps)
        caps = new_caps
        after_frames = tree_names()
        plan = plan_hand_over(caps)
        print(f"  TF 프레임 +{len(after_frames - before_frames)} -{len(before_frames - after_frames)}"
              f" (현재 {len(after_frames)}개), 관절 {len(asm.joint_names())}개")
        if added:
            print(f"  능력 추가: {', '.join(added)}")
        if removed:
            print(f"  능력 제거: {', '.join(removed)}")
        print(f"  \"빼빼로 건네줘\" → {plan['call']}" + (f" ({plan['reason']})" if not plan["ok"] else ""))
        rec.update({"frames": sorted(after_frames), "capabilities": caps["available"], "added": added,
                    "removed": removed, "plan": plan, "instances": asm.snapshot()["instances"]})
        if save:
            path = os.path.join(args.out, "urdf", f"{save}.urdf")
            with open(path, "w", encoding="utf-8") as f:
                f.write(to_urdf(asm, attach_mode=args.attach_mode, bhl_urdf=args.bhl_urdf, mesh_prefix=args.mesh_prefix))
            rec["urdf"] = path
            print(f"  URDF 저장: {path}")
        log.append(rec)

    step("A 기본 구성 (설정 파일로 전체 장착)", [], save="A")
    step("B 손 교체: 오른손 분리 후 그리퍼 장착", [("detach", "r_hand"),
                                              ("attach", "r_conn.hand", "gripper_placeholder", "r_grip")], save="B")
    step("A로 복귀: 그리퍼 분리, 오른손 재장착", [("detach", "r_grip"),
                                            ("attach", "r_conn.hand", "hand_dexhand_v1", "r_hand", None, "control_pc")])
    step("C 오른팔 분리 (손, 커넥터 포함 하위 트리 제거)", [("detach", "r_arm")], save="C")
    step("A로 복귀: 오른팔, 커넥터, 손 재장착", [("attach", "torso.S_R", "arm_bhl", "r_arm", None, "control_pc"),
                                           ("attach", "r_arm.flange", "connector_d84", "r_conn"),
                                           ("attach", "r_conn.hand", "hand_dexhand_v1", "r_hand", None, "control_pc")])
    step("D 양손 분리", [("detach", "r_hand"), ("detach", "l_hand")], save="D")
    step("A로 복귀: 양손 재장착", [("attach", "r_conn.hand", "hand_dexhand_v1", "r_hand", None, "control_pc"),
                               ("attach", "l_conn.hand", "hand_dexhand_v1", "l_hand", None, "control_pc")])
    step("E 머리 분리 (목 모듈을 ZED 2i째로 분리)", [("detach", "neck")], save="E")
    step("검증 사례: 잘못된 장착 시도 후 머리 복귀", [
        ("attach", "torso.N", "head_zed2i", "head", None, "control_pc"),         # 포트 타입 불일치: 카메라는 목 브래킷(1/4"-20)에 결합
        ("attach", "torso.N", "neck_pan_tilt_scs", "neck", None, "control_pc"),  # 목 모듈 장착
        ("attach", "neck.mount", "head_zed2i", "head", None, "rdk_x5"),          # CUDA 없음
        ("attach", "torso.S_L", "hand_dexhand_v1", "x_hand", None, "control_pc"),  # 포트 사용 중 + 타입 불일치
        ("attach", "neck.mount", "head_zed2i", "head", None, "control_pc")])     # 올바른 호스트
    step("카메라만 분리 (목 관절은 남음)", [("detach", "head")])
    with open(os.path.join(args.out, "plug_log.json"), "w", encoding="utf-8") as f:
        json.dump(log, f, ensure_ascii=False, indent=1)
    print(f"\n로그: {os.path.join(args.out, 'plug_log.json')}")


if __name__ == "__main__":
    main()
