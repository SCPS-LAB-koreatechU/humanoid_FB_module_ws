"""Core tests. Run from the package root:  python3 -m pytest -q test
Set BHL_URDF to berkeley_humanoid_lite.urdf to also check the DH model against the original URDF."""
import math
import os

import numpy as np
import pytest

from bhl_module_manager import (Assembly, DescriptorError, build, evaluate, load_hosts, load_library, parse_module,
                                parse_urdf, plan_hand_over, to_urdf, urdf_fk, attach_joint_names)
from bhl_module_manager import se3

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BHL_URDF = os.environ.get("BHL_URDF", "")
LIB = load_library(os.path.join(ROOT, "modules"))
HOSTS = load_hosts(os.path.join(ROOT, "hosts", "hosts.yaml"))


def full():
    asm, _ = build(os.path.join(ROOT, "configs", "A_full.yaml"), LIB, HOSTS)
    return asm


def rot_err_deg(R):
    return math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(R) - 1) / 2))))


@pytest.mark.skipif(not os.path.exists(BHL_URDF), reason="set BHL_URDF to berkeley_humanoid_lite.urdf")
def test_dh_matches_bhl_urdf():
    model = parse_urdf(BHL_URDF)
    asm = full()
    rng = np.random.default_rng(0)
    for side, arm, hand in (("R", "r_arm", "arm_right_hand_link"), ("L", "l_arm", "arm_left_hand_link")):
        joints = asm.instances[arm].mtype.joints_for(side)
        Tb0 = urdf_fk(model)[hand].copy()
        Tb0[:3, 3] *= 1000
        X = se3.inv(Tb0) @ asm.poses()[f"{arm}/F"]
        assert np.linalg.norm(X[:3, 3]) < 0.01            # F is the URDF hand frame origin
        for _ in range(500):
            q = {j.hw: rng.uniform(math.radians(j.range[0]), math.radians(j.range[1])) for j in joints}   # whole mechanical range
            Tb = urdf_fk(model, q)[hand].copy()
            Tb[:3, 3] *= 1000
            D = se3.inv(Tb @ X) @ asm.poses(q)[f"{arm}/F"]
            assert np.linalg.norm(D[:3, 3]) < 0.01           # mm
            assert rot_err_deg(D[:3, :3]) < 1e-2            # deg (DH rows are rounded: d4 = -1 vs -0.999)


@pytest.mark.parametrize("mode", ["dynamic", "fixed"])
def test_generated_urdf_matches_assembly(mode):
    asm = full()
    model = parse_urdf(to_urdf(asm, attach_mode=mode))
    rng = np.random.default_rng(1)
    for _ in range(100):
        q = {}
        for iid in asm.order:
            inst = asm.instances[iid]
            for j in inst.mtype.joints_for(inst.side):
                q[j.hw] = rng.uniform(math.radians(j.soft[0]), math.radians(j.soft[1]))
        P = asm.poses(q)
        U = urdf_fk(model, q)
        for name, _, _ in asm.tree():
            assert name in U, name
            assert np.allclose(U[name][:3, 3] * 1000, P[name][:3, 3], atol=1e-3), name
            assert rot_err_deg(U[name][:3, :3].T @ P[name][:3, :3]) < 1e-4, name


def test_port_type_mismatch_rejected():
    asm = Assembly(LIB, HOSTS, "torso_bhl")
    r = asm.attach("torso.S_R", "hand_dexhand_v1", "h", host="control_pc")
    assert not r and any("포트 타입 불일치" in x for x in r.reasons)


def test_side_mismatch_rejected():
    asm = Assembly(LIB, HOSTS, "torso_bhl")
    r = asm.attach("torso.S_R", "arm_bhl", "a", side="L", host="control_pc")
    assert not r and any("좌우 불일치" in x for x in r.reasons)


def test_cuda_host_required_for_zed():
    asm = Assembly(LIB, HOSTS, "torso_bhl")
    assert asm.attach("torso.N", "neck_pan_tilt_scs", "neck", host="control_pc")
    r = asm.attach("neck.mount", "head_zed2i", "head", host="rdk_x5")
    assert not r and any("cuda" in x for x in r.reasons)
    assert asm.attach("neck.mount", "head_zed2i", "head", host="control_pc")


def test_camera_mounts_on_the_neck_bracket_not_on_the_holder():
    asm = Assembly(LIB, HOSTS, "torso_bhl")
    r = asm.attach("torso.N", "head_zed2i", "head", host="control_pc")
    assert not r and any("포트 타입 불일치" in x for x in r.reasons)          # head_mount vs camera_1_4_20


def Rz(a):
    c, s_ = math.cos(a), math.sin(a)
    return np.array([[c, -s_, 0], [s_, c, 0], [0, 0, 1]])


def Ry(a):
    c, s_ = math.cos(a), math.sin(a)
    return np.array([[c, 0, s_], [0, 1, 0], [-s_, 0, c]])


def about(R, p):
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = p - R @ p
    return T


def test_neck_dh_is_pan_about_the_port_z_then_tilt_about_the_carried_lateral_axis():
    asm = full()
    P0 = asm.poses()
    N = P0["torso/N"][:3, 3]
    tilt_point = N + np.array([0, 0, 28.0])
    rng = np.random.default_rng(4)
    for _ in range(200):
        qp, qt = math.radians(rng.uniform(-90, 90)), math.radians(rng.uniform(-40, 60))
        P = asm.poses({"head_pan_joint": qp, "head_tilt_joint": qt})
        want = about(Rz(qp), N) @ about(Ry(qt), tilt_point) @ P0["neck/mount"]
        assert np.allclose(P["neck/mount"], want, atol=1e-9)
        assert np.allclose(P["zed_camera_link"], P["neck/mount"], atol=1e-9)
    P = asm.poses({"head_pan_joint": math.radians(30), "head_tilt_joint": math.radians(20)})
    x = P["zed_camera_link"][:3, 0]                          # optical axis: +pan looks left (+y), +tilt looks down (-z)
    assert x[1] > 0.4 and x[2] < -0.3


def test_look_at_needs_the_camera_on_the_neck():
    asm = full()
    assert "look_at(point)" in evaluate(asm)["available"]
    assert asm.detach("head")                                # unscrew the ZED only: the neck joints stay
    caps = evaluate(asm)
    assert "look_at(point)" not in caps["available"] and "detect(object)" in caps["unavailable"]["look_at(point)"]
    assert {"head_pan_joint", "head_tilt_joint"} <= set(asm.joint_names())


def test_detach_removes_subtree_frames_and_skills():
    asm = full()
    before = evaluate(asm)
    r = asm.detach("r_arm")
    assert r and len(r.events) == 3                       # r_hand, r_conn, r_arm
    names = {n for n, _, _ in asm.tree()}
    assert not any(n.startswith(("r_arm/", "r_conn/", "r_hand/")) for n in names)
    assert ("torso.S_R", "shoulder_bhl") in asm.free_ports()
    after = evaluate(asm)
    gone = set(before["available"]) - set(after["available"])
    assert {"wave(R_arm)", "grasp(R_hand)", "hand_over(R_hand)", "handshake(R_arm, R_hand)"} <= gone


def scenario():
    asm = full()
    out = {"A": plan_hand_over(evaluate(asm))["call"]}
    asm.detach("r_hand")
    assert asm.attach("r_conn.hand", "gripper_placeholder", "r_grip")
    out["B"] = plan_hand_over(evaluate(asm))["call"]
    asm.detach("r_grip")
    assert asm.attach("r_conn.hand", "hand_dexhand_v1", "r_hand", host="control_pc")
    asm.detach("r_arm")
    out["C"] = plan_hand_over(evaluate(asm))["call"]
    assert asm.attach("torso.S_R", "arm_bhl", "r_arm", host="control_pc")
    assert asm.attach("r_arm.flange", "connector_d84", "r_conn")
    assert asm.attach("r_conn.hand", "hand_dexhand_v1", "r_hand", host="control_pc")
    asm.detach("r_hand")
    asm.detach("l_hand")
    out["D"] = plan_hand_over(evaluate(asm))["call"]
    assert asm.attach("r_conn.hand", "hand_dexhand_v1", "r_hand", host="control_pc")
    assert asm.attach("l_conn.hand", "hand_dexhand_v1", "l_hand", host="control_pc")
    r = asm.detach("neck")                                    # the neck comes off with the camera (subtree)
    assert r and len(r.events) == 2
    out["E"] = plan_hand_over(evaluate(asm))["call"]
    return out


def test_configurations_A_to_E():
    assert scenario() == {"A": "hand_over(R_hand)", "B": "hand_over(R_gripper)", "C": "hand_over(L_hand)",
                          "D": "reject(no_grasp)", "E": "reject(no_vision)"}


def test_new_module_needs_only_a_descriptor():
    suction = parse_module({
        "id": "suction_cup", "name": "흡착 그리퍼", "kind": "end_effector", "sides": ["R", "L"],
        "fb": {"mechanical": {"mates_with": "hand_d84", "frames": {"tcp": {"xyz": [0, 0, 95]}}},
               "functional": {"skills": [
                   {"name": "pick({side}_suction)", "requires": {"frames": ["tcp"]}},
                   {"name": "hand_over({side}_suction)", "requires": {"frames": ["tcp"], "capabilities": ["locate_hand(person)"]}}]}}})
    lib = dict(LIB, suction_cup=suction)
    asm, _ = build(os.path.join(ROOT, "configs", "A_full.yaml"), lib, HOSTS)
    asm.detach("r_hand")
    assert asm.attach("r_conn.hand", "suction_cup", "r_suck")
    caps = evaluate(asm)
    assert "pick(R_suction)" in caps["available"] and "hand_over(R_suction)" in caps["available"]
    assert "r_suck/tcp" in asm.poses()
    assert "attach__r_suck" in to_urdf(asm)


def test_attach_joints_are_zero_range_revolute_in_dynamic_mode():
    asm = full()
    text = to_urdf(asm, attach_mode="dynamic")
    model = parse_urdf(text)
    att = {j["name"]: j for j in model["joints"] if j["name"].startswith("attach__")}
    assert set(att) == set(attach_joint_names(asm))
    assert all(j["type"] == "revolute" for j in att.values())
    assert 'lower="0.000000" upper="0.000000"' in text


def test_descriptor_errors():
    with pytest.raises(DescriptorError):
        parse_module({"id": "x", "kind": "end_effector", "fb": {"mechanical": {}}})          # no mates_with
    with pytest.raises(DescriptorError):
        parse_module({"id": "x", "kind": "adapter", "fb": {"mechanical": {"mates_with": "hand_d84",
                      "ports": [{"name": "p", "type": "t", "frame": "nowhere"}]}}})
    with pytest.raises(DescriptorError):
        parse_module({"id": "x", "kind": "adapter", "fb": {"mechanical": {"mates_with": "m", "frames": {"F": {True: "dh5"}}}}})


@pytest.mark.skipif(not os.path.exists(BHL_URDF), reason="set BHL_URDF to berkeley_humanoid_lite.urdf")
def test_bhl_meshes_land_where_the_original_urdf_puts_them():
    asm = full()
    gen = parse_urdf(to_urdf(asm, bhl_urdf=BHL_URDF))
    org = parse_urdf(BHL_URDF)
    rng = np.random.default_rng(2)
    for _ in range(50):
        q = {}
        for iid in asm.order:
            inst = asm.instances[iid]
            for j in inst.mtype.joints_for(inst.side):
                q[j.hw] = rng.uniform(math.radians(j.soft[0]), math.radians(j.soft[1]))
        G, O = urdf_fk(gen, q), urdf_fk(org, q)
        org_vis = {os.path.basename(v["mesh"]): O[l] @ v["T"] for l, vs in org["links"].items() for v in vs if v["mesh"]}
        n = 0
        for l, vs in gen["links"].items():
            for v in vs:
                if v["mesh"] and "zed2i" not in v["mesh"]:
                    Tg = G[l] @ v["T"]
                    To = org_vis[os.path.basename(v["mesh"])]
                    assert np.allclose(Tg[:3, 3], To[:3, 3], atol=1e-5) and rot_err_deg(Tg[:3, :3].T @ To[:3, :3]) < 1e-2
                    n += 1
        assert n == 10                                       # 5 links per arm


def test_joint_limits_are_split_between_mechanical_and_control():
    neck = LIB["neck_pan_tilt_scs"]
    pan, tilt = neck.joints_for(None)
    assert pan.range == (-100.0, 100.0) and tilt.range == (-70.0, 100.0)          # fb.mechanical (structure)
    assert pan.soft == (-90.0, 90.0) and tilt.soft == (-40.0, 60.0)                # fb.control (software)
    arm = LIB["arm_bhl"].joints_for("R")[0]
    assert arm.soft == arm.range                                                   # no soft limits -> mechanical range
    text = to_urdf(full())
    model = {j["name"]: j for j in parse_urdf(text)["joints"]}
    assert "head_tilt_joint" in model
    assert 'name="head_tilt_joint"' in text and f'lower="{math.radians(-40):.6f}" upper="{math.radians(60):.6f}" effort="1.5" velocity="6.5"' in text
    base = {"id": "x", "kind": "actuator", "fb": {"mechanical": {"mates_with": "m", "kinematics": {"type": "dh_standard", "joints": [
        {"hw": "j1", "a": 0, "alpha": 0, "d": 0, "theta0": 0, "range": [-30, 30]}]}}}}
    import copy
    bad = copy.deepcopy(base); bad["fb"]["control"] = {"soft_limits": {"j1": [-40, 10]}}          # soft outside the mechanical range
    with pytest.raises(DescriptorError):
        parse_module(bad)
    bad = copy.deepcopy(base); bad["fb"]["mechanical"]["kinematics"]["joints"][0]["limit"] = [-30, 30]   # old mixed key
    with pytest.raises(DescriptorError):
        parse_module(bad)
    bad = copy.deepcopy(base); bad["fb"]["mechanical"]["kinematics"]["effort"] = 2.0              # actuator value in mechanical
    with pytest.raises(DescriptorError):
        parse_module(bad)
    ok = copy.deepcopy(base); ok["fb"]["control"] = {"soft_limits": {"j1": [-20, 20]}, "effort": 2.0}
    j = parse_module(ok).joints_for(None)[0]
    assert j.range == (-30.0, 30.0) and j.soft == (-20.0, 20.0)
