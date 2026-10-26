"""Compose a URDF (for robot_state_publisher) from the current assembly, plus a small URDF FK used in tests.

attach_mode
- "fixed"   : port -> module joints are fixed. robot_state_publisher sends them on /tf_static.
              tf2 never deletes static transforms, so a detached subtree stays queryable in listeners.
- "dynamic" : port -> module joints are zero-range revolute joints whose value (0) the module manager
              publishes on /joint_states. When a module is unplugged its attachment joint disappears,
              the subtree stops updating on /tf and lookups at the current time fail for it.
"""
import math
import os
import xml.etree.ElementTree as ET

import numpy as np

from . import se3

COLORS = {"torso": "0.42 0.45 0.47 1", "arm": "0.64 0.67 0.70 1", "if": "0.93 0.49 0.19 1",
          "hand": "0.85 0.83 0.79 1", "gripper": "0.23 0.25 0.27 1", "cam": "0.17 0.18 0.20 1",
          "servo": "0.20 0.22 0.24 1", "bracket": "0.73 0.75 0.78 1"}
# pan-tilt neck (photo estimates, mm): (frame, centre, size, colour); frames: base = {N}, dh1 = tilt axis after pan, dh2 = after tilt
NECK_VISUALS = [("base", (0, 0, -7.75), (20, 40, 40.5), "servo"),           # yaw servo standing in the holder
                ("dh1", (-10, 0, 0), (40, 20, 40.5), "servo"),              # tilt servo lying flat (x1 forward, y1 down, z1 = tilt axis)
                ("dh1", (-10, 11.25, 0), (44, 2.5, 36), "bracket"),         # pan bracket under the tilt servo
                ("dh2", (0, 0, 34.75), (26, 54, 2.5), "bracket"),           # U-bracket top plate
                ("dh2", (0, 25.5, 10.25), (26, 3, 46.5), "bracket"),        # U-bracket side plates
                ("dh2", (0, -25.5, 10.25), (26, 3, 46.5), "bracket")]


def _fmt3(v):
    return " ".join(f"{x:.6f}" for x in v)


def _origin(T_mm):
    r, p, y = se3.R_to_rpy(T_mm[:3, :3])
    return _fmt3(T_mm[:3, 3] / 1000.0), _fmt3((r, p, y))


def _jname(s):
    return s.replace("/", "__")


# ---------------------------------------------------------------- URDF reading
def parse_urdf(text_or_path):
    if os.path.exists(str(text_or_path)):
        root = ET.parse(text_or_path).getroot()
    else:
        root = ET.fromstring(text_or_path)
    joints = []
    for j in root.findall("joint"):
        o = j.find("origin")
        xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
        rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
        ax = j.find("axis")
        axis = [float(v) for v in ax.get("xyz").split()] if ax is not None else [1.0, 0, 0]
        joints.append({"name": j.get("name"), "type": j.get("type"), "parent": j.find("parent").get("link"),
                       "child": j.find("child").get("link"), "T": se3.H(se3.rpy_to_R(*rpy), xyz), "axis": axis})
    links = {}
    for l in root.findall("link"):
        vis = []
        for v in l.findall("visual"):
            o = v.find("origin")
            xyz = [float(x) for x in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
            rpy = [float(x) for x in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
            mesh = v.find("geometry/mesh")
            vis.append({"T": se3.H(se3.rpy_to_R(*rpy), xyz), "mesh": mesh.get("filename") if mesh is not None else None})
        links[l.get("name")] = vis
    return {"name": root.get("name"), "joints": joints, "links": links}


def urdf_fk(model, q=None):
    """Poses of every link (4x4, metres) for joint values q {name: rad}."""
    q = q or {}
    children = {}
    has_parent = set()
    for j in model["joints"]:
        children.setdefault(j["parent"], []).append(j)
        has_parent.add(j["child"])
    roots = [l for l in model["links"] if l not in has_parent]
    out = {r: np.eye(4) for r in roots}
    stack = list(roots)
    while stack:
        p = stack.pop()
        for j in children.get(p, []):
            T = out[p] @ j["T"]
            if j["type"] in ("revolute", "continuous"):
                T = T @ se3.H(se3.axis_angle(j["axis"], q.get(j["name"], 0.0)))
            elif j["type"] == "prismatic":
                T = T @ se3.H(None, np.asarray(j["axis"]) * q.get(j["name"], 0.0))
            out[j["child"]] = T
            stack.append(j["child"])
    return out


# ---------------------------------------------------------------- URDF writing
def _bhl_visuals(asm, bhl_urdf, mesh_prefix):
    """BHL link meshes re-expressed in the DH link frames: {dh link name: [(T_mm, mesh)]}."""
    model = parse_urdf(bhl_urdf)
    by_joint = {j["name"]: j for j in model["joints"]}
    T0 = urdf_fk(model)
    poses = asm.poses()
    out = {}
    for iid in asm.order:
        inst = asm.instances[iid]
        for i, j in enumerate(inst.mtype.joints_for(inst.side), start=1):
            bj = by_joint.get(j.hw)
            if bj is None:
                continue
            link = bj["child"]
            Tdh = poses[f"{iid}/dh{i}"].copy()
            Tdh[:3, 3] /= 1000.0
            for v in model["links"].get(link, []):
                if not v["mesh"]:
                    continue
                Tl = se3.inv(Tdh) @ T0[link] @ v["T"]
                Tl[:3, 3] *= 1000.0
                fn = v["mesh"]
                if mesh_prefix:
                    fn = mesh_prefix.rstrip("/") + "/" + os.path.basename(fn)
                out.setdefault(f"{iid}/dh{i}", []).append((Tl, fn))
    return out


def _primitive_visuals(asm, zed_mesh):
    out = {}
    for iid in asm.order:
        inst = asm.instances[iid]
        mt, base = inst.mtype, asm.base_frame_name(inst)
        if mt.kind == "base":
            out.setdefault(base, []).append((se3.H(None, (0, 0, 707)), ("box", (140, 250, 246)), "torso"))
            if "N" in mt.frames_for(inst.side):            # printed servo holder on the front top 2020 bar, under port N
                N = asm.local_frames(inst, {})["N"]
                out[base].append((N @ se3.H(None, (8.5, 0, -28)), ("box", (7, 124, 20)), "if"))
                out[base].append((N @ se3.H(None, (0, 0, -21)), ("box", (24, 56, 42)), "if"))
        elif mt.kind == "actuator" and mt.id.startswith("neck"):
            for frame, c, size, col in NECK_VISUALS:
                link = base if frame == "base" else f"{iid}/{frame}"
                out.setdefault(link, []).append((se3.H(None, c), ("box", size), col))
        elif mt.kind == "adapter":
            out.setdefault(base, []).append((se3.H(None, (0, 0, 12.0)), ("cylinder", (42, 24.05)), "if"))
        elif mt.kind == "end_effector":
            col = "hand" if "hand" in mt.id else "gripper"
            size = (80, 70, 300) if col == "hand" else (64, 104, 160)
            out.setdefault(base, []).append((se3.H(None, (0, 0, size[2] / 2 + 2)), ("box", size), col))
        elif mt.kind == "sensor":
            frames = mt.frames_for(inst.side)
            if "camera_link" in frames and zed_mesh:
                link = asm.global_name(inst, "camera_link")
                out.setdefault(link, []).append((se3.H(None, (-10, 0, 15)), ("mesh", zed_mesh), "cam"))
    return out


def _local_export_T(asm, inst, fname, frames):
    """Transform of an exported frame relative to its nearest exported ancestor (or base / DH frame)."""
    T = np.eye(4)
    f = frames[fname]
    while True:
        T = se3.from_xyz_rpy_deg(f.xyz, f.rpy) @ T
        par = f.parent
        if par is None:
            return asm.base_frame_name(inst), T
        if par.startswith("dh") and par[2:].isdigit():
            return (asm.base_frame_name(inst) if par == "dh0" else f"{inst.id}/{par}"), T
        pf = frames[par]
        if pf.export:
            return asm.global_name(inst, par), T
        f = pf


def to_urdf(asm, robot_name="bhl_modular", attach_mode="dynamic", bhl_urdf=None, mesh_prefix=None,
            zed_mesh="package://zed_description/meshes/zed2i.stl", visuals=True):
    links, joints = [], []
    root_inst = asm.instances[asm.base_id]
    links.append(asm.base_frame_name(root_inst))
    for iid in asm.order:
        inst = asm.instances[iid]
        mt = inst.mtype
        base = asm.base_frame_name(inst)
        if inst.parent_port is not None:
            links.append(base)
            parent = asm.port_frame_name(inst.parent_port)
            if attach_mode == "dynamic":
                joints.append({"name": f"attach__{iid}", "type": "revolute", "parent": parent, "child": base,
                               "T": np.eye(4), "axis": (0, 0, 1), "limit": (0.0, 0.0, 0.0, 0.0)})
            else:
                joints.append({"name": f"attach__{iid}", "type": "fixed", "parent": parent, "child": base, "T": np.eye(4)})
        prev = base
        for i, j in enumerate(mt.joints_for(inst.side), start=1):
            qlink, dlink = f"{iid}/q{i}", f"{iid}/dh{i}"
            links += [qlink, dlink]
            joints.append({"name": j.hw, "type": "revolute", "parent": prev, "child": qlink, "T": np.eye(4),
                           "axis": (0, 0, 1),
                           "limit": (math.radians(j.soft[0]), math.radians(j.soft[1]), mt.effort or 0.0, mt.velocity or 0.0),   # control domain
                           "note": f"{j.hw}: mechanical range {list(j.range)} deg (fb.mechanical), soft limits {list(j.soft)} deg (fb.control)"})
            Tfix = se3.dh_standard(j.a, math.radians(j.alpha), j.d, math.radians(j.theta0))
            joints.append({"name": _jname(f"{iid}/dh{i}_fixed"), "type": "fixed", "parent": qlink, "child": dlink, "T": Tfix})
            prev = dlink
        frames = mt.frames_for(inst.side)
        for fname, f in frames.items():
            if not f.export:
                continue
            par, T = _local_export_T(asm, inst, fname, frames)
            child = asm.global_name(inst, fname)
            links.append(child)
            joints.append({"name": _jname(f"{child}_fixed"), "type": "fixed", "parent": par, "child": child, "T": T})
    vis = {}
    if visuals:
        for k, v in _primitive_visuals(asm, zed_mesh).items():
            vis.setdefault(k, []).extend(v)
        if bhl_urdf:
            for k, v in _bhl_visuals(asm, bhl_urdf, mesh_prefix).items():
                vis.setdefault(k, []).extend((T, ("mesh", fn), "arm") for T, fn in v)
    lines = [f'<?xml version="1.0"?>', f'<robot name="{robot_name}">']
    for c, rgba in COLORS.items():
        lines.append(f'  <material name="{c}"><color rgba="{rgba}"/></material>')
    for l in links:
        items = vis.get(l, [])
        if not items:
            lines.append(f'  <link name="{l}"/>')
            continue
        lines.append(f'  <link name="{l}">')
        for T, (gtype, g), col in items:
            xyz, rpy = _origin(T)
            if gtype == "box":
                geom = f'<box size="{_fmt3(np.asarray(g) / 1000.0)}"/>'
            elif gtype == "cylinder":
                geom = f'<cylinder radius="{g[0] / 1000.0:.6f}" length="{g[1] / 1000.0:.6f}"/>'
            else:
                geom = f'<mesh filename="{g}"/>'
            lines.append(f'    <visual><origin xyz="{xyz}" rpy="{rpy}"/><geometry>{geom}</geometry><material name="{col}"/></visual>')
        lines.append('  </link>')
    for j in joints:
        xyz, rpy = _origin(j["T"])
        if j.get("note"):
            lines.append(f'  <!-- {j["note"]} -->')
        lines.append(f'  <joint name="{j["name"]}" type="{j["type"]}">')
        lines.append(f'    <parent link="{j["parent"]}"/><child link="{j["child"]}"/>')
        lines.append(f'    <origin xyz="{xyz}" rpy="{rpy}"/>')
        if j["type"] == "revolute":
            lo, hi, eff, vel = j["limit"]
            lines.append(f'    <axis xyz="{_fmt3(j["axis"])}"/>')
            lines.append(f'    <limit lower="{lo:.6f}" upper="{hi:.6f}" effort="{eff}" velocity="{vel}"/>')
        lines.append('  </joint>')
    lines.append('</robot>')
    return "\n".join(lines)


def attach_joint_names(asm):
    return [f"attach__{iid}" for iid in asm.order if asm.instances[iid].parent_port is not None]
