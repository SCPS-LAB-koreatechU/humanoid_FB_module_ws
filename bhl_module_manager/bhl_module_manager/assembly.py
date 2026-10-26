"""Configuration tree of attached modules: plug-in / plug-out with validation, and forward kinematics.

Conventions
- Lengths in mm, angles in deg in descriptors; joint values q in rad (as in /joint_states).
- Every port frame's z axis is the mating direction. When a module is attached, its base frame
  coincides with the port frame (identity attachment), so the configuration is a kinematic tree.
- Global frame names are "<instance>/<frame>"; the base module's root frame keeps its own name (e.g. "base").
"""
import math
from dataclasses import dataclass, field

import numpy as np

from . import se3


@dataclass
class Result:
    ok: bool
    reasons: list = field(default_factory=list)
    events: list = field(default_factory=list)

    def __bool__(self):
        return self.ok


@dataclass
class Instance:
    id: str
    mtype: object
    side: str = None
    host: str = None
    parent_port: str = None            # "inst.port" or None for the base
    children: dict = field(default_factory=dict)   # port name -> child instance id

    def label(self):
        return f"{self.id} ({self.mtype.name}{', ' + self.side if self.side else ''})"


class Assembly:
    def __init__(self, library, hosts, base_type, base_id="torso"):
        if base_type not in library:
            raise KeyError(f"unknown base module {base_type}")
        mt = library[base_type]
        if mt.kind != "base":
            raise ValueError(f"{base_type} is not a base module")
        self.library = library
        self.hosts = hosts
        self.instances = {base_id: Instance(base_id, mt)}
        self.base_id = base_id
        self.order = [base_id]

    # ------------------------------------------------------------------ queries
    def port(self, ref):
        inst_id, _, pname = ref.partition(".")
        inst = self.instances.get(inst_id)
        if inst is None:
            return None, None
        for p in inst.mtype.ports:
            if p.name == pname:
                return inst, p
        return inst, None

    def free_ports(self):
        out = []
        for iid in self.order:
            inst = self.instances[iid]
            for p in inst.mtype.ports:
                if p.name not in inst.children:
                    out.append((f"{iid}.{p.name}", p.type))
        return out

    def subtree(self, inst_id):
        out = [inst_id]
        for child in self.instances[inst_id].children.values():
            out += self.subtree(child)
        return out

    def global_name(self, inst, frame_name):
        f = inst.mtype.frames_for(inst.side).get(frame_name)
        if f is not None and f.tf_name:
            return f.tf_name
        if inst.id == self.base_id and frame_name == inst.mtype.root_frame:
            return frame_name
        return f"{inst.id}/{frame_name}"

    def base_frame_name(self, inst):
        if inst.id == self.base_id:
            return inst.mtype.root_frame or f"{inst.id}/base"
        return f"{inst.id}/base"

    def port_frame_name(self, ref):
        inst, p = self.port(ref)
        return self.global_name(inst, p.frame)

    # ------------------------------------------------------------- plug in/out
    def attach(self, port_ref, module_id, inst_id=None, side=None, host=None):
        reasons = []
        mt = self.library.get(module_id)
        if mt is None:
            return Result(False, [f"모듈 명세 없음: {module_id}"])
        inst_id = inst_id or f"{module_id}_{len(self.order)}"
        if inst_id in self.instances:
            reasons.append(f"인스턴스 이름 중복: {inst_id}")
        parent, port = self.port(port_ref)
        if parent is None:
            return Result(False, reasons + [f"포트를 가진 모듈이 장착되어 있지 않음: {port_ref}"])
        if port is None:
            return Result(False, reasons + [f"포트 없음: {port_ref}"])
        if port.name in parent.children:
            reasons.append(f"포트 사용 중: {port_ref} ← {parent.children[port.name]}")
        # mechanical: port type
        if mt.mates_with != port.type:
            reasons.append(f"포트 타입 불일치: {port_ref}는 {port.type}, {module_id}는 {mt.mates_with}에 결합")
        # side: from the port, else inherited from the parent chain
        want = port.side or parent.side
        if mt.sides:
            side = side or want
            if side not in mt.sides:
                reasons.append(f"{module_id}에 {side} 쪽 명세가 없음 (가능: {mt.sides})")
            elif want and side != want:
                reasons.append(f"좌우 불일치: 포트는 {want}, 모듈은 {side}")
        else:
            side = None
        # electrical: host requirements
        req = mt.host_requirements
        if req:
            if host is None:
                reasons.append(f"{module_id}는 호스트 지정이 필요함 (요구: {req})")
            elif host not in self.hosts:
                reasons.append(f"알 수 없는 호스트: {host}")
            else:
                caps = self.hosts[host]
                for flag, needed in req.items():
                    have = caps.get(flag)
                    if needed and have is None:
                        reasons.append(f"호스트 {host}의 {flag} 지원 여부 미확인")
                    elif needed and not have:
                        reasons.append(f"호스트 {host}에 {flag} 없음 ({module_id} 요구)")
        # electrical: id conflicts on the same host and bus
        ids = mt.electrical.get("ids")
        if ids:
            for other in self.instances.values():
                oids = other.mtype.electrical.get("ids")
                if oids and other.host == host and other.mtype.electrical.get("bus") == mt.electrical.get("bus"):
                    clash = sorted(set(ids) & set(oids))
                    if clash:
                        reasons.append(f"{mt.electrical.get('bus')} ID 충돌 {clash}: {other.id}")
        # TF names must stay unique
        new = Instance(inst_id, mt, side, host, port_ref)
        taken = {n for n, _, _ in self.tree()}
        for fname, f in mt.frames_for(side).items():
            g = self.global_name(new, fname)
            if f.export and g in taken:
                reasons.append(f"TF 이름 중복: {g}")
        # mechanical: payload (only when both numbers are known)
        payload = getattr(port, "payload_kg", None)
        if payload is not None and mt.mass_kg is not None and mt.mass_kg > payload:
            reasons.append(f"질량 초과: {mt.mass_kg} kg > 포트 한도 {payload} kg")
        if reasons:
            return Result(False, reasons)
        self.instances[inst_id] = new
        parent.children[port.name] = inst_id
        self.order.append(inst_id)
        return Result(True, [], [f"장착: {new.label()} → {port_ref} ({self.port_frame_name(port_ref)})"])

    def detach(self, inst_id):
        if inst_id not in self.instances:
            return Result(False, [f"장착되어 있지 않음: {inst_id}"])
        if inst_id == self.base_id:
            return Result(False, ["기준 모듈은 분리할 수 없음"])
        pid, _, pname = self.instances[inst_id].parent_port.partition(".")
        removed = self.subtree(inst_id)
        events = []
        for iid in reversed(removed):           # leaves first
            inst = self.instances.pop(iid)
            self.order.remove(iid)
            events.append(f"분리: {inst.label()}")
        del self.instances[pid].children[pname]
        return Result(True, [], events)

    # ------------------------------------------------------------- kinematics
    def joint_names(self):
        out = []
        for iid in self.order:
            inst = self.instances[iid]
            out += [j.hw for j in inst.mtype.joints_for(inst.side)]
        return out

    def poses(self, q=None):
        """World poses (4x4, mm) of every frame: {global name: T}. q: {hw joint name: rad}."""
        q = q or {}
        out = {}
        for iid in self.order:
            inst = self.instances[iid]
            if inst.parent_port is None:
                Tb = np.eye(4)
            else:
                Tb = out[self.port_frame_name(inst.parent_port)]
            out[self.base_frame_name(inst)] = Tb
            local = self.local_frames(inst, q)
            for name, T in local.items():
                out[self.global_name(inst, name)] = Tb @ T
        return out

    def local_frames(self, inst, q):
        """Frames of one module relative to its base frame (mm)."""
        mt = inst.mtype
        dh = [np.eye(4)]
        for j in mt.joints_for(inst.side):
            th = math.radians(j.theta0) + q.get(j.hw, 0.0)
            dh.append(dh[-1] @ se3.dh_standard(j.a, math.radians(j.alpha), j.d, th))
        frames = mt.frames_for(inst.side)
        res = {f"dh{i}": T for i, T in enumerate(dh) if i > 0}

        def resolve(name, depth=0):
            if name in res:
                return res[name]
            if depth > 32:
                raise ValueError(f"frame cycle at {name}")
            f = frames[name]
            if f.parent is None:
                parent = np.eye(4)
            elif f.parent.startswith("dh") and f.parent[2:].isdigit():
                parent = dh[int(f.parent[2:])]
            else:
                parent = resolve(f.parent, depth + 1)
            res[name] = parent @ se3.from_xyz_rpy_deg(f.xyz, f.rpy)
            return res[name]

        for name in frames:
            resolve(name)
        return res

    def tree(self):
        """(frame, parent frame, kind) for every exported frame, attachment edges included."""
        out = []
        for iid in self.order:
            inst = self.instances[iid]
            base = self.base_frame_name(inst)
            if inst.parent_port is None:
                out.append((base, None, "root"))
            else:
                out.append((base, self.port_frame_name(inst.parent_port), "attach"))
            prev = base
            for i, j in enumerate(inst.mtype.joints_for(inst.side), start=1):
                name = f"{iid}/dh{i}"
                out.append((name, prev, f"revolute:{j.hw}"))
                prev = name
            frames = inst.mtype.frames_for(inst.side)
            for fname, f in frames.items():
                if not f.export:
                    continue
                if f.parent is None:
                    par = base
                elif f.parent.startswith("dh") and f.parent[2:].isdigit():
                    par = base if f.parent == "dh0" else f"{iid}/{f.parent}"
                else:
                    par = self.global_name(inst, f.parent)
                out.append((self.global_name(inst, fname), par, "fixed"))
        return out

    def snapshot(self):
        return {
            "base": self.base_id,
            "instances": [{"id": i, "module": self.instances[i].mtype.id, "side": self.instances[i].side,
                           "host": self.instances[i].host, "port": self.instances[i].parent_port}
                          for i in self.order],
            "free_ports": [{"port": p, "type": t} for p, t in self.free_ports()],
            "joints": self.joint_names(),
            "joint_limits": {j.hw: {"range_deg": list(j.range), "soft_deg": list(j.soft),
                                    "effort": self.instances[iid].mtype.effort, "velocity": self.instances[iid].mtype.velocity,
                                    "zero_offset": (self.instances[iid].mtype.control.get("zero_offset") or {}).get(j.hw)}
                             for iid in self.order for j in self.instances[iid].mtype.joints_for(self.instances[iid].side)},
        }
