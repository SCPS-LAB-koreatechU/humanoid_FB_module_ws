"""Module descriptors (Function Block interface with four MLMD-style domains).

A descriptor is one YAML file per module type. Rule for the split: what the structure fixes is mechanical,
what the actuator or the software sets is control.
  fb.mechanical  : mating port type, mass, kinematics (standard DH a, alpha, d, theta0 and the mechanical
                   joint range = hard stops or interference), named frames, output ports
  fb.electrical  : bus, ids, power, host_requirements (e.g. cuda, usb3, can)
  fb.control     : rate_hz, effort, velocity, soft_limits {joint: [lo, hi]} inside the mechanical range,
                   zero_offset {joint: actuator reading at q = 0} (calibration)
  fb.functional  : skills and their requirements (frames, other capabilities, host flags)
Adding a module = adding a file. Nothing in the core needs to change.
"""
import glob
import os
from dataclasses import dataclass, field

import yaml


class DescriptorError(Exception):
    pass


KINDS = {"base", "arm", "actuator", "adapter", "end_effector", "sensor", "tool"}


@dataclass
class JointDef:
    hw: str
    a: float
    alpha: float
    d: float
    theta0: float
    range: tuple                # mechanical range, deg (fb.mechanical)
    soft: tuple = None          # soft limits, deg (fb.control), defaults to the mechanical range


@dataclass
class FrameDef:
    name: str
    parent: str = None          # None = module base frame, "dhN" = DH frame N, or another frame name
    xyz: tuple = (0.0, 0.0, 0.0)  # mm
    rpy: tuple = (0.0, 0.0, 0.0)  # deg
    tf_name: str = None         # exported TF name override (global, unprefixed)
    export: bool = True         # write to URDF/TF


@dataclass
class PortDef:
    name: str
    type: str
    frame: str
    side: str = None


@dataclass
class ModuleType:
    id: str
    name: str
    kind: str
    sides: list
    mates_with: str
    mass_kg: float
    kin_type: str
    joints: dict = field(default_factory=dict)        # side -> [JointDef]
    effort: float = None                              # fb.control
    velocity: float = None                            # fb.control
    frames: dict = field(default_factory=dict)        # side -> {name: FrameDef}
    ports: list = field(default_factory=list)
    electrical: dict = field(default_factory=dict)
    control: dict = field(default_factory=dict)
    skills: list = field(default_factory=list)
    root_frame: str = None
    source: str = None

    def joints_for(self, side):
        if self.kin_type != "dh_standard":
            return []
        return self.joints.get(side) or self.joints.get(None) or []

    def frames_for(self, side):
        return self.frames.get(side) or self.frames.get(None) or {}

    @property
    def host_requirements(self):
        return dict(self.electrical.get("host_requirements") or {})


def _side_split(value, sides):
    """Accept either a plain value or {R: ..., L: ...}."""
    if isinstance(value, dict) and sides and set(value.keys()) <= set(sides) and value:
        return {k: value[k] for k in value}
    return {None: value}


def _frame(name, spec):
    if spec is None:
        spec = {}
    if True in spec or "on" in spec:            # YAML 1.1 reads a bare `on:` key as True
        raise DescriptorError(f"frame {name}: use `at: dhN` (a bare `on:` key is read as a boolean by YAML)")
    if "at" in spec:
        return FrameDef(name=name, parent=spec["at"], tf_name=spec.get("tf_name"), export=spec.get("export", True))
    xyz = tuple(float(v) for v in spec.get("xyz", (0, 0, 0)))
    rpy = tuple(float(v) for v in spec.get("rpy", (0, 0, 0)))
    if len(xyz) != 3 or len(rpy) != 3:
        raise DescriptorError(f"frame {name}: xyz and rpy need 3 values")
    return FrameDef(name=name, parent=spec.get("parent"), xyz=xyz, rpy=rpy, tf_name=spec.get("tf_name"),
                    export=spec.get("export", True))


def parse_module(data, source=None):
    where = source or data.get("id", "?")
    for key in ("id", "kind", "fb"):
        if key not in data:
            raise DescriptorError(f"{where}: missing '{key}'")
    kind = data["kind"]
    if kind not in KINDS:
        raise DescriptorError(f"{where}: unknown kind '{kind}' (allowed: {sorted(KINDS)})")
    fb = data["fb"]
    mech = fb.get("mechanical") or {}
    sides = list(data.get("sides") or [])
    mates = mech.get("mates_with")
    if kind != "base" and not mates:
        raise DescriptorError(f"{where}: non-base modules need fb.mechanical.mates_with")
    kin = mech.get("kinematics") or {}
    kin_type = kin.get("type")
    joints = {}
    if kin_type == "dh_standard":
        if kin.get("length_unit", "mm") != "mm" or kin.get("angle_unit", "deg") != "deg":
            raise DescriptorError(f"{where}: DH rows must be in mm and deg")
        for k in ("effort", "velocity"):
            if k in kin:
                raise DescriptorError(f"{where}: '{k}' belongs to fb.control (set by the actuator, not by the structure)")
        raw = kin.get("joints") or {}
        per_side = raw if isinstance(raw, dict) else {None: raw}
        for side, rows in per_side.items():
            out = []
            for i, r in enumerate(rows):
                if "limit" in r:
                    raise DescriptorError(f"{where}: joint {r.get('hw', i + 1)}: use 'range' for the mechanical range here "
                                          "and fb.control.soft_limits for software limits")
                for k in ("hw", "a", "alpha", "d", "theta0", "range"):
                    if k not in r:
                        raise DescriptorError(f"{where}: joint {i + 1} ({side}) missing '{k}'")
                rng = tuple(float(v) for v in r["range"])
                if len(rng) != 2 or rng[0] > rng[1]:
                    raise DescriptorError(f"{where}: joint {r['hw']} bad range {r['range']}")
                out.append(JointDef(r["hw"], float(r["a"]), float(r["alpha"]), float(r["d"]), float(r["theta0"]), rng, rng))
            joints[side] = out
        if sides and set(joints) != set(sides):
            raise DescriptorError(f"{where}: DH joints must be given for sides {sides}")
    elif kin_type not in (None, "fixed"):
        raise DescriptorError(f"{where}: unsupported kinematics type '{kin_type}'")
    # control domain: soft limits must sit inside the mechanical range of the same joint
    ctrl = fb.get("control") or {}
    by_hw = {j.hw: j for rows in joints.values() for j in rows}
    for hw, lim in (ctrl.get("soft_limits") or {}).items():
        if hw not in by_hw:
            raise DescriptorError(f"{where}: soft_limits for unknown joint {hw}")
        lo, hi = (float(v) for v in lim)
        j = by_hw[hw]
        if lo > hi or lo < j.range[0] - 1e-9 or hi > j.range[1] + 1e-9:
            raise DescriptorError(f"{where}: soft limit {hw} {list(lim)} is outside the mechanical range {list(j.range)}")
        j.soft = (lo, hi)
    for hw in (ctrl.get("zero_offset") or {}):
        if hw not in by_hw:
            raise DescriptorError(f"{where}: zero_offset for unknown joint {hw}")
    frames = {}
    for name, spec in (mech.get("frames") or {}).items():
        for side, s in _side_split(spec, sides).items():
            frames.setdefault(side, {})[name] = _frame(name, s)
    # side-independent frames are shared by every side
    if None in frames and sides:
        for side in sides:
            merged = dict(frames[None])
            merged.update(frames.get(side, {}))
            frames[side] = merged
    ports = [PortDef(p["name"], p["type"], p["frame"], p.get("side")) for p in (mech.get("ports") or [])]
    n_dh = len(next(iter(joints.values()))) if joints else 0
    for side_frames in frames.values():
        for f in side_frames.values():
            if f.parent and f.parent.startswith("dh"):
                if int(f.parent[2:]) > n_dh:
                    raise DescriptorError(f"{where}: frame {f.name} refers to {f.parent} but the chain has {n_dh} joints")
            elif f.parent and f.parent not in side_frames:
                raise DescriptorError(f"{where}: frame {f.name} has unknown parent {f.parent}")
    any_frames = frames.get(sides[0] if sides else None, frames.get(None, {}))
    for p in ports:
        if p.frame not in any_frames:
            raise DescriptorError(f"{where}: port {p.name} uses unknown frame {p.frame}")
    skills = []
    for s in (fb.get("functional") or {}).get("skills") or []:
        if "name" not in s:
            raise DescriptorError(f"{where}: skill without name")
        req = s.get("requires") or {}
        for fr in req.get("frames") or []:
            if fr not in any_frames:
                raise DescriptorError(f"{where}: skill {s['name']} needs unknown frame {fr}")
        skills.append({"name": s["name"], "requires": {"frames": list(req.get("frames") or []),
                                                        "capabilities": list(req.get("capabilities") or []),
                                                        "host": list(req.get("host") or [])}})
    return ModuleType(id=data["id"], name=data.get("name", data["id"]), kind=kind, sides=sides, mates_with=mates,
                      mass_kg=mech.get("mass_kg"), kin_type=kin_type, joints=joints, effort=ctrl.get("effort"),
                      velocity=ctrl.get("velocity"), frames=frames, ports=ports,
                      electrical=fb.get("electrical") or {}, control=fb.get("control") or {}, skills=skills,
                      root_frame=mech.get("root_frame"), source=source)


def load_module(path):
    with open(path, encoding="utf-8") as f:
        return parse_module(yaml.safe_load(f), source=os.path.basename(path))


def load_library(*dirs):
    lib = {}
    for d in dirs:
        for path in sorted(glob.glob(os.path.join(d, "*.yaml"))):
            m = load_module(path)
            if m.id in lib:
                raise DescriptorError(f"duplicate module id {m.id} ({path})")
            lib[m.id] = m
    return lib


def load_hosts(path):
    with open(path, encoding="utf-8") as f:
        return (yaml.safe_load(f) or {}).get("hosts", {})
