"""Capability list derived from the attached modules (functional domain).

A skill is available when its module is attached (so its frames are connected to the base),
the module's host has the required flags, and every required capability is itself available.
"""


def _fmt(text, inst):
    side = inst.side or ""
    return text.replace("{side}", side)


def evaluate(asm):
    cands = {}
    for iid in asm.order:
        inst = asm.instances[iid]
        frames = inst.mtype.frames_for(inst.side)
        host = asm.hosts.get(inst.host, {}) if inst.host else {}
        for s in inst.mtype.skills:
            name = _fmt(s["name"], inst)
            req = s["requires"]
            missing = [f for f in req["frames"] if f not in frames]
            host_missing = [h for h in req["host"] if not host.get(h)]
            cands[name] = {"provider": iid, "caps": [_fmt(c, inst) for c in req["capabilities"]],
                           "static_fail": ([f"프레임 없음 {missing}"] if missing else []) +
                                          ([f"호스트 {inst.host or '-'}에 {host_missing} 없음"] if host_missing else [])}
    available = {n for n, c in cands.items() if not c["static_fail"] and not c["caps"]}
    changed = True
    while changed:                       # fixed point over capability requirements
        changed = False
        for n, c in cands.items():
            if n in available or c["static_fail"]:
                continue
            if all(x in available for x in c["caps"]):
                available.add(n)
                changed = True
    unavailable = {}
    for n, c in cands.items():
        if n in available:
            continue
        why = list(c["static_fail"]) + [f"{x} 없음" for x in c["caps"] if x not in available]
        unavailable[n] = ", ".join(why)
    return {"available": sorted(available), "unavailable": unavailable,
            "providers": {n: c["provider"] for n, c in cands.items()}}


HAND_OVER_ORDER = ["hand_over(R_hand)", "hand_over(R_gripper)", "hand_over(L_hand)", "hand_over(L_gripper)"]


def plan_hand_over(caps):
    """'빼빼로 건네줘' -> first available hand_over, else a reason (no_grasp / no_vision)."""
    avail = set(caps["available"])
    for name in HAND_OVER_ORDER:
        if name in avail:
            return {"ok": True, "call": name}
    effectors = [n for n in avail if n.startswith("grasp(")]
    if not effectors:
        return {"ok": False, "call": "reject(no_grasp)", "reason": "전달에 쓸 손이나 그리퍼가 없음"}
    if "locate_hand(person)" not in avail:
        return {"ok": False, "call": "reject(no_vision)", "reason": "받는 사람 손 위치를 알 수 없음 (카메라 모듈 없음)"}
    return {"ok": False, "call": "reject(unknown)", "reason": "hand_over 조건 미충족"}


def diff(before, after):
    a, b = set(before["available"]), set(after["available"])
    return sorted(b - a), sorted(a - b)
