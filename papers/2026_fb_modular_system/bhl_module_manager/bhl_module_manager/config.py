"""Build an assembly from a configuration file (base module + list of attachments)."""
import os

import yaml

from .assembly import Assembly
from .descriptor import load_hosts, load_library

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def default_paths():
    """Source tree layout, or the installed share directory of the ROS 2 package."""
    if os.path.isdir(os.path.join(PKG, "modules")):
        return os.path.join(PKG, "modules"), os.path.join(PKG, "hosts", "hosts.yaml"), os.path.join(PKG, "configs")
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory("bhl_module_manager")
        return os.path.join(share, "modules"), os.path.join(share, "hosts", "hosts.yaml"), os.path.join(share, "configs")
    except Exception:  # noqa: BLE001
        raise FileNotFoundError("module descriptors not found")


def build(config_path, library=None, hosts=None, module_dirs=None, hosts_path=None):
    if library is None or hosts is None:
        mdir, hpath, _ = default_paths()
        library = library or load_library(*(module_dirs or [mdir]))
        hosts = hosts or load_hosts(hosts_path or hpath)
    with open(config_path, encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    asm = Assembly(library, hosts, cfg["base"]["module"], cfg["base"].get("id", "torso"))
    results = []
    for a in cfg.get("attach") or []:
        r = asm.attach(a["port"], a["module"], a.get("id"), a.get("side"), a.get("host"))
        results.append((a, r))
        if not r:
            raise ValueError(f"config {config_path}: {a} rejected: {r.reasons}")
    return asm, results
