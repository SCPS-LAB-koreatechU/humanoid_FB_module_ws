"""Plug-in / plug-out kinematic and capability manager for modular humanoids (BHL upper body).

Pure Python core (numpy, PyYAML). The ROS 2 node lives in node.py and is optional.
"""
from .assembly import Assembly, Result  # noqa: F401
from .capability import diff, evaluate, plan_hand_over  # noqa: F401
from .config import build, default_paths  # noqa: F401
from .descriptor import DescriptorError, load_hosts, load_library, load_module, parse_module  # noqa: F401
from .urdf_gen import attach_joint_names, parse_urdf, to_urdf, urdf_fk  # noqa: F401
