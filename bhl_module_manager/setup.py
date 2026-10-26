import os
from glob import glob

from setuptools import setup

package_name = "bhl_module_manager"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "modules"), glob("modules/*.yaml")),
        (os.path.join("share", package_name, "hosts"), glob("hosts/*.yaml")),
        (os.path.join("share", package_name, "configs"), glob("configs/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools", "numpy", "PyYAML"],
    zip_safe=True,
    maintainer="Chanwoo Jeong",
    maintainer_email="hexk0131@gmail.com",
    description="Plug-in / plug-out kinematic tree and capability manager for a modular BHL upper body",
    license="TBD",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["module_manager = bhl_module_manager.node:main"]},
)
