"""把 src/tl_vision 注册成标准 Python 包, 供 catkin_python_setup() 安装。"""
from setuptools import setup

setup(
    name="tl_vision",
    version="1.0.0",
    packages=["tl_vision"],
    package_dir={"": "src"},
    zip_safe=True,
)
