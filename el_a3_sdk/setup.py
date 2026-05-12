from setuptools import setup, find_packages

setup(
    name="el_a3_sdk",
    version="1.0.0",
    description="EL-A3 7-DOF Robotic Arm Pure Python SDK (Direct CAN, multi-arm)",
    author="EL-A3 Team",
    packages=find_packages(),
    python_requires=">=3.8",
    install_requires=[
        "numpy",
        "pyyaml",
    ],
    extras_require={
        "dynamics": ["pin"],
        "slcan": ["pyserial>=3.5"],
        "windows": ["pyserial>=3.5"],
        "debugger": [
            "pyqt6",
            "pyqtgraph",
            "pyvista",
            "pyvistaqt",
        ],
    },
    entry_points={
        "console_scripts": [
            "el-a3-debugger=MotorStudio.main:main",
        ],
    },
    classifiers=[
        "Programming Language :: Python :: 3",
        "Operating System :: POSIX :: Linux",
        "Operating System :: Microsoft :: Windows",
    ],
)
