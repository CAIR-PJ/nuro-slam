#!/usr/bin/env python3
"""Read-only dependency report: never installs or upgrades the working environment."""
import importlib
import json
import os
import platform
import re
import shutil
import subprocess
import sys
from pathlib import Path

def run(*cmd):
    p=subprocess.run(cmd,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
    if p.returncode: raise RuntimeError(f'{cmd}: {p.stdout}')
    return p.stdout.strip()

def main():
    if platform.system()!='Darwin' or platform.machine()!='arm64': raise RuntimeError('native Darwin arm64 required')
    if os.environ.get('ROS_DISTRO')!='jazzy': raise RuntimeError('source your Jazzy setup first')
    for mod in ['rclpy','ament_package','colcon_core','em','lark','numpy']:
        importlib.import_module(mod)
    for tool in ['ros2','cmake','brew','otool','c++']:
        if not shutil.which(tool): raise RuntimeError(f'missing tool: {tool}')
    report=dict(python=sys.executable,architecture=platform.machine(),ros_distro=os.environ['ROS_DISTRO'],
                ros2=shutil.which('ros2'),compiler=run('c++','--version'))
    packages={}
    # The port uses local camera/image adapters; do not inspect or link the
    # installed cv_bridge/image_geometry libraries (which may have mixed ABI).
    for pkg in ['ament_cmake','rclcpp','rosidl_default_generators','sensor_msgs','nav_msgs','tf2_ros']:
        packages[pkg]=run('ros2','pkg','prefix',pkg)
    report['ros_packages_and_opencv_linkage']=packages
    report['homebrew']={}
    for pkg in ['eigen','opencv','gsl','glog']:
        report['homebrew'][pkg]=run('brew','list','--versions',pkg)
    report['note']='Local camera/image adapters avoid cv_bridge and image_geometry linkage. OpenCV 5 / Eigen 5 compatibility and all other transitive ROS library ABIs remain unverified.'
    print(json.dumps(report,indent=2))
    Path('preflight.json').write_text(json.dumps(report,indent=2)+'\n')
if __name__=='__main__': main()
