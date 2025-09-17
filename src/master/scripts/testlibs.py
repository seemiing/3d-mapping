#!/usr/bin/env python3
import sys
import os
sys.path.insert(0, os.path.expanduser('~/catkin_ws_ov/src/master'))
import numpy as np
import open3d as o3d
import ros_numpy
print(f"NumPy version: {np.__version__}")
print(f"Open3D version: {o3d.__version__}")
print("ros_numpy imported successfully")