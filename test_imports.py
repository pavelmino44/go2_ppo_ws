#!/usr/bin/env python3
# test_imports.py

print("Testing imports...")

try:
    import rclpy
    print("✓ rclpy")
except ImportError as e:
    print(f"✗ rclpy: {e}")

try:
    from geometry_msgs.msg import Twist
    print("✓ geometry_msgs")
except ImportError as e:
    print(f"✗ geometry_msgs: {e}")

try:
    from sensor_msgs.msg import PointCloud2
    print("✓ sensor_msgs")
except ImportError as e:
    print(f"✗ sensor_msgs: {e}")

try:
    from unitree_go.msg import LowState, LowCmd
    print("✓ unitree_go")
except ImportError as e:
    print(f"✗ unitree_go: {e}")

try:
    import numpy as np
    print(f"✓ numpy {np.__version__}")
except ImportError as e:
    print(f"✗ numpy: {e}")

try:
    import onnxruntime as ort
    print(f"✓ onnxruntime {ort.__version__}")
except ImportError as e:
    print(f"✗ onnxruntime: {e}")

try:
    import transforms3d
    print(f"✓ transforms3d {transforms3d.__version__}")
except ImportError as e:
    print(f"✗ transforms3d: {e}")

print("\nAll imports tested!")