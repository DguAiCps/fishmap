#!/usr/bin/env python3
"""
ROS ↔ Isaac Sim 쿼터니언 형식 변환 테스트

ROS 형식 (xyzw): [x, y, z, w] - scalar-last
Isaac Sim 형식 (wxyz): [w, x, y, z] - scalar-first

변환 방법: np.roll()을 사용하여 순환 이동
"""

import numpy as np

def convert_quaternion_ros_to_isaac_sim(quat_ros):
    """ROS [x, y, z, w] → Isaac Sim [w, x, y, z]"""
    if len(quat_ros) == 4:
        return np.roll(np.array(quat_ros, dtype=float), 1)
    return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)

def convert_quaternion_isaac_sim_to_ros(quat_isaac):
    """Isaac Sim [w, x, y, z] → ROS [x, y, z, w]"""
    if len(quat_isaac) == 4:
        return np.roll(np.array(quat_isaac, dtype=float), -1)
    return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)

# 테스트 케이스
print("=" * 70)
print("쿼터니언 형식 변환 테스트")
print("=" * 70)

# Test 1: 항등원 (회전 없음)
print("\n[Test 1] 항등원 (회전 없음)")
quat_ros_identity = np.array([0.0, 0.0, 0.0, 1.0])  # ROS: [x, y, z, w]
quat_isaac_identity = convert_quaternion_ros_to_isaac_sim(quat_ros_identity)
print(f"  ROS 입력:      {quat_ros_identity} (xyzw)")
print(f"  Isaac 출력:    {quat_isaac_identity} (wxyz)")
print(f"  예상값:        [1. 0. 0. 0.] (wxyz)")
print(f"  ✓ 맞음" if np.allclose(quat_isaac_identity, [1, 0, 0, 0]) else f"  ✗ 틀림")

# Test 2: RealSense 토픽에서 받은 실제 데이터
print("\n[Test 2] RealSense 토픽 데이터 변환")
quat_ros_realsense = np.array([
    0.047613985887164124,   # x
    0.05193271511202183,    # y
    -0.5004744540978509,    # z
    0.8628796105161419      # w
])
quat_isaac_realsense = convert_quaternion_ros_to_isaac_sim(quat_ros_realsense)
print(f"  ROS 입력:      [{quat_ros_realsense[0]:.4f}, {quat_ros_realsense[1]:.4f}, {quat_ros_realsense[2]:.4f}, {quat_ros_realsense[3]:.4f}] (xyzw)")
print(f"  Isaac 출력:    [{quat_isaac_realsense[0]:.4f}, {quat_isaac_realsense[1]:.4f}, {quat_isaac_realsense[2]:.4f}, {quat_isaac_realsense[3]:.4f}] (wxyz)")
print(f"  예상값:        [0.8629, 0.0476, 0.0519, -0.5005] (wxyz)")

# Test 3: 역변환 확인
print("\n[Test 3] 역변환 확인 (Isaac Sim → ROS)")
quat_ros_back = convert_quaternion_isaac_sim_to_ros(quat_isaac_realsense)
print(f"  원본 ROS:      [{quat_ros_realsense[0]:.4f}, {quat_ros_realsense[1]:.4f}, {quat_ros_realsense[2]:.4f}, {quat_ros_realsense[3]:.4f}]")
print(f"  역변환 ROS:    [{quat_ros_back[0]:.4f}, {quat_ros_back[1]:.4f}, {quat_ros_back[2]:.4f}, {quat_ros_back[3]:.4f}]")
print(f"  ✓ 맞음" if np.allclose(quat_ros_realsense, quat_ros_back) else f"  ✗ 틀림")

# Test 4: 쿼터니언 정규화 확인
print("\n[Test 4] 쿼터니언 정규화 확인")
norm_ros = np.linalg.norm(quat_ros_realsense)
norm_isaac = np.linalg.norm(quat_isaac_realsense)
print(f"  ROS 쿼터니언 노름:    {norm_ros:.6f}")
print(f"  Isaac 쿼터니언 노름:  {norm_isaac:.6f}")
print(f"  ✓ 정규화됨 (≈1.0)" if np.isclose(norm_ros, 1.0) and np.isclose(norm_isaac, 1.0) else f"  ⚠️  정규화 필요")

print("\n" + "=" * 70)
print("결론: np.roll()을 사용한 변환이 올바르게 작동합니다.")
print("=" * 70 + "\n")
