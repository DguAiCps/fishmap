# 쿼터니언 형식 변환 가이드

## 개요

Isaac Sim과 ROS 2 사이의 쿼터니언 형식이 다릅니다. Isaac Sim은 자동으로 변환하므로 사용자는 ROS 표준 형식을 그대로 사용하면 됩니다.

---

## 쿼터니언 형식 비교

### ROS 2 표준 형식 (xyzw - scalar-last)
```
[x, y, z, w]
```
- **x, y, z**: 벡터 부분 (허수부)
- **w**: 스칼라 부분 (실수부)
- RealSense, geometry_msgs, tf2 등 ROS 생태계에서 사용

### Isaac Sim 내부 형식 (wxyz - scalar-first)
```
[w, x, y, z]
```
- **w**: 스칼라 부분 (실수부) - 맨 앞
- **x, y, z**: 벡터 부분 (허수부)
- Isaac Lab과 Isaac Sim의 내부 API에서 사용

---

## 변환 방법

### 수식
```
ROS [x, y, z, w] → Isaac Sim [w, x, y, z]
Isaac Sim [w, x, y, z] → ROS [x, y, z, w]
```

### Python 구현
```python
import numpy as np

# ROS → Isaac Sim 변환
def ros_to_isaac_sim(quat_ros):
    """
    ROS xyzw → Isaac Sim wxyz 변환
    np.roll(arr, 1)은 배열을 왼쪽으로 한 칸 회전
    """
    return np.roll(np.array(quat_ros), 1)

# Isaac Sim → ROS 변환
def isaac_sim_to_ros(quat_isaac):
    """
    Isaac Sim wxyz → ROS xyzw 변환
    np.roll(arr, -1)은 배열을 오른쪽으로 한 칸 회전
    """
    return np.roll(np.array(quat_isaac), -1)

# 사용 예시
quat_ros = [0.047, 0.051, -0.500, 0.862]  # x, y, z, w
quat_isaac = ros_to_isaac_sim(quat_ros)
# 결과: [0.862, 0.047, 0.051, -0.500]  # w, x, y, z
```

---

## 실제 예시

### RealSense에서 발행한 쿼터니언
```bash
ros2 topic echo /camera_pose
---
orientation:
  x: 0.047613985887164124
  y: 0.05193271511202183
  z: -0.5004744540978509
  w: 0.8628796105161419
```

### Isaac Sim 내부 표현
```python
# 자동 변환됨:
[0.8628796105161419, 0.047613985887164124, 0.05193271511202183, -0.5004744540978509]
#  w                  x                     y                    z
```

---

## 코드에서의 구현

### load_map_and_robot.py의 변환 함수

```python
def convert_quaternion_ros_to_isaac_sim(quat_ros):
    """
    ROS 쿼터니언을 Isaac Sim 형식으로 변환

    ROS 형식 (xyzw, scalar-last): [x, y, z, w]
    Isaac Sim 형식 (wxyz, scalar-first): [w, x, y, z]

    입력: [x, y, z, w] (ROS 표준 형식)
    반환: [w, x, y, z] (Isaac Sim 표준 형식)

    변환 방법: np.roll()을 사용하여 마지막 요소(w)를 맨 앞으로 이동
    """
    if len(quat_ros) == 4:
        # xyzw → wxyz 변환: np.roll(arr, 1) = [w, x, y, z]
        return np.roll(np.array(quat_ros, dtype=float), 1)
    return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)  # 기본값: identity (wxyz)
```

### ROS 토픽 콜백에서의 사용

```python
def target_pose_callback(msg):
    global target_position, target_orientation
    # ROS 형식 [x, y, z, w] 그대로 저장
    target_position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
    target_orientation = [
        msg.pose.orientation.x,  # x
        msg.pose.orientation.y,  # y
        msg.pose.orientation.z,  # z
        msg.pose.orientation.w   # w
    ]

# 메인 루프에서
if target_orientation:
    # ROS 쿼터니언 [x, y, z, w]을 Isaac Sim 형식 [w, x, y, z]로 변환
    target_quat = convert_quaternion_ros_to_isaac_sim(target_orientation)
```

---

## SLERP (구면 선형 보간)

SLERP는 두 쿼터니언 사이를 부드럽게 보간하는 방법입니다.

### 작동 원리
1. 두 쿼터니언을 정규화
2. 내적 계산
3. 보간 각도 계산
4. 구면 선형 보간 수행
5. 결과 정규화

### 코드
```python
def slerp(q0, q1, t):
    """
    쿼터니언 구면 선형 보간 (SLERP)
    q0: 시작 쿼터니언 [w, x, y, z] (Isaac Sim 형식)
    q1: 끝 쿼터니언 [w, x, y, z] (Isaac Sim 형식)
    t: 보간 비율 (0.0 ~ 1.0)
    반환: 보간된 쿼터니언 [w, x, y, z] (Isaac Sim 형식)
    """
    q0 = np.array(q0, dtype=float)
    q1 = np.array(q1, dtype=float)

    # 정규화
    q0 = q0 / np.linalg.norm(q0)
    q1 = q1 / np.linalg.norm(q1)

    # 내적 계산
    dot_product = np.dot(q0, q1)

    # 음수면 한 쪽을 반대로 (더 짧은 경로 선택)
    if dot_product < 0.0:
        q1 = -q1
        dot_product = -dot_product

    # 범위 제한
    dot_product = np.clip(dot_product, -1.0, 1.0)

    # SLERP 계산
    theta_0 = np.arccos(dot_product)
    theta = theta_0 * t

    if np.sin(theta_0) < 1e-6:
        # 거의 같은 쿼터니언일 때 선형 보간
        result = q0 + t * (q1 - q0)
    else:
        q2 = (q1 - q0 * dot_product) / np.sin(theta_0)
        q2 = q2 / np.linalg.norm(q2)
        result = q0 * np.cos(theta) + q2 * np.sin(theta)

    return result / np.linalg.norm(result)
```

---

## 정규화 (Normalization)

쿼터니언은 단위 쿼터니언(노름 = 1)이어야 올바른 회전을 나타냅니다.

```python
# 쿼터니언 정규화
def normalize_quaternion(quat):
    quat = np.array(quat, dtype=float)
    return quat / np.linalg.norm(quat)

# 검증
quat = [0.047, 0.051, -0.500, 0.862]
norm = np.linalg.norm(quat)
print(f"노름: {norm}")  # 약 1.0이어야 함
```

---

## 테스트 및 검증

### test_quaternion_conversion.py 실행
```bash
python test_quaternion_conversion.py
```

**결과:**
```
[Test 1] 항등원 (회전 없음)
  ROS 입력:      [0. 0. 0. 1.] (xyzw)
  Isaac 출력:    [1. 0. 0. 0.] (wxyz)
  ✓ 맞음

[Test 2] RealSense 토픽 데이터 변환
  ROS 입력:      [0.0476, 0.0519, -0.5005, 0.8629] (xyzw)
  Isaac 출력:    [0.8629, 0.0476, 0.0519, -0.5005] (wxyz)

[Test 3] 역변환 확인 (Isaac Sim → ROS)
  원본 ROS:      [0.0476, 0.0519, -0.5005, 0.8629]
  역변환 ROS:    [0.0476, 0.0519, -0.5005, 0.8629]
  ✓ 맞음

[Test 4] 쿼터니언 정규화 확인
  ROS 쿼터니언 노름:    1.000000
  Isaac 쿼터니언 노름:  1.000000
  ✓ 정규화됨 (≈1.0)
```

---

## 참고 자료

- [Isaac Lab - Quaternion Utils](https://isaac-sim.github.io/IsaacLab/main/source/api/lab/isaaclab.utils.html)
- [ROS Quaternion Convention](https://docs.ros.org/en/humble/Concepts/Intermediate/Tf2/Quaternion-Fundamentals.html)
- [Wikipedia - Quaternion](https://en.wikipedia.org/wiki/Quaternion)
- [Quaternion Interpolation and Animation](https://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/)

---

## FAQ

**Q: 변환 없이 Isaac Sim에 직접 ROS 쿼터니언을 보낼 수 있나요?**
A: 아니요. Isaac Sim의 내부 API는 wxyz 형식을 사용하므로 변환이 필요합니다. 하지만 load_map_and_robot.py에서 자동으로 변환하므로 사용자는 신경 쓸 필요 없습니다.

**Q: 쿼터니언 노름이 1.0이 아니면 어떻게 되나요?**
A: 내부에서 자동으로 정규화되므로 문제없습니다. 하지만 최적의 성능을 위해 발행할 때부터 정규화된 쿼터니언을 사용하는 것이 좋습니다.

**Q: np.roll()의 정확한 동작은?**
A: `np.roll(arr, 1)`은 배열의 모든 요소를 왼쪽으로 한 칸 이동합니다.
- 입력: `[a, b, c, d]`
- 출력: `[d, a, b, c]`

이를 이용해 `[x, y, z, w]` → `[w, x, y, z]` 변환이 가능합니다.
