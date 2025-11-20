# Isaac Sim 로봇 ROS 2 컨트롤 가이드

Isaac Sim에서 실행 중인 로봇을 ROS 2 토픽을 통해 제어하는 방법입니다.

---

## 📡 토픽 정보

### 토픽명
```
/target_pose
```

### 메시지 타입
```
geometry_msgs/msg/PoseStamped
```

### 토픽 역할
로봇의 **위치와 회전(Roll)**을 제어합니다.

---

## 📋 메시지 구조

```yaml
header:
  stamp:
    sec: 1234567890
    nsec: 123456789
  frame_id: "map"                    # 기준 좌표계

pose:
  position:
    x: 30.0                          # X 좌표 (미터)
    y: -160.0                        # Y 좌표 (미터)
    z: 40.0                          # Z 좌표 (미터)

  orientation:
    x: 0.2588                        # 쿼터니언 X (ROS xyzw 형식)
    y: 0.0                           # 쿼터니언 Y
    z: 0.0                           # 쿼터니언 Z
    w: 0.9659                        # 쿼터니언 W (스칼라)
```

### 쿼터니언 형식

이 가이드에서는 **ROS 표준 xyzw 형식** (scalar-last)을 사용합니다:
```
[x, y, z, w]  ← ROS 표준
```

Isaac Sim 내부에서는 자동으로 **wxyz 형식** (scalar-first)으로 변환됩니다:
```
[w, x, y, z]  ← Isaac Sim 내부 형식
```

---

## 🎮 제어 방법

### 1. 위치만 제어 (회전 없음)

```python
import rclpy
from geometry_msgs.msg import PoseStamped
import math

rclpy.init()
node = rclpy.create_node('robot_controller')
publisher = node.create_publisher(PoseStamped, '/target_pose', 10)

# 메시지 생성
msg = PoseStamped()
msg.header.frame_id = 'map'

# 위치 설정 (30, -160, 40)
msg.pose.position.x = 30.0
msg.pose.position.y = -160.0
msg.pose.position.z = 40.0

# 회전 없음 (Roll = 0도)
msg.pose.orientation.x = 0.0
msg.pose.orientation.y = 0.0
msg.pose.orientation.z = 0.0
msg.pose.orientation.w = 1.0

publisher.publish(msg)
```

### 2. 위치 + 회전(Roll) 제어

```python
import rclpy
from geometry_msgs.msg import PoseStamped
import math

rclpy.init()
node = rclpy.create_node('robot_controller')
publisher = node.create_publisher(PoseStamped, '/target_pose', 10)

# Roll 각도 (도 단위)
roll_deg = 45.0

# Roll을 라디안으로 변환
roll_rad = roll_deg * math.pi / 180.0

# Roll을 쿼터니언으로 변환
qx = math.sin(roll_rad / 2.0)
qy = 0.0
qz = 0.0
qw = math.cos(roll_rad / 2.0)

# 메시지 생성
msg = PoseStamped()
msg.header.frame_id = 'map'

# 위치 설정
msg.pose.position.x = 30.0
msg.pose.position.y = -160.0
msg.pose.position.z = 40.0

# 회전 설정
msg.pose.orientation.x = qx
msg.pose.orientation.y = qy
msg.pose.orientation.z = qz
msg.pose.orientation.w = qw

publisher.publish(msg)
```

---

## 🔄 Roll 각도 변환 공식

**Roll 각도(도) → 쿼터니언 변환:**

```
1. Roll을 라디안으로 변환
   roll_rad = roll_deg × π / 180

2. 쿼터니언 계산
   qx = sin(roll_rad / 2)
   qy = 0.0
   qz = 0.0
   qw = cos(roll_rad / 2)
```

**예시:**

| Roll(도) | Roll(라디안) | qx | qy | qz | qw |
|---------|------------|-----|----|----|-----|
| 0° | 0.0 | 0.0000 | 0.0 | 0.0 | 1.0000 |
| 30° | 0.5236 | 0.2588 | 0.0 | 0.0 | 0.9659 |
| 45° | 0.7854 | 0.3827 | 0.0 | 0.0 | 0.9239 |
| 90° | 1.5708 | 0.7071 | 0.0 | 0.0 | 0.7071 |
| 180° | 3.1416 | 1.0000 | 0.0 | 0.0 | 0.0000 |

---

## 🔧 ROS 2 CLI로 토픽 발행

### 기본 명령어

```bash
ros2 topic pub -1 /target_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 30.0, y: -160.0, z: 40.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

### Roll 45도 회전 예시

```bash
ros2 topic pub -1 /target_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 30.0, y: -160.0, z: 40.0}, orientation: {x: 0.3827, y: 0.0, z: 0.0, w: 0.9239}}}"
```

---

## 📊 좌표계 설명

### 위치 (Position)

- **X축**: 로봇의 전진/후진 방향
  - 양수: 전진
  - 음수: 후진

- **Y축**: 로봇의 좌우 이동
  - 양수: 오른쪽 이동
  - 음수: 왼쪽 이동

- **Z축**: 높이
  - 양수: 위로 이동
  - 음수: 아래로 이동

### 회전 (Orientation)

- **Roll (X축 회전)**: 로봇의 앞뒤축을 중심으로 회전
  - 양수: 시계 방향 회전 (우측이 위로)
  - 음수: 반시계 방향 회전 (좌측이 위로)

---

## ⚠️ 중요 사항

### 1. frame_id는 항상 "map"

```python
msg.header.frame_id = 'map'  # 필수
```

### 2. 쿼터니언 형식

Isaac Sim에서는 ROS 토픽의 쿼터니언 형식을 자동으로 변환합니다:

**수신 형식 (ROS xyzw):**
```python
msg.pose.orientation.x = 0.047  # x
msg.pose.orientation.y = 0.051  # y
msg.pose.orientation.z = -0.500 # z
msg.pose.orientation.w = 0.862  # w (스칼라)
```

**내부 변환 (Isaac Sim wxyz):**
Isaac Sim 내부에서 아래와 같이 자동 변환됩니다:
```
ROS [x, y, z, w] → Isaac Sim [w, x, y, z]
[0.047, 0.051, -0.500, 0.862] → [0.862, 0.047, 0.051, -0.500]
```

**변환 원리:**
```python
import numpy as np

# ROS 형식 쿼터니언 [x, y, z, w]
quat_ros = [0.047, 0.051, -0.500, 0.862]

# Isaac Sim 형식으로 변환 [w, x, y, z]
quat_isaac = np.roll(quat_ros, 1)
# 결과: [0.862, 0.047, 0.051, -0.500]
```

### 3. 부드러운 이동

로봇은 새로운 목표를 받으면 자동으로 **30프레임에 걸쳐 부드럽게 이동**합니다.
빠르게 변경하고 싶으면 짧은 시간 간격으로 토픽을 계속 발행하세요.

### 4. 쿼터니언 정규화

쿼터니언은 자동으로 정규화되므로, 계산 오류가 있더라도 대부분 작동합니다.
수신된 쿼터니언의 노름이 1.0에 가깝지 않더라도 내부에서 자동으로 정규화됩니다.

---

## 🧪 테스트 예제

### Python 테스트 스크립트

```python
#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import PoseStamped
import math
import time

def send_target_pose(x, y, z, roll_deg=0.0):
    """로봇에 목표 위치와 회전을 발행"""
    rclpy.init()
    node = rclpy.create_node('test_publisher')
    publisher = node.create_publisher(PoseStamped, '/target_pose', 10)

    # Roll을 쿼터니언으로 변환
    roll_rad = roll_deg * math.pi / 180.0
    qx = math.sin(roll_rad / 2.0)
    qw = math.cos(roll_rad / 2.0)

    # 메시지 생성
    msg = PoseStamped()
    msg.header.frame_id = 'map'
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z
    msg.pose.orientation.x = qx
    msg.pose.orientation.y = 0.0
    msg.pose.orientation.z = 0.0
    msg.pose.orientation.w = qw

    # 5회 발행 (확실한 전달을 위해)
    for i in range(5):
        publisher.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.01)

    node.destroy_node()
    rclpy.shutdown()
    print(f"✓ 발행 완료: pos=({x}, {y}, {z}), roll={roll_deg}°")

# 테스트
if __name__ == '__main__':
    print("로봇 컨트롤 테스트")

    # 위치만 변경
    send_target_pose(30.0, -160.0, 40.0)
    time.sleep(2)

    # 위치 + 회전
    send_target_pose(35.0, -155.0, 40.0, roll_deg=45.0)
    time.sleep(2)

    # 다른 위치로 이동
    send_target_pose(40.0, -150.0, 40.0, roll_deg=90.0)

    print("✓ 모든 테스트 완료")
```

---

## 🚀 빠른 시작

### 1단계: Isaac Sim 실행

```bash
python load_map_and_robot.py
```

### 2단계: 토픽 퍼블리셔 실행

```bash
# bash 스크립트 사용
./publish_moving_target.sh --x 5.0

# 또는 Python 테스트 스크립트 사용
python test_robot_control.py
```

### 3단계: 로봇 이동 확인

Isaac Sim 뷰포트에서 로봇이 움직이는 것을 확인하세요.

---

## 📚 참고 자료

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [geometry_msgs/PoseStamped](https://docs.ros2.org/humble/api/geometry_msgs/structures.html)
- [Quaternion Reference](https://en.wikipedia.org/wiki/Quaternion)
