# Isaac Sim 로봇 시뮬레이터

Isaac Sim 5.0.0에서 맵과 터틀봇을 로드하고, ROS 2 토픽을 통해 로봇을 제어하는 시뮬레이션 프로젝트입니다.

## 📋 구성 요소

- **load_map_and_robot.py**: Isaac Sim 메인 시뮬레이션 스크립트
- **publish_moving_target.sh**: ROS 2 토픽 퍼블리셔 (로봇 제어)
- **config.py**: 맵, 로봇 모델 경로 설정
- **637.usdc**: 맵 파일 (Universal Scene Description)
- **create_3.usd**: 터틀봇 로봇 모델

## 🚀 시작하기

### 1. 환경 설정

```bash
# Isaac Sim 환경 활성화
conda activate isaacsim_env

# 또는 Isaac Sim이 설치된 경로로 이동
cd /home/yunseon/code/fishmap
```

### 2. Isaac Sim 시뮬레이션 실행

터미널 1에서 실행:

```bash
python load_map_and_robot.py
```

**예상 출력:**
```
============================================================
Isaac Sim - 맵 + 터틀봇 로더
============================================================

World 초기화 중...
✓ World 생성됨

라이팅 추가 중...
✓ 라이팅 추가됨

맵 로드 중: /home/yunseon/code/fishmap/637.usdc
✓ 맵 파일 로드 완료
✓ 맵 콜리전 추가 완료

...
시뮬레이션 실행 중...
```

### 3. 로봇 제어 (ROS 2 토픽 발행)

터미널 2에서 실행:

```bash
chmod +x publish_moving_target.sh
./publish_moving_target.sh [옵션]
```

## 🎮 로봇 제어 방법

### 기본 사용법

```bash
# 1. 위치만 이동 (x축으로 계속 증가)
./publish_moving_target.sh

# 2. x축 이동 + 회전
./publish_moving_target.sh --x 5.0 --roll 10.0

# 3. y축 이동 + 회전
./publish_moving_target.sh --y 3.0 --roll 5.0

# 4. 원형 이동 + 회전
./publish_moving_target.sh --x 2.0 --y 2.0 --roll 15.0

# 5. 회전만
./publish_moving_target.sh --x 0 --roll 10.0
```

### 파라미터 설명

| 파라미터 | 설명 | 기본값 | 단위 |
|---------|------|--------|------|
| `--x` | x축 이동 증가량 | 5.0 | 미터/업데이트 |
| `--y` | y축 이동 증가량 | 0.0 | 미터/업데이트 |
| `--z` | z축 이동 증가량 | 0.0 | 미터/업데이트 |
| `--roll` | Roll(X축 회전) 증가량 | 0.0 | 도/업데이트 |

## 📊 로봇 제어 구조

### 위치 제어

로봇의 위치는 3D 좌표계로 제어됩니다:
- **x**: 전진/후진
- **y**: 좌측/우측 이동
- **z**: 상하 이동

### 회전 제어

로봇의 회전은 **Roll(X축 회전)** 만 사용됩니다:
- **Roll**: 로봇이 앞뒤축을 중심으로 기울어지는 회전

#### 쿼터니언 변환

내부적으로는 Roll 각도를 쿼터니언으로 변환하여 ROS 토픽으로 발행됩니다:

```
Roll 각도 (도)
    ↓
라디안 변환: roll_rad = roll_deg × π / 180
    ↓
쿼터니언 계산:
  - qx = sin(roll_rad / 2)
  - qy = 0.0
  - qz = 0.0
  - qw = cos(roll_rad / 2)
    ↓
ROS 2 PoseStamped 토픽 발행
```

### 부드러운 이동

로봇의 이동은 자동으로 30프레임에 걸쳐 부드럽게 보간됩니다:
- 위치: 선형 보간 (Linear Interpolation)
- 회전: 각도 선형 보간 후 쿼터니언으로 변환

## 🔧 ROS 2 토픽 정보

### 토픽명
```
/camera_pose
```

### 메시지 타입
```
geometry_msgs/msg/PoseStamped
```

### 메시지 구조
```yaml
header:
  frame_id: 'map'
pose:
  position:
    x: float64  # X 위치 (미터)
    y: float64  # Y 위치 (미터)
    z: float64  # Z 위치 (미터)
  orientation:
    x: float64  # 쿼터니언 X (ROS xyzw 형식)
    y: float64  # 쿼터니언 Y
    z: float64  # 쿼터니언 Z
    w: float64  # 쿼터니언 W (스칼라)
```

### 쿼터니언 형식 변환

Isaac Sim에서는 쿼터니언 형식을 자동으로 변환합니다:

| 형식 | 순서 | 예시 | 비고 |
|------|------|------|------|
| **ROS 표준** (수신) | xyzw (scalar-last) | [0.047, 0.051, -0.500, 0.862] | RealSense에서 발행 |
| **Isaac Sim** (내부) | wxyz (scalar-first) | [0.862, 0.047, 0.051, -0.500] | 자동 변환됨 |

**변환 방법:**
```python
import numpy as np
quat_ros = [x, y, z, w]  # ROS 형식
quat_isaac = np.roll(quat_ros, 1)  # [w, x, y, z] 형식으로 변환
```

## 📝 커스텀 토픽 발행

Python에서 커스텀 토픽을 발행하려면:

```python
import rclpy
from geometry_msgs.msg import PoseStamped
import math

rclpy.init()
node = rclpy.create_node('custom_controller')
publisher = node.create_publisher(PoseStamped, '/target_pose', 10)

# Roll 각도를 쿼터니언으로 변환
roll_deg = 45.0
roll_rad = roll_deg * math.pi / 180

msg = PoseStamped()
msg.header.frame_id = 'map'
msg.pose.position.x = 30.0
msg.pose.position.y = -160.0
msg.pose.position.z = 40.0
msg.pose.orientation.x = math.sin(roll_rad / 2)
msg.pose.orientation.y = 0.0
msg.pose.orientation.z = 0.0
msg.pose.orientation.w = math.cos(roll_rad / 2)

publisher.publish(msg)
```

## ⚙️ 설정 파일 (config.py)

```python
MAP_FILE = "/path/to/637.usdc"              # 맵 파일 경로
TURTLEBOT_FILE = "/path/to/create_3.usd"   # 로봇 모델 경로
TURTLEBOT_START_POS = [23.0, -170.0, 40]   # 로봇 초기 위치 [x, y, z]
```

## 🛑 시뮬레이션 종료

- Isaac Sim 창을 닫거나 터미널에서 **Ctrl+C** 입력
- ROS 토픽 퍼블리셔는 **Ctrl+C**로 종료

## 🔍 로그 및 디버깅

### Isaac Sim 콘솔 출력
- 로봇 위치: 500 스텝마다 출력 (약 8초 간격 @ 60fps)
- 새로운 목표 수신: 토픽 메시지 수신 시 즉시 출력
- 목표 도달: 보간 완료 시 출력

### ROS 토픽 모니터링
```bash
# 발행되는 토픽 모니터링
ros2 topic echo /target_pose
```

## 💡 팁

1. **빠른 이동**: `--x 10.0` 등으로 증가량을 크게 설정
2. **느린 이동**: `--x 1.0` 등으로 증가량을 작게 설정
3. **회전 테스트**: `./publish_moving_target.sh --x 0 --roll 5.0`로 회전만 테스트
4. **여러 경로 시뮬레이션**: 다양한 파라미터 조합으로 로봇 움직임 테스트

## 📚 참고

- Isaac Sim 공식 문서: https://docs.omniverse.nvidia.com/isaacsim
- ROS 2 Humble: https://docs.ros.org/en/humble/
- USD (Universal Scene Description): https://graphics.pixar.com/usd/
