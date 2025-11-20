#!/usr/bin/env python3
"""
Isaac Sim 5.0 - 맵(USDC) + 터틀봇 로더

공식 예제 기반 (turtlebot3_nine_colored.py)

실행 방법:
  python load_map_and_robot.py
"""

import os
import sys

# config.py에서 설정 로드
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from config import MAP_FILE, TURTLEBOT_FILE, TURTLEBOT_START_POS

print("\n" + "="*60)
print("Isaac Sim - 맵 + 터틀봇 로더 (ROS 토픽 구독)")
print("="*60)

# Isaac Sim 초기화 (가장 먼저 실행)
print("\nInitializing Isaac Sim...")
from isaacsim import SimulationApp

CONFIG = {
    "headless": False,
    "width": 1920,
    "height": 1080,
}

simulation_app = SimulationApp(CONFIG)

# ===== ROS 경로 설정 (Isaac Sim 이후) =====
print("\nSetting up ROS 2 environment...")

ws_path = os.path.expanduser("~/IsaacSim-ros_workspaces/build_ws/humble/humble_ws/install")# 이거 바꿈 됩니다~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
ld_library_path = "/usr/lib/x86_64-linux-gnu"

if os.path.exists(ws_path):
    ros2_lib_paths = [
        os.path.join(ws_path, "lib"),
        os.path.join(ws_path, "local/lib"),
    ]
    for lib_path in ros2_lib_paths:
        if os.path.exists(lib_path):
            ld_library_path = lib_path + ":" + ld_library_path
    os.environ["AMENT_PREFIX_PATH"] = ws_path

os.environ["LD_LIBRARY_PATH"] = ld_library_path + ":" + os.environ.get("LD_LIBRARY_PATH", "")

# Python 3.10 경로 제거하고 3.11 ROS 경로 추가
original_path_count = len(sys.path)
sys.path = [p for p in sys.path if 'python3.10' not in p or 'ros' not in p.lower()]
removed_count = original_path_count - len(sys.path)
if removed_count > 0:
    print(f"  Removed {removed_count} Python 3.10 ROS paths")

if os.path.exists(ws_path):
    ros2_paths = [
        os.path.join(ws_path, "local/lib/python3.11/dist-packages"),
        os.path.join(ws_path, "lib/python3.11/site-packages"),
    ]
    for path in ros2_paths:
        if os.path.exists(path):
            sys.path.insert(0, path)
            print(f"  ✓ Added ROS path: {path}")

# ROS 구독을 위한 전역 변수
target_position = None
target_orientation = None
initial_position_received = False  # 초기 위치를 받았는지 확인
ROS_AVAILABLE = False

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    ROS_AVAILABLE = True
    print("✓ ROS 2 modules imported successfully")
except ImportError as e:
    print(f"⚠️  ROS 2 import failed: {e}")
    ROS_AVAILABLE = False

# Isaac Sim 모듈들 import
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
import numpy as np
from pxr import UsdGeom, Gf, UsdLux, UsdPhysics, UsdShade, Sdf

print("\n" + "="*60)
print("Isaac Sim - 맵 + 터틀봇 로더")
print("="*60)

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


def conjugate_quaternion(quat):
    """
    쿼터니언 켤레(역회전) 계산
    입력: [x, y, z, w]
    반환: [x, y, z, w]의 켤레 (역회전)
    """
    quat = np.array(quat, dtype=float)
    return np.array([-quat[0], -quat[1], -quat[2], quat[3]], dtype=float)


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

    # 범위 제한 (수치 오차 방지)
    dot_product = np.clip(dot_product, -1.0, 1.0)

    # 두 쿼터니언 사이의 각도
    theta_0 = np.arccos(dot_product)
    theta = theta_0 * t

    # SLERP 계산
    if np.sin(theta_0) < 1e-6:
        # 거의 같은 쿼터니언일 때 선형 보간
        result = q0 + t * (q1 - q0)
    else:
        q2 = (q1 - q0 * dot_product) / np.sin(theta_0)
        q2 = q2 / np.linalg.norm(q2)
        result = q0 * np.cos(theta) + q2 * np.sin(theta)

    return result / np.linalg.norm(result)


def init_ros_node():
    """ROS 노드 초기화"""
    global target_position
    if not ROS_AVAILABLE:
        return None

    try:
        if not rclpy.ok():
            rclpy.init()

        node = rclpy.create_node('isaac_sim_robot_controller')

        def target_pose_callback(msg):
            global target_position, target_orientation
            target_position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
            target_orientation = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]

            # 쿼터니언 변환 후 값 확인
            quat_isaac = convert_quaternion_ros_to_isaac_sim(target_orientation)
            print(f"📍 ROS 토픽 수신:")
            print(f"   위치: pos=({target_position[0]:.2f}, {target_position[1]:.2f}, {target_position[2]:.2f})")
            print(f"   ROS 쿼터니언 (xyzw): [{target_orientation[0]:.4f}, {target_orientation[1]:.4f}, {target_orientation[2]:.4f}, {target_orientation[3]:.4f}]")
            print(f"   Isaac 쿼터니언 (wxyz): [{quat_isaac[0]:.4f}, {quat_isaac[1]:.4f}, {quat_isaac[2]:.4f}, {quat_isaac[3]:.4f}]")

        # /camera_pose 토픽 구독
        node.create_subscription(
            PoseStamped,
            '/camera_pose',
            target_pose_callback,
            10
        )

        print("✓ ROS 노드 초기화 완료")
        print("  토픽: /camera_pose 구독 (geometry_msgs/PoseStamped)")
        print("  대기 중: ROS 토픽 메시지...")
        return node

    except Exception as e:
        print(f"⚠️  ROS 초기화 실패: {e}")
        import traceback
        traceback.print_exc()
        return None


def load_map(stage, map_path, map_prim_path="/World/Map", position=[0, 0, 0]):
    """맵 파일 로드 (OBJ, USDC, USD 지원) - 콜리전 포함"""
    if not os.path.exists(map_path):
        print(f"❌ 맵 파일을 찾을 수 없습니다: {map_path}")
        return False

    file_ext = os.path.splitext(map_path)[1].lower()
    supported = ['.obj', '.usdc', '.usd', '.usda']

    if file_ext not in supported:
        print(f"❌ 지원하지 않는 파일 형식: {file_ext}")
        print(f"   지원 형식: {supported}")
        return False

    try:
        print(f"\n맵 로드 중: {map_path}")
        print(f"  맵 위치: x={position[0]}, y={position[1]}, z={position[2]}")

        # Map 컨테이너 생성 (UsdGeom.Xform으로 명시적 변환)
        map_xform = UsdGeom.Xform.Define(stage, map_prim_path)
        map_prim = map_xform.GetPrim()

        # 맵 회전: Z축 중심으로 180도 회전
        rotate_z_op = map_xform.AddRotateZOp()
        rotate_z_op.Set(180.0)

        # 맵 위치 설정
        if position != [0, 0, 0]:
            translate_op = map_xform.AddTranslateOp()
            translate_op.Set(Gf.Vec3d(position[0], position[1], position[2]))

        map_prim.GetReferences().AddReference(map_path)

        # 맵 하위의 모든 Mesh에 콜리전 추가
        def add_collision_recursive(prim, depth=0):
            """재귀적으로 모든 메시에 콜리전 추가"""
            if depth > 20:  # 깊이 제한
                return

            if prim.IsA(UsdGeom.Mesh):
                # Mesh에 콜리전 추가
                UsdPhysics.CollisionAPI.Apply(prim)
                # 메시 콜리전 근사값 설정 (성능 개선)
                try:
                    UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr().Set("convexDecomposition")
                except:
                    pass

            # 자식 프림에 대해 재귀
            for child in prim.GetChildren():
                add_collision_recursive(child, depth + 1)

        add_collision_recursive(map_prim)

        print(f"✓ 맵 파일 로드 완료: {map_path}")
        print(f"✓ 맵 콜리전 추가 완료")
        return True

    except Exception as e:
        print(f"❌ 맵 파일 로드 중 오류: {e}")
        import traceback
        traceback.print_exc()
        return False


class RobotWrapper:
    """Isaac Sim Robot 클래스를 사용하지 않는 간단한 로봇 래퍼"""
    def __init__(self, prim_path, stage):
        self.prim_path = prim_path
        self.stage = stage
        self.prim = stage.GetPrimAtPath(prim_path)

    def set_world_pose(self, position, orientation):
        """
        로봇의 위치와 방향 설정
        position: [x, y, z]
        orientation: [w, x, y, z] (Isaac Sim 형식)
        """
        if not self.prim.IsValid():
            print(f"⚠️  프림이 유효하지 않음: {self.prim_path}")
            return

        try:
            xform = UsdGeom.Xformable(self.prim)

            # 쿼터니언 → 오일러 각도 (roll, pitch, yaw)
            w, x, y, z = orientation[0], orientation[1], orientation[2], orientation[3]

            roll = 0.0#np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
            pitch = 0.0#np.arcsin(2*(w*y - z*x))
            yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))

            # 오일러 각도를 도 단위로 변환
            roll_deg = np.degrees(roll)
            pitch_deg = np.degrees(pitch)
            yaw_deg = np.degrees(yaw)

            # translate 속성 찾기 또는 생성
            translate_attr = self.prim.GetAttribute("xformOp:translate")
            if not translate_attr:
                # xformOp:translate 속성이 없으면 생성
                ops = xform.GetOrderedXformOps()
                if len(ops) == 0:
                    translate_attr = xform.AddTranslateOp()
                else:
                    translate_attr = xform.InsertTranslateOp(0)

            translate_attr.Set(Gf.Vec3d(position[0], position[1], position[2]))

            # rotate Z 속성 찾기 또는 생성
            for attr_name in ["xformOp:rotateZ", "xformOp:rotateZ:0"]:
                rotate_z_attr = self.prim.GetAttribute(attr_name)
                if rotate_z_attr:
                    break
            else:
                rotate_z_attr = xform.AddRotateZOp()

            rotate_z_attr.Set(yaw_deg)

            # rotate Y 속성 찾기 또는 생성
            for attr_name in ["xformOp:rotateY", "xformOp:rotateY:0"]:
                rotate_y_attr = self.prim.GetAttribute(attr_name)
                if rotate_y_attr:
                    break
            else:
                rotate_y_attr = xform.AddRotateYOp()

            rotate_y_attr.Set(pitch_deg)

            # rotate X 속성 찾기 또는 생성
            for attr_name in ["xformOp:rotateX", "xformOp:rotateX:0"]:
                rotate_x_attr = self.prim.GetAttribute(attr_name)
                if rotate_x_attr:
                    break
            else:
                rotate_x_attr = xform.AddRotateXOp()

            rotate_x_attr.Set(roll_deg)

        except Exception as e:
            print(f"⚠️  위치/방향 설정 중 오류: {e}")

    def get_world_pose(self):
        """로봇의 현재 위치와 방향 반환"""
        if not self.prim.IsValid():
            return None, None

        try:
            xform = UsdGeom.Xformable(self.prim)
            matrix = xform.GetLocalTransformation()

            # 위치 추출
            position = np.array([matrix.ExtractTranslation()[0],
                                matrix.ExtractTranslation()[1],
                                matrix.ExtractTranslation()[2]])

            # 방향 추출 (행렬 → 쿼터니언)
            rotation_matrix = Gf.Matrix3d(matrix)
            quat = Gf.Quatd(rotation_matrix)
            orientation = np.array([quat.GetReal(), quat.GetImaginary()[0],
                                   quat.GetImaginary()[1], quat.GetImaginary()[2]])

            return position, orientation
        except:
            return None, None


def load_turtlebot(stage, turtlebot_usd, position):
    """
    터틀봇 모델 로드 - 물리 시뮬레이션 비활성화
    Robot 클래스를 사용하지 않음 (충돌 문제 해결)
    """
    try:
        robot_path = "/World/Turtlebot3"

        print(f"\n터틀봇 로드 중...")
        print(f"  경로: {turtlebot_usd}")
        print(f"  위치: {position}")

        # USD reference 추가
        add_reference_to_stage(usd_path=turtlebot_usd, prim_path=robot_path)

        # 물리 시뮬레이션 비활성화 (충돌 제거만, 메시는 유지)
        def disable_physics_recursive(prim, depth=0):
            """재귀적으로 모든 물리 기능 비활성화"""
            if depth > 20:  # 깊이 제한
                return

            # Visibility 확인 및 활성화 (메시 표시)
            try:
                vis_attr = prim.GetAttribute("visibility")
                if vis_attr and vis_attr.Get() == "invisible":
                    vis_attr.Set("inherited")
            except:
                pass

            # CollisionAPI 비활성화 (API 자체는 남겨두고 충돌만 비활성화)
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                try:
                    collision_api = UsdPhysics.CollisionAPI(prim)
                    collision_api.GetCollisionEnabledAttr().Set(False)
                except:
                    pass

            # RigidBodyAPI를 kinematic으로 설정
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                try:
                    rigidbody = UsdPhysics.RigidBodyAPI(prim)
                    rigidbody.CreateKinematicEnabledAttr().Set(True)
                except:
                    pass

            # 자식 프림에 대해 재귀
            for child in prim.GetChildren():
                disable_physics_recursive(child, depth + 1)

        robot_prim = stage.GetPrimAtPath(robot_path)
        disable_physics_recursive(robot_prim)

        # 스케일 설정
        try:
            scale_attr = robot_prim.GetAttribute("xformOp:scale")

            if scale_attr:
                scale_attr.Set(Gf.Vec3d(100, 100, 100))
                print(f"✓ 터틀봇 스케일 설정: 100배")
            else:
                scale_attr = robot_prim.CreateAttribute("xformOp:scale", Sdf.ValueTypeNames.Double3)
                scale_attr.Set(Gf.Vec3d(100, 100, 100))
                print(f"✓ 터틀봇 스케일 설정: 100배 (새로 생성)")

        except Exception as scale_err:
            print(f"⚠️  스케일 설정 중 오류: {scale_err}")

        # RobotWrapper 객체 생성
        robot = RobotWrapper(robot_path, stage)

        print(f"✓ 터틀봇 로드 완료")
        print(f"✓ 터틀봇 물리 비활성화 완료")
        return robot

    except Exception as e:
        print(f"❌ 터틀봇 로드 중 오류: {e}")
        import traceback
        traceback.print_exc()
        return None


def main():
    """메인 함수"""

    # ROS 노드 초기화
    ros_node = None
    if ROS_AVAILABLE:
        ros_node = init_ros_node()

    # World 생성
    print("\nWorld 초기화 중...")
    world = World(stage_units_in_meters=1.0)
    stage = world.stage

    # World 프림 생성
    stage.DefinePrim("/World", "Xform")
    print("✓ World 생성됨")

    # 라이팅 추가
    print("\n라이팅 추가 중...")
    distant_light = UsdLux.DistantLight.Define(stage, "/World/DistantLight")
    distant_light.CreateIntensityAttr(1500)
    distant_light.CreateAngleAttr(0.53)
    distant_light.AddRotateXYZOp().Set(Gf.Vec3f(-45, 0, 0))

    dome_light = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome_light.CreateIntensityAttr(500)
    print("✓ 라이팅 추가됨")

    # 카메라 설정
    print("\n카메라 설정 중...")

    # 로봇의 오른쪽 옆에서 로봇의 측면을 바라보도록 카메라 배치
    # 로봇이 +X 방향을 바라보면 카메라는 +Y 방향(오른쪽)의 측면에서 로봇을 봄
    camera_pos = np.array(TURTLEBOT_START_POS) + np.array([-100, 0, 50])  # 로봇 오른쪽 옆 상단

    try:
        # UsdGeom.Xform을 통한 카메라 생성
        camera_xform = UsdGeom.Xform.Define(stage, "/World/CameraXform")
        camera_xform.AddTranslateOp().Set(Gf.Vec3d(camera_pos[0], camera_pos[1], camera_pos[2]))

        # 카메라 회전: Z축 중심으로 -90도 (로봇 오른쪽에서 바라보기)
        rot_z = camera_xform.AddRotateZOp()
        rot_z.Set(-90.0)

        # 추가 회전: X축 중심으로 30도 (위에서 내려다보는 각도)
        rot_x = camera_xform.AddRotateXOp()
        rot_x.Set(60.0)

        # 카메라 생성
        camera_prim = UsdGeom.Camera.Define(stage, "/World/CameraXform/Camera")
        camera_prim.GetFocalLengthAttr().Set(24.0)  # 초점거리 설정

        # 카메라를 기본 뷰포트로 설정
        try:
            from omni.kit.viewport.utility import get_active_viewport
            viewport = get_active_viewport()
            if viewport:
                viewport.camera_path = "/World/CameraXform/Camera"
        except:
            pass

        print(f"✓ 카메라 위치: ({camera_pos[0]:.1f}, {camera_pos[1]:.1f}, {camera_pos[2]:.1f})")
        print(f"✓ 카메라가 로봇 오른쪽 옆에서 바라봄")
    except Exception as e:
        print(f"⚠️  카메라 설정 중 오류: {e}")
        # 카메라 설정 실패해도 시뮬레이션은 계속 진행

    # 맵 로드 (y축 260, x축 -140 이동)
    map_loaded = load_map(stage, MAP_FILE, position=[-140, 260, 0])

    # World 초기화
    print("\n물리 시뮬레이션 초기화 중...")
    world.reset()
    print("✓ 초기화 완료")

    # 터틀봇 로드
    robot = load_turtlebot(stage, TURTLEBOT_FILE, TURTLEBOT_START_POS)

    world.reset()

    # 시뮬레이션 실행
    print("\n" + "="*60)
    print("시뮬레이션 실행 중...")
    print("="*60)
    print(f"\n맵: {'로드됨' if map_loaded else '로드 안됨'}")
    print(f"터틀봇: {'로드됨' if robot else '로드 안됨'}")
    print(f"\nCtrl+C를 눌러 종료하세요.\n")

    step_count = 0
    current_pos = np.array([0.0, 0.0, 6.13], dtype=float)  # 초기 위치: (0, 0, 6.13)
    current_quat = np.array([1.0, 0.0, 0.0, 0.0])  # 시작 쿼터니언 identity [w, x, y, z] (Isaac Sim 형식)
    target_pos = None
    target_quat = None
    interpolation_steps = 0
    interpolation_frames = 10  # 60프레임 동안 부드럽게 이동 (약 1초 @ 60fps)

    # 토픽 값 버퍼 (3개 단위로 추가/제거하는 슬라이딩 윈도우)
    position_buffer = []
    orientation_buffer = []
    buffer_size = 10 # 최대 10개
    window_add_count = 0  # 3개씩 모으기 위한 카운터

    print("\n⏳ 첫 번째 ROS 토픽을 기다리는 중...")
    print("   로봇의 초기 위치는 토픽에서 받은 첫 메시지로 설정됩니다.\n")

    # 초기 위치 (0, 0, 6)를 먼저 로봇에 설정
    if robot:
        robot.set_world_pose(position=current_pos, orientation=current_quat)
        print(f"✓ 대기 위치 설정: pos=({current_pos[0]:.2f}, {current_pos[1]:.2f}, {current_pos[2]:.2f})")

    try:
        while simulation_app.is_running():
            # ROS 메시지 처리 (non-blocking)
            if ros_node:
                try:
                    rclpy.spin_once(ros_node, timeout_sec=0.0)
                except:
                    pass

            world.step(render=True)

            # ROS로부터 받은 새로운 목표 위치와 방향
            global target_position, target_orientation, initial_position_received
            if target_position is not None and robot:
                # 첫 번째 메시지를 받으면 초기 위치로 설정
                if not initial_position_received:
                    current_pos = np.array(target_position, dtype=float)
                    current_pos[0] *= 100.0  # X값 100배
                    current_pos[1] *= 100.0  # Y값 100배
                    current_pos[2] = 6.13  # Z값 고정
                    if target_orientation:
                        current_quat = convert_quaternion_ros_to_isaac_sim(target_orientation)
                    else:
                        current_quat = np.array([1.0, 0.0, 0.0, 0.0])

                    print(f"✓ 초기 위치 설정: pos=({current_pos[0]:.2f}, {current_pos[1]:.2f}, {current_pos[2]:.2f}) (x,y 100배, z=6.13 고정)")
                    robot.set_world_pose(position=current_pos, orientation=current_quat)
                    initial_position_received = True
                    target_position = None
                    target_orientation = None
                    continue

                # 토픽 값을 버퍼에 추가 (슬라이딩 윈도우)
                pos = np.array(target_position, dtype=float)
                pos[0] *= 100.0  # X값 100배
                pos[1] *= 100.0  # Y값 100배
                pos[2] = 6.13  # Z값 고정
                position_buffer.append(pos)

                if target_orientation:
                    quat = convert_quaternion_ros_to_isaac_sim(target_orientation)
                else:
                    quat = np.array([1.0, 0.0, 0.0, 0.0])
                orientation_buffer.append(quat)

                target_position = None  # 초기화
                target_orientation = None
                window_add_count += 1

                # 3개씩 묶어서 처리: 3개가 모이면 슬라이딩 윈도우에서 3개 제거
                if window_add_count >= 3:
                    # 버퍼 크기 초과 시 가장 오래된 값 3개 제거
                    if len(position_buffer) > buffer_size:
                        position_buffer = position_buffer[3:]
                        orientation_buffer = orientation_buffer[3:]
                    window_add_count = 0

                # 버퍼에 충분한 데이터가 있으면 실시간으로 평균값 계산 및 목표 설정
                if len(position_buffer) >= 3:
                    target_pos = np.mean(position_buffer, axis=0)
                    target_quat = np.mean(orientation_buffer, axis=0)
                    target_quat = target_quat / np.linalg.norm(target_quat)  # 정규화

                    interpolation_steps = 0
                    print(f"[Step {step_count}] 슬라이딩 윈도우 평균 목표: pos=({target_pos[0]:.2f}, {target_pos[1]:.2f}, {target_pos[2]:.2f}), 버퍼={len(position_buffer)}")

            # 부드러운 선형 보간 이동 및 회전
            if target_pos is not None and robot:
                if interpolation_steps < interpolation_frames:
                    # 위치 선형 보간 (Linear interpolation)
                    t = interpolation_steps / interpolation_frames
                    new_pos = current_pos + (target_pos - current_pos) * t

                    # 쿼터니언 구면 선형 보간 (SLERP)
                    new_quat = slerp(current_quat, target_quat, t)

                    try:
                        robot.set_world_pose(
                            position=new_pos,
                            orientation=new_quat
                        )
                    except Exception as e:
                        print(f"⚠️  로봇 위치 설정 오류: {e}")

                    interpolation_steps += 1
                else:
                    # 보간 완료, 최종 위치와 방향 설정
                    current_pos = target_pos.copy()
                    current_quat = target_quat.copy()
                    try:
                        robot.set_world_pose(
                            position=current_pos,
                            orientation=current_quat
                        )
                    except Exception as e:
                        print(f"⚠️  로봇 최종 위치 설정 오류: {e}")
                    print(f"[Step {step_count}] ✓ 목표 도달: pos=({current_pos[0]:.2f}, {current_pos[1]:.2f}, {current_pos[2]:.2f})")
                    target_pos = None
                    target_quat = None

            # 로봇 위치 출력 (매 500 스텝마다)
            if step_count % 500 == 0 and step_count > 0 and robot:
                try:
                    pos, rot = robot.get_world_pose()
                    print(f"[Step {step_count}] 로봇 현재 위치: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")
                except:
                    pass

            step_count += 1

    except KeyboardInterrupt:
        print("\n\n시뮬레이션 중지됨 (Ctrl+C)")

    # 정리
    print("\n" + "="*60)
    print("Isaac Sim 종료 중...")
    print("="*60)

    if ros_node:
        ros_node.destroy_node()
        rclpy.shutdown()
        print("✓ ROS 노드 종료됨")

    simulation_app.close()
    print("✓ 종료됨\n")


if __name__ == "__main__":
    main()
