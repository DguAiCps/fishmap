#!/bin/bash
# ROS 2에서 위치와 회전각도를 함께 조정하면서 /target_pose 토픽 발행

# 초기 위치
x=23.0
y=-170.0
z=6.0

# 초기 회전 (도 단위, roll만 사용 - 로봇 기준 회전)
roll=180.0

# Roll 각도(도)를 쿼터니언으로 변환하는 함수 (X축 회전만)
roll_to_quaternion() {
    local roll_deg=$1

    # 도를 라디안으로 변환
    local roll=$(echo "scale=6; $roll_deg * 3.14159265359 / 180" | bc)

    # Roll만 사용한 쿼터니언 계산 (X축 회전)
    local cr=$(echo "scale=6; c($roll/2)" | bc -l)
    local sr=$(echo "scale=6; s($roll/2)" | bc -l)

    # Roll만 회전할 때: qx=sin(roll/2), qy=0, qz=0, qw=cos(roll/2)
    local qx=$(echo "scale=6; $sr" | bc -l)
    local qy=0.0
    local qz=0.0
    local qw=$(echo "scale=6; $cr" | bc -l)

    echo "$qx $qy $qz $qw"
}

echo "============================================================"
echo "Moving Target Publisher (위치 + Roll 회전) (ROS 2 Topic)"
echo "============================================================"
echo "시작 위치: x=$x, y=$y, z=$z"
echo "시작 회전: roll=$roll (로봇 회전)"
echo ""
echo "매개변수: --x 증가량, --y 증가량, --z 증가량, --roll 증가량"
echo "예: ./publish_moving_target.sh --x 5.0 --roll 10.0"
echo "Ctrl+C를 눌러 종료하세요"
echo ""

# 증가량 파라미터 처리
dx=5.0
dy=0.0
dz=0.0
droll=0.0

while [[ $# -gt 0 ]]; do
    case $1 in
        --x) dx=$2; shift 2 ;;
        --y) dy=$2; shift 2 ;;
        --z) dz=$2; shift 2 ;;
        --roll) droll=$2; shift 2 ;;
        *) echo "알 수 없는 옵션: $1"; shift ;;
    esac
done

while true; do
    # Roll 각도를 쿼터니언으로 변환
    read -r qx qy qz qw <<< "$(roll_to_quaternion $roll)"

    # ROS 토픽 발행
    ros2 topic pub -1 /target_pose geometry_msgs/msg/PoseStamped \
        "{header: {frame_id: 'map'}, pose: {position: {x: $x, y: $y, z: $z}, orientation: {x: $qx, y: $qy, z: $qz, w: $qw}}}" \
        > /dev/null 2>&1

    echo "발행: pos=($x, $y, $z) | roll=$roll"

    # 값 증가
    x=$(echo "scale=2; $x + $dx" | bc)
    y=$(echo "scale=2; $y + $dy" | bc)
    z=$(echo "scale=2; $z + $dz" | bc)
    roll=$(echo "scale=2; $roll + $droll" | bc)

    # Roll 값이 360도 이상이면 0으로 초기화
    roll=$(echo "if ($roll >= 360) 0 else if ($roll <= -360) 0 else $roll" | bc)

    # 매우 짧은 대기 (빠른 업데이트)
    sleep 0.0001
done
