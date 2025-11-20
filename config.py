"""
Isaac Sim 설정 파일
"""

import os

PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))

# 맵 파일 (637.usdc 또는 637.obj)
MAP_FILE = os.path.join(PROJECT_ROOT, "637.usdc")

# 터틀봇 USD 파일
TURTLEBOT_FILE = os.path.join(PROJECT_ROOT, "create_3.usd")

# 터틀봇 시작 위치
TURTLEBOT_START_POS = [0.0, -0.0, 6.13]
