#!/bin/bash
# run_all.sh - run_all.py 실행 도우미 (Windows .bat 파일 대신, 라즈베리파이/Linux용)
# 맵은 기본값(./Build_Map/map_output)을 쓰므로 목표 좌표만 입력하면 됨.
# 사용법: ./run_all.sh <goal_x,goal_y> [planner]
# 예:     ./run_all.sh 1000,1000
#         ./run_all.sh 1000,1000 rrt
#
# 다른 맵 폴더를 쓰고 싶으면 이 스크립트 대신 직접 실행:
#     python3 run_all.py --map_dir <다른 폴더> --goal <x,y>

cd "$(dirname "$0")" || exit 1

GOAL="${1:?목표 좌표를 입력하세요 (예: 1000,1000)}"
PLANNER="${2:-astar}"

python3 run_all.py --goal "$GOAL" --planner "$PLANNER"
