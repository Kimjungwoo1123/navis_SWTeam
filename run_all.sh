#!/bin/bash
# run_all.sh - run_all.py 실행 도우미 (Windows .bat 파일 대신, 라즈베리파이/Linux용)
# 사용법: ./run_all.sh <map_dir> <goal_x,goal_y> [planner]
# 예:     ./run_all.sh ./Build_Map/map_output 2000,1500

cd "$(dirname "$0")" || exit 1

MAP_DIR="${1:?맵 폴더 경로를 입력하세요 (예: ./Build_Map/map_output)}"
GOAL="${2:?목표 좌표를 입력하세요 (예: 2000,1500)}"
PLANNER="${3:-astar}"

python3 run_all.py --map_dir "$MAP_DIR" --goal "$GOAL" --planner "$PLANNER"
