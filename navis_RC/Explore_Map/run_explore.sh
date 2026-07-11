#!/bin/bash
# run_explore.sh - run_explore.py 실행 도우미 (Windows .bat 파일 대신, 라즈베리파이/Linux용)
# 사용법: ./run_explore.sh [map_out_dir]
# 예:     ./run_explore.sh ./map_output

cd "$(dirname "$0")" || exit 1

OUT_DIR="${1:-./map_output}"

python3 run_explore.py --out "$OUT_DIR"
