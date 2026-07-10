"""
run_all.py
===========
Camera(차선인식), Drive/Steer, Drive/Throttle, Drive/OutInterface(STM32 CAN 출력),
Drive/OutInterface(수동 비상정지 킬스위치), Path_Planning 여섯 프로세스를 한 번에 띄우고
감시하는 상위 실행 스크립트.

원래 .bat 파일로 만들 생각이었는데, 라즈베리파이는 Linux(Ubuntu)라 Windows .bat 파일은
애초에 실행이 안 됩니다. 대신 파이썬으로 만들면 argparse로 목표 좌표 같은 입력도 그대로
받을 수 있고, 자식 프로세스 중 하나가 죽었을 때 나머지도 안전하게 정지시키는 감시 로직도
넣기 쉬워서 이 방식을 골랐습니다. 터미널 진입 장벽이 걱정되면 같이 만들어둔 run_all.sh 를
더블클릭(또는 ./run_all.sh)해서 실행해도 됩니다 - 내부에서 이 스크립트를 호출합니다.

[동작]
- 6개 프로세스를 모두 실행 (kill_switch는 이 터미널의 키보드 입력을 그대로 받습니다 -
  아무 키나 누르면 즉시 비상정지, 다시 누르면 해제)
- 1초마다 살아있는지 확인
- 하나라도 예기치 않게(자기가 알아서 끝난 게 아니라) 죽으면, 나머지도 전부 SIGINT를 보내
  안전하게 정지시키고 종료 (Steer/Throttle은 SIGINT를 받아야 상태 파일에 중립/정지 값을
  마지막으로 기록하고, Ras_output은 SIGINT를 받아야 STM32에 정지 CAN 명령을 보내고 종료함).
  kill_switch가 죽는 경우(터미널을 닫는 등)도 이 경로로 잡혀서 전체가 안전 종료됩니다.
- Ctrl+C 를 누르면 6개 전부 같은 방식으로 안전 종료

[사용법]
    python3 run_all.py --goal 2000,1500
    (맵 폴더는 기본값 ./Build_Map/map_output 를 씀. 다른 맵을 쓰려면 --map_dir 로 지정)
"""

import argparse
import os
import signal
import subprocess
import sys
import time

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
CAMERA_SCRIPT = os.path.join(THIS_DIR, "Camera", "lane_center_detection.py")
STEER_SCRIPT = os.path.join(THIS_DIR, "Drive", "Steer", "steer_control.py")
THROTTLE_SCRIPT = os.path.join(THIS_DIR, "Drive", "Throttle", "throttle_control.py")
RAS_OUTPUT_SCRIPT = os.path.join(THIS_DIR, "Drive", "OutInterface", "Ras_output.py")
KILL_SWITCH_SCRIPT = os.path.join(THIS_DIR, "Drive", "OutInterface", "kill_switch.py")
PATH_PLANNING_SCRIPT = os.path.join(THIS_DIR, "Path_Planning", "localize_and_plan.py")
DEFAULT_MAP_DIR = os.path.join(THIS_DIR, "Build_Map", "map_output")

CHECK_PERIOD_S = 1.0
SHUTDOWN_TIMEOUT_S = 5.0


def start(name, script_path, extra_args=None):
    cmd = [sys.executable, script_path] + (extra_args or [])
    print(f"[run_all] 시작: {name} ({' '.join(cmd)})")
    return subprocess.Popen(cmd)


def stop_all(procs):
    for name, p in procs.items():
        if p.poll() is None:
            print(f"[run_all] 종료 신호 전송: {name}")
            p.send_signal(signal.SIGINT)

    deadline = time.time() + SHUTDOWN_TIMEOUT_S
    for name, p in procs.items():
        remaining = max(0.0, deadline - time.time())
        try:
            p.wait(timeout=remaining)
        except subprocess.TimeoutExpired:
            print(f"[run_all] {name} 가 제때 안 죽어서 강제 종료(kill)합니다.")
            p.kill()


def find_dead_process(procs):
    for name, p in procs.items():
        if p.poll() is not None:
            return name
    return None


def main():
    parser = argparse.ArgumentParser(description="Camera/Steer/Throttle/Ras_output/Path_Planning을 한번에 실행하고 감시")
    parser.add_argument("--map_dir", default=DEFAULT_MAP_DIR,
                         help=f"Path_Planning 에 넘길 --map_dir (기본값: {DEFAULT_MAP_DIR})")
    parser.add_argument("--goal", required=True, help="Path_Planning 에 넘길 --goal 'x_mm,y_mm'")
    parser.add_argument("--planner", default="astar", choices=["astar", "rrt"])
    args = parser.parse_args()

    procs = {
        "camera": start("camera", CAMERA_SCRIPT),
        "steer": start("steer", STEER_SCRIPT),
        "throttle": start("throttle", THROTTLE_SCRIPT),
        "ras_output": start("ras_output", RAS_OUTPUT_SCRIPT),
        "kill_switch": start("kill_switch", KILL_SWITCH_SCRIPT),
        "path_planning": start("path_planning", PATH_PLANNING_SCRIPT, [
            "--map_dir", args.map_dir, "--goal", args.goal, "--planner", args.planner,
        ]),
    }

    print("[run_all] 6개 프로세스 실행 중 - 아무 키나 누르면 즉시 비상정지 (Ctrl+C 로 전체 종료)")
    try:
        while True:
            dead_name = find_dead_process(procs)
            if dead_name is not None:
                dead_proc = procs.pop(dead_name)
                print(f"[run_all] '{dead_name}' 프로세스가 예기치 않게 종료됐습니다 "
                      f"(exit code {dead_proc.returncode}). 안전을 위해 나머지 프로세스도 전부 종료합니다.")
                stop_all(procs)
                sys.exit(1)
            time.sleep(CHECK_PERIOD_S)
    except KeyboardInterrupt:
        print("\n[run_all] 종료 요청 받음 - 전체 프로세스를 정지합니다.")
        stop_all(procs)


if __name__ == "__main__":
    main()
