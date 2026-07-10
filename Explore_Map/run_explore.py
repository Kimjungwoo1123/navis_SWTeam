"""
run_explore.py
================
Explore_Map(자율 탐색+매핑) 단계 전용 상위 실행 스크립트.
run_all.py 와 로직은 동일하지만(감시/안전종료), 띄우는 프로세스 구성이 다릅니다:

- explore_and_map.py, Drive/Steer/steer_control.py, Drive/Throttle/throttle_control.py,
  Drive/OutInterface/Ras_output.py, Drive/OutInterface/kill_switch.py 만 실행
- Camera는 켜지 않습니다. 실내 매핑 단계라 차선이 없고, Drive/Steer는 camera_steering.json이
  없거나 오래되면 자동으로 lidar_steering.json(explore_and_map.py가 씀)만 보고 동작하도록
  이미 만들어져 있어서 그대로 재사용됩니다.
- Path_Planning도 실행하지 않습니다 (아직 목표지점까지 갈 맵 자체가 없는 단계이므로).

[사용법]
    python3 run_explore.py --out ./map_output
"""

import argparse
import os
import signal
import subprocess
import sys
import time

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
TESTING_DIR = os.path.dirname(THIS_DIR)
EXPLORE_SCRIPT = os.path.join(THIS_DIR, "explore_and_map.py")
STEER_SCRIPT = os.path.join(TESTING_DIR, "Drive", "Steer", "steer_control.py")
THROTTLE_SCRIPT = os.path.join(TESTING_DIR, "Drive", "Throttle", "throttle_control.py")
RAS_OUTPUT_SCRIPT = os.path.join(TESTING_DIR, "Drive", "OutInterface", "Ras_output.py")
KILL_SWITCH_SCRIPT = os.path.join(TESTING_DIR, "Drive", "OutInterface", "kill_switch.py")

CHECK_PERIOD_S = 1.0
SHUTDOWN_TIMEOUT_S = 5.0


def start(name, script_path, extra_args=None):
    cmd = [sys.executable, script_path] + (extra_args or [])
    print(f"[run_explore] 시작: {name} ({' '.join(cmd)})")
    return subprocess.Popen(cmd)


def stop_all(procs):
    for name, p in procs.items():
        if p.poll() is None:
            print(f"[run_explore] 종료 신호 전송: {name}")
            p.send_signal(signal.SIGINT)

    deadline = time.time() + SHUTDOWN_TIMEOUT_S
    for name, p in procs.items():
        remaining = max(0.0, deadline - time.time())
        try:
            p.wait(timeout=remaining)
        except subprocess.TimeoutExpired:
            print(f"[run_explore] {name} 가 제때 안 죽어서 강제 종료(kill)합니다.")
            p.kill()


def find_dead_process(procs):
    for name, p in procs.items():
        if p.poll() is not None:
            return name
    return None


def main():
    parser = argparse.ArgumentParser(description="Explore_Map+Steer+Throttle+Ras_output을 한번에 실행하고 감시")
    parser.add_argument("--out", default="./map_output", help="explore_and_map.py 에 넘길 --out (완성된 맵 저장 폴더)")
    parser.add_argument("--max_cycles", type=int, default=None, help="explore_and_map.py 에 넘길 --max_cycles (생략시 기본값 사용)")
    args = parser.parse_args()

    explore_args = ["--out", args.out]
    if args.max_cycles is not None:
        explore_args += ["--max_cycles", str(args.max_cycles)]

    procs = {
        "explore_and_map": start("explore_and_map", EXPLORE_SCRIPT, explore_args),
        "steer": start("steer", STEER_SCRIPT),
        "throttle": start("throttle", THROTTLE_SCRIPT),
        "ras_output": start("ras_output", RAS_OUTPUT_SCRIPT),
        "kill_switch": start("kill_switch", KILL_SWITCH_SCRIPT),
    }

    print("[run_explore] 5개 프로세스 실행 중 - 아무 키나 누르면 즉시 비상정지 - "
          "Camera/Path_Planning은 이 단계에선 켜지 않음 (Ctrl+C 로 전체 종료)")
    try:
        while True:
            dead_name = find_dead_process(procs)
            if dead_name is not None:
                dead_proc = procs.pop(dead_name)
                print(f"[run_explore] '{dead_name}' 프로세스가 예기치 않게 종료됐습니다 "
                      f"(exit code {dead_proc.returncode}). 안전을 위해 나머지 프로세스도 전부 종료합니다.")
                stop_all(procs)
                sys.exit(1)
            time.sleep(CHECK_PERIOD_S)
    except KeyboardInterrupt:
        print("\n[run_explore] 종료 요청 받음 - 전체 프로세스를 정지합니다.")
        stop_all(procs)


if __name__ == "__main__":
    main()
