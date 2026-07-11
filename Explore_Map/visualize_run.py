"""
visualize_run.py
==================
explore_and_map.py가 만든 --out 폴더(맵 + command_log.csv)를 읽어서 한 장으로 보여주는
오프라인 분석 도구. 하드웨어와 무관하게(라이다/CAN 연결 없이) PC에서 그냥 돌리면 됨.

- 왼쪽: 점유 격자 위에 실제 이동 궤적을 겹쳐 그림 (사이클이 지날수록 밝은 색)
- 오른쪽 위: 사이클별 조향각(+ = 좌회전, - = 우회전) - 이게 계속 한쪽 부호로 쏠려있으면
  소프트웨어(조향 계산)가 원인, 0 근방인데도 궤적이 한쪽으로 휘면 구동축/얼라인먼트 같은
  기구적 문제로 좁혀진다.
- 오른쪽 아래: 사이클별 speed_percent

[사용법]
    python3 visualize_run.py --map_dir ./map_output
    python3 visualize_run.py --map_dir ./map_output --out run.png   # 저장 경로 직접 지정
"""

import argparse
import os

import numpy as np
import pandas as pd
import matplotlib

matplotlib.use("Agg")  # 이 스크립트는 파일로 저장만 하지 창을 띄우지 않으므로, 인터랙티브
# 백엔드(Tk 등)가 이 PC에 없거나 깨져있어도 상관없이 항상 되게 고정
import matplotlib.pyplot as plt


def load_run(map_dir):
    grid = np.load(os.path.join(map_dir, "map_occupancy_grid.npy"))
    meta = pd.read_csv(os.path.join(map_dir, "map_grid_meta.csv")).iloc[0]
    origin = (meta["origin_x_mm"], meta["origin_y_mm"])
    resolution = meta["resolution_mm"]
    log_path = os.path.join(map_dir, "command_log.csv")
    log = pd.read_csv(log_path) if os.path.exists(log_path) else None
    return grid, origin, resolution, log


def plot_run(map_dir, out_path=None):
    grid, origin, resolution, log = load_run(map_dir)
    h, w = grid.shape
    extent = [origin[0], origin[0] + w * resolution, origin[1] + h * resolution, origin[1]]
    out_path = out_path or os.path.join(map_dir, "run_visualization.png")

    if log is None or log.empty:
        fig, ax = plt.subplots(figsize=(8, 8))
        ax.imshow(grid, cmap="gray_r", extent=extent, origin="upper")
        ax.set_title("점유 격자 (command_log.csv 없음 - 궤적/조향 표시 불가)")
        fig.savefig(out_path, dpi=150)
        print(f"command_log.csv가 없어서 격자만 그렸습니다: {out_path}")
        return

    fig = plt.figure(figsize=(14, 8))
    gs = fig.add_gridspec(2, 2, width_ratios=[1.3, 1])
    ax_map = fig.add_subplot(gs[:, 0])
    ax_steer = fig.add_subplot(gs[0, 1])
    ax_speed = fig.add_subplot(gs[1, 1])

    # --- 왼쪽: 점유 격자 + 실제 궤적 (사이클이 지날수록 밝은 색) ---
    ax_map.imshow(grid, cmap="gray_r", extent=extent, origin="upper")
    moved = log.dropna(subset=["x_mm", "y_mm"])
    if len(moved):
        sc = ax_map.scatter(moved["x_mm"], moved["y_mm"], c=moved["cycle"], cmap="viridis", s=14, zorder=3)
        ax_map.plot(moved["x_mm"], moved["y_mm"], "-", color="dodgerblue", linewidth=1, alpha=0.5, zorder=2)
        fig.colorbar(sc, ax=ax_map, label="cycle", shrink=0.6)
    ax_map.plot(0, 0, "s", color="orange", markersize=8, label="원점", zorder=4)
    ax_map.set_title("점유 격자 + 실제 이동 궤적 (밝을수록 나중 사이클)")
    ax_map.set_aspect("equal")
    ax_map.legend(loc="upper right")

    # --- 오른쪽 위: 조향각 ---
    steer = log.dropna(subset=["steering_deg"])
    ax_steer.axhline(0, color="gray", linewidth=1)
    if len(steer):
        ax_steer.plot(steer["cycle"], steer["steering_deg"], "-o", color="crimson", markersize=3)
    ax_steer.set_title("사이클별 조향각 (+ = 좌회전, - = 우회전)")
    ax_steer.set_xlabel("cycle")
    ax_steer.set_ylabel("steering_deg")

    # --- 오른쪽 아래: 속도 ---
    ax_speed.plot(log["cycle"], log["speed_percent"], "-o", color="seagreen", markersize=3)
    ax_speed.set_title("사이클별 speed_percent")
    ax_speed.set_xlabel("cycle")
    ax_speed.set_ylabel("speed_percent")

    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"저장 완료: {out_path}")

    if len(steer):
        mean_steer = steer["steering_deg"].mean()
        side = "좌" if mean_steer > 0 else "우"
        print(f"조향각 평균: {mean_steer:+.2f}deg -> 평균적으로 {side}회전 쪽으로 명령이 쏠려있음"
              f"{'(직진 명령이면 소프트웨어 원인 의심)' if abs(mean_steer) > 1.0 else ''}")
    else:
        print("조향 기록이 없습니다 (계속 정지 상태였거나 로그가 비어있음).")


def main():
    parser = argparse.ArgumentParser(description="map_output 폴더의 맵+명령기록(command_log.csv)을 시각화")
    parser.add_argument("--map_dir", default="./map_output", help="explore_and_map.py --out 폴더 (기본 ./map_output)")
    parser.add_argument("--out", default=None, help="저장할 png 경로 (기본: map_dir/run_visualization.png)")
    args = parser.parse_args()
    plot_run(args.map_dir, args.out)


if __name__ == "__main__":
    main()
