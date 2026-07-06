"""
build_map.py
=============
여러 위치에서 찍은 LD06 라이다 스캔(CSV: angle, distance)을
ICP(Iterative Closest Point)로 정합해서 하나의 통합 맵으로 만드는 프로그램.

[사용법]
    python3 build_map.py scan_pos1.csv scan_pos2.csv scan_pos3.csv scan_pos4.csv --out map_output

[동작 방식 - Multi-start ICP]
    스캔 위치를 옮길 때 "좌/우/앞/뒤"가 사람이 보는 기준인지 라이다 0도(화살표) 기준인지
    헷갈리기 쉽습니다. 이 프로그램은 사람이 입력한 --init 값을 정확한 정답으로 믿지 않고,
    축이 뒤바뀌었거나 부호가 반대인 경우까지 포함한 여러 후보 + 다양한 회전각을 모두
    자동으로 시도해서 가장 정합이 잘 되는 조합을 채택합니다.
    따라서 --init 은 "대략의 힌트"일 뿐이며 생략해도 동작합니다(없으면 원점 근방만 탐색).

    --init 은 pos2, pos3, pos4 ... 순서대로 "pos1 기준 대략적 누적 이동량"입니다.
    형식: dx_mm,dy_mm,dtheta_deg  (회전 없이 평행이동만 했다면 dtheta는 0)

    예시:
    python3 build_map.py scan_pos1.csv scan_pos2.csv scan_pos3.csv scan_pos4.csv \
        --init=1200,0,0 --init=1200,1000,0 --init=0,1000,0 --out map_output

    (음수가 들어가면 --init=-900,300,0 처럼 = 기호로 붙여 써야 argparse 오류가 안 납니다)

[출력]
    map_points.csv          - 병합된 전체 포인트 (x_mm, y_mm), 다운샘플 아닌 원본 전체
    map_poses.csv           - 각 스캔 위치의 채택된 pose 및 정합 오차 (디버깅/검증용)
    map_occupancy_grid.npy  - occupancy grid (다음 단계인 localize_and_plan.py 에서 사용)
    map_grid_meta.csv       - grid 좌표 변환 메타데이터
    map_result.png          - 시각화 이미지
"""

import sys
import os
import argparse
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree

# ----------------------------
# 설정값 (필요시 조정)
# ----------------------------
ANGLE_UNIT = "deg"        # CSV의 angle 컬럼 단위: "deg" 또는 "rad"
DIST_UNIT = "mm"          # CSV의 distance 컬럼 단위: "mm" 또는 "m"
MIN_RANGE_MM = 150        # 이 거리 이내는 노이즈로 간주(거치대 등) -> 제거
MAX_RANGE_MM = 8000       # 이 거리 이상은 오류값으로 간주 -> 제거

ICP_MAX_ITER = 60
ICP_TOLERANCE = 1e-5
ICP_MAX_CORRESPONDENCE_DIST = 300.0  # mm, 이보다 먼 점쌍은 매칭에서 제외(아웃라이어 방지)

GRID_RESOLUTION_MM = 50   # occupancy grid 한 칸 크기


# ----------------------------
# CSV 로드 + 극좌표 -> 직교좌표 변환
# ----------------------------
def load_scan_as_xy(csv_path):
    df = pd.read_csv(csv_path)

    angle_col, dist_col = None, None
    for c in df.columns:
        cl = c.strip().lower()
        if cl in ("angle", "angle_deg", "angle_rad", "theta"):
            angle_col = c
        if cl in ("distance", "dist", "range", "distance_mm", "range_mm"):
            dist_col = c
    if angle_col is None or dist_col is None:
        raise ValueError(f"[{csv_path}] angle/distance 컬럼을 못 찾았어요. 실제 컬럼: {list(df.columns)}")

    angle = df[angle_col].to_numpy(dtype=float)
    dist = df[dist_col].to_numpy(dtype=float)

    angle_rad = np.deg2rad(angle) if ANGLE_UNIT == "deg" else angle
    dist_mm = dist * 1000.0 if DIST_UNIT == "m" else dist

    valid = (dist_mm > MIN_RANGE_MM) & (dist_mm < MAX_RANGE_MM)
    angle_rad, dist_mm = angle_rad[valid], dist_mm[valid]

    x = dist_mm * np.cos(angle_rad)
    y = dist_mm * np.sin(angle_rad)
    pts = np.stack([x, y], axis=1)

    print(f"  - {os.path.basename(csv_path)}: 원본 {len(df)}점 -> 유효 {len(pts)}점")
    return pts


# ----------------------------
# ICP (point-to-point, SVD 기반)
# ----------------------------
def best_fit_transform(A, B):
    """A를 B에 맞추는 최적 회전 R, 이동 t (Nx2, 대응관계 맞춰진 상태)"""
    centroid_A, centroid_B = np.mean(A, axis=0), np.mean(B, axis=0)
    AA, BB = A - centroid_A, B - centroid_B
    H = AA.T @ BB
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    t = centroid_B - R @ centroid_A
    return R, t


def icp(source, target, max_iterations=ICP_MAX_ITER, tolerance=ICP_TOLERANCE,
        max_corr_dist=ICP_MAX_CORRESPONDENCE_DIST, init_guess=None, verbose=True):
    """
    source 점군을 target 점군에 정합.
    init_guess: (dx, dy, dtheta_deg) - 대략적인 초기 이동/회전 추정치.
    반환: 정합된 source 좌표, 누적 R(2x2), 누적 t(2,), 평균오차, coverage(매칭된 점 비율)
    """
    src = source.copy()
    R_total, t_total = np.eye(2), np.zeros(2)

    if init_guess is not None:
        dx, dy, dtheta_deg = init_guess
        th = np.deg2rad(dtheta_deg)
        R0 = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
        t0 = np.array([dx, dy])
        src = (R0 @ src.T).T + t0
        R_total, t_total = R0, t0

    tree = cKDTree(target)
    prev_error, mean_error = None, None
    coverage = 0.0

    for i in range(max_iterations):
        distances, indices = tree.query(src)
        mask = distances < max_corr_dist
        coverage = mask.sum() / len(src)
        if mask.sum() < 10:
            if verbose:
                print(f"    [경고] 유효 대응점이 {mask.sum()}개뿐 - 정합 신뢰도 낮음")
            return src, R_total, t_total, 1e9, 0.0

        R, t = best_fit_transform(src[mask], target[indices[mask]])
        src = (R @ src.T).T + t
        R_total = R @ R_total
        t_total = R @ t_total + t

        mean_error = np.mean(distances[mask])
        if prev_error is not None and abs(prev_error - mean_error) < tolerance:
            break
        prev_error = mean_error

    return src, R_total, t_total, mean_error, coverage


def icp_multistart(source, target, base_guess=None, max_iterations=ICP_MAX_ITER,
                    tolerance=ICP_TOLERANCE, max_corr_dist=ICP_MAX_CORRESPONDENCE_DIST,
                    angle_step=15):
    """
    좌표축(좌/우/앞/뒤)이 헷갈렸을 가능성까지 고려해서 여러 후보로 ICP를 시도하고
    가장 그럴듯한 결과를 채택.

    base_guess: (dx, dy, dtheta_deg) - 사람이 입력한 대략적 힌트.
    angle_step: 0~360도를 이 간격으로 끊어서 회전 후보 생성 (기본 15도 -> 24개 후보)

    채택 기준은 단순 최소오차가 아니라 다음 두 가지를 함께 고려:
    1) coverage(매칭된 점 비율)가 충분히 높아야 함 (일부만 우연히 들어맞는 가짜 매칭 배제)
    2) 같은 수준이면 힌트(base_guess) 위치에 가까운 후보를 우선
    빈 직사각형 방처럼 벽만 있는 경우, 엉뚱한 위치/회전이 낮은 오차를 내는 함정에
    빠지기 쉬운데, 이 기준으로 그런 가짜 정답을 걸러냄.
    """
def icp_multistart(source, target, base_guess=None, max_iterations=ICP_MAX_ITER,
                    tolerance=ICP_TOLERANCE, max_corr_dist=ICP_MAX_CORRESPONDENCE_DIST,
                    angle_step=15):
    """
    좌표축(좌/우/앞/뒤)이 헷갈렸을 가능성까지 고려해서 여러 후보로 ICP를 시도하고
    가장 그럴듯한 결과를 채택.

    base_guess: (dx, dy, dtheta_deg) - 사람이 입력한 대략적 힌트.
    angle_step: 0~360도를 이 간격으로 끊어서 회전 후보 생성 (기본 15도 -> 24개 후보)

    속도를 위해 2단계 funnel 구조로 동작:
    1) 스크리닝: 모든 후보(위치 x 각도)를 짧은 반복(5회)으로 빠르게 평가
    2) 정밀화: 스크리닝 상위 후보 몇 개만 골라 max_iterations까지 충분히 반복

    채택 기준은 단순 최소오차가 아니라:
    - coverage(매칭된 점 비율)가 충분히 높아야 함 (일부만 우연히 들어맞는 가짜 매칭 배제)
    - 같은 수준이면 힌트(base_guess) 위치에 가까운 후보를 우선
    """
    if base_guess is None:
        base_guess = (0, 0, 0)
    bx, by, _ = base_guess

    # 힌트 주변 위치 후보 (성기게) + 축 혼동/부호반전 패턴 (보조)
    offsets = [-600, -300, 0, 300, 600]
    pos_candidates = [(bx + ox, by + oy) for ox in offsets for oy in offsets]
    backup_candidates = list({
        (by, bx), (-bx, by), (bx, -by), (-bx, -by),
        (-by, bx), (by, -bx), (-by, -bx), (0, 0),
    })

    MIN_COVERAGE = 0.6
    SCREEN_ITER = 5     # 1단계 스크리닝용 짧은 반복 횟수
    TOP_K = 6            # 정밀화로 넘길 상위 후보 개수

    def screen(pos_list, angle_range):
        scored = []
        for px, py in pos_list:
            for theta0 in angle_range:
                _, R, t, err, cov = icp(source, target, max_iterations=SCREEN_ITER,
                                         tolerance=tolerance, max_corr_dist=max_corr_dist,
                                         init_guess=(px, py, theta0), verbose=False)
                scored.append((err, cov, R, t, px, py, theta0))
        return scored

    def refine(candidates):
        results = []
        for _, _, _, _, px, py, theta0 in candidates:
            _, R, t, err, cov = icp(source, target, max_iterations=max_iterations,
                                     tolerance=tolerance, max_corr_dist=max_corr_dist,
                                     init_guess=(px, py, theta0), verbose=False)
            dist_from_hint = np.hypot(t[0] - bx, t[1] - by)
            results.append((err, cov, dist_from_hint, R, t))
        return results

    print(f"    Multi-start ICP: 1단계 스크리닝 {len(pos_candidates)}개 위치 x {360 // angle_step}개 각도")
    screened = screen(pos_candidates, range(0, 360, angle_step))
    # coverage 우선, 그 다음 오차 기준으로 상위 K개 후보 선정
    screened.sort(key=lambda r: (-r[1], r[0]))
    top_candidates = screened[:TOP_K]

    print(f"    -> 상위 {len(top_candidates)}개 후보 정밀화")
    results = refine(top_candidates)
    good_results = [r for r in results if r[1] >= MIN_COVERAGE]

    if not good_results:
        print(f"    힌트 근처에서 충분히 신뢰할 만한 정합을 못 찾음 -> 축 혼동 패턴까지 확장 탐색")
        screened2 = screen(backup_candidates, range(0, 360, angle_step))
        screened2.sort(key=lambda r: (-r[1], r[0]))
        results += refine(screened2[:TOP_K])
        good_results = [r for r in results if r[1] >= MIN_COVERAGE]

    if good_results:
        # coverage 기준을 만족하는 것들 중, 힌트에 가장 가까운 것을 우선 채택
        # (오차가 비슷한 수준이면 엉뚱한 곳보다 힌트 근처가 실제 정답일 확률이 높음)
        good_results.sort(key=lambda r: (round(r[0] / 10), r[2]))  # 오차(10mm단위로 묶음) -> 힌트와의 거리 순
        best = good_results[0]
    else:
        # coverage 기준을 만족하는 게 하나도 없으면 그냥 오차 최소인 것 채택 (차선책)
        print(f"    [경고] coverage {MIN_COVERAGE*100:.0f}% 이상인 정합을 찾지 못함 - 결과 신뢰도가 낮을 수 있음")
        results.sort(key=lambda r: r[0])
        best = results[0]

    err, cov, dist_from_hint, R, t = best
    print(f"    -> 채택: coverage={cov*100:.0f}%, 힌트와의 거리={dist_from_hint:.0f}mm")
    return None, R, t, err


# ----------------------------
# occupancy grid 생성
# ----------------------------
def points_to_grid(points, resolution=GRID_RESOLUTION_MM, margin=200):
    xs, ys = points[:, 0], points[:, 1]
    min_x, max_x = xs.min() - margin, xs.max() + margin
    min_y, max_y = ys.min() - margin, ys.max() + margin

    width = int((max_x - min_x) / resolution) + 1
    height = int((max_y - min_y) / resolution) + 1
    grid = np.zeros((height, width), dtype=np.uint8)

    col = np.clip(((xs - min_x) / resolution).astype(int), 0, width - 1)
    row = np.clip(((ys - min_y) / resolution).astype(int), 0, height - 1)
    grid[row, col] = 1

    origin = (min_x, min_y)  # grid[0,0] 칸이 의미하는 실제 world 좌표(mm)
    return grid, origin


# ----------------------------
# 메인 파이프라인
# ----------------------------
def downsample(points, max_points=4000):
    """점이 너무 많으면 ICP가 느려지고 점개수 불균형으로 편향이 생기므로 균일 샘플링"""
    if len(points) <= max_points:
        return points
    idx = np.random.choice(len(points), max_points, replace=False)
    return points[idx]


def main(csv_paths, init_guesses=None, out_dir=".", max_points_per_scan=4000):
    if len(csv_paths) < 2:
        print("CSV 파일을 2개 이상 입력해주세요.")
        sys.exit(1)

    if init_guesses is None:
        init_guesses = [(0, 0, 0)] * (len(csv_paths) - 1)

    print("=== 1. 스캔 로드 ===")
    scans = [load_scan_as_xy(p) for p in csv_paths]

    print(f"\n=== 1-1. 다운샘플링 (스캔당 최대 {max_points_per_scan}점) ===")
    scans_full = scans  # 원본은 최종 맵 저장에 사용
    scans_for_icp = [downsample(s, max_points_per_scan) for s in scans]
    for i, (full, ds) in enumerate(zip(scans_full, scans_for_icp)):
        print(f"  - pos{i+1}: {len(full)}점 -> ICP용 {len(ds)}점")

    print("\n=== 2. ICP 1차 순차 정합 (Multi-start, pos1 = world 좌표계 원점(0,0)으로 고정) ===")
    print("    (좌/우/앞/뒤 축 혼동, 회전 등 여러 후보를 자동으로 시도합니다)")
    world_map_icp = scans_for_icp[0].copy()   # 정합용 (다운샘플)
    transforms = [(np.eye(2), np.zeros(2))]

    for i in range(1, len(scans)):
        guess = init_guesses[i - 1]
        print(f"\n  -> pos{i+1} 를 누적 world map에 정합 중... (힌트 dx={guess[0]}, dy={guess[1]}, dtheta={guess[2]}deg)")
        _, R, t, err = icp_multistart(scans_for_icp[i], world_map_icp, base_guess=guess)
        print(f"     pos{i+1} 채택된 변환: t=[{t[0]:.1f}, {t[1]:.1f}], theta={np.degrees(np.arctan2(R[1,0],R[0,0])):.1f}deg, 정합오차={err:.2f}mm")
        if err > 80:
            print(f"     [주의] 정합 오차가 {err:.1f}mm로 큽니다. 이 스캔의 위치 추정이 부정확할 수 있어요.")

        aligned_icp = (R @ scans_for_icp[i].T).T + t
        transforms.append((R, t))
        world_map_icp = np.vstack([world_map_icp, aligned_icp])

    # ----------------------------
    # 2차: 전체맵 기준 재정합 (global refinement)
    # 1차는 직전 스캔까지의 누적맵에만 맞춰서 순서상 뒤쪽 스캔의 정보가
    # 앞쪽 정합 판단에 전혀 반영이 안 됨. 한 스캔이 잘못 붙으면 그 이후로
    # 도미노처럼 계속 잘못 쌓이는 문제가 있어서, 1차 결과로 일단 전체 맵을
    # 만든 뒤 "각 스캔을 자기 자신을 제외한 전체 누적맵"에 다시 정합해서
    # 서로의 정보로 상호 보정함.
    # ----------------------------
    print("\n=== 2-1. 전체맵 기준 재정합 (global refinement) ===")
    REFINE_ROUNDS = 2
    for round_i in range(REFINE_ROUNDS):
        print(f"\n  -- 보정 라운드 {round_i+1}/{REFINE_ROUNDS} --")
        new_transforms = [transforms[0]]  # pos1은 계속 원점 고정
        for i in range(1, len(scans)):
            others = [
                (transforms[j][0] @ scans_for_icp[j].T).T + transforms[j][1]
                for j in range(len(scans)) if j != i
            ]
            other_map = np.vstack(others)
            R_prev, t_prev = transforms[i]
            hint = (t_prev[0], t_prev[1], np.degrees(np.arctan2(R_prev[1, 0], R_prev[0, 0])))
            _, R, t, err = icp_multistart(scans_for_icp[i], other_map, base_guess=hint, angle_step=10)
            print(f"     pos{i+1} 재정합: t=[{t[0]:.1f}, {t[1]:.1f}], "
                  f"theta={np.degrees(np.arctan2(R[1,0],R[0,0])):.1f}deg, 오차={err:.2f}mm")
            new_transforms.append((R, t))
        transforms = new_transforms

    # 최종 변환으로 전체 맵(다운샘플본/원본 전체) 재구성
    world_map_icp = np.vstack([
        (transforms[i][0] @ scans_for_icp[i].T).T + transforms[i][1]
        for i in range(len(scans))
    ])
    world_map_full = np.vstack([
        (transforms[i][0] @ scans_full[i].T).T + transforms[i][1]
        for i in range(len(scans))
    ])

    transformed_scans = []
    for i, s in enumerate(scans_full):
        R, t = transforms[i]
        transformed_scans.append((R @ s.T).T + t)
    world_map = world_map_full

    print("\n=== 3. 결과 저장 ===")
    os.makedirs(out_dir, exist_ok=True)

    merged_csv = os.path.join(out_dir, "map_points.csv")
    pd.DataFrame(world_map, columns=["x_mm", "y_mm"]).to_csv(merged_csv, index=False)
    print(f"  - 병합 포인트 CSV: {merged_csv} ({len(world_map)}점)")

    pose_rows = []
    for i, (R, t) in enumerate(transforms):
        theta = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
        pose_rows.append({"scan": f"pos{i+1}", "x_mm": t[0], "y_mm": t[1], "theta_deg": theta})
    pose_df = pd.DataFrame(pose_rows)
    pose_csv = os.path.join(out_dir, "map_poses.csv")
    pose_df.to_csv(pose_csv, index=False)
    print(f"  - 스캔 위치 pose: {pose_csv}")
    print(pose_df.to_string(index=False))

    grid, origin = points_to_grid(world_map)
    grid_npy = os.path.join(out_dir, "map_occupancy_grid.npy")
    np.save(grid_npy, grid)
    # localize_and_plan.py 에서 grid <-> 실제 mm 좌표 변환에 필요한 메타데이터도 같이 저장
    meta_path = os.path.join(out_dir, "map_grid_meta.csv")
    pd.DataFrame([{
        "origin_x_mm": origin[0], "origin_y_mm": origin[1],
        "resolution_mm": GRID_RESOLUTION_MM,
        "width": grid.shape[1], "height": grid.shape[0]
    }]).to_csv(meta_path, index=False)
    print(f"  - occupancy grid: {grid_npy} (shape={grid.shape})")
    print(f"  - grid 메타데이터: {meta_path}")

    fig, axes = plt.subplots(1, 2, figsize=(16, 8))
    colors = plt.cm.tab10(np.linspace(0, 1, len(transformed_scans)))
    for i, pts in enumerate(transformed_scans):
        axes[0].scatter(pts[:, 0], pts[:, 1], s=2, color=colors[i], label=f"pos{i+1}")
    axes[0].set_title("ICP merged point map")
    axes[0].set_xlabel("x (mm)")
    axes[0].set_ylabel("y (mm)")
    axes[0].legend(markerscale=5)
    axes[0].axis("equal")
    axes[0].invert_yaxis()

    axes[1].imshow(grid, cmap="gray_r", origin="upper")
    axes[1].set_title(f"Occupancy Grid ({GRID_RESOLUTION_MM}mm/cell)")

    fig.tight_layout()
    out_png = os.path.join(out_dir, "map_result.png")
    fig.savefig(out_png, dpi=150)
    print(f"  - 시각화 이미지: {out_png}")

    print("\n완료! pos1을 찍은 그 물리적 위치가 곧 map 좌표계의 (0,0)입니다.")
    print("(다음 단계인 localize_and_plan.py 는 이 (0,0)을 몰라도 동작하도록 만들었습니다.)")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ICP로 여러 라이다 스캔을 정합해서 하나의 맵으로 합치기")
    parser.add_argument("csv_files", nargs="+", help="스캔 CSV 파일들 (pos1, pos2, pos3, ... 순서)")
    parser.add_argument(
        "--init", action="append", default=None,
        help="pos2부터 각 스캔의 대략적 초기 이동량 'dx,dy,dtheta_deg' (mm, mm, deg). "
             "예: --init 1200,0,0 --init 1200,1000,0 --init 0,1000,0"
    )
    parser.add_argument("--out", default=".", help="결과 저장 폴더 (기본: 현재 폴더)")
    parser.add_argument("--max_points", type=int, default=4000,
                         help="ICP 정합에 사용할 스캔당 최대 점 개수 (기본 4000). "
                              "점이 너무 많으면 느려지고 위치별 점개수 불균형이 정합을 왜곡시킬 수 있음.")
    args = parser.parse_args()

    init_guesses = None
    if args.init:
        init_guesses = []
        for s in args.init:
            dx, dy, dth = map(float, s.split(","))
            init_guesses.append((dx, dy, dth))
        if len(init_guesses) != len(args.csv_files) - 1:
            print(f"[오류] --init 개수({len(init_guesses)})는 CSV 개수-1({len(args.csv_files)-1})와 같아야 해요.")
            sys.exit(1)

    main(args.csv_files, init_guesses, args.out, args.max_points)
