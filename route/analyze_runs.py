#!/usr/bin/env python3
"""
analyze_runs.py — Computes performance evaluation metrics from route_eval.py
log files: tracking accuracy, cross-track error, corner handling,
velocity stability, and route completion success rate.

Usage:
  python3 analyze_runs.py --logdir ~/clearpath/eval_logs
"""
import argparse
import glob
import json
import math
import os
import statistics as stats

CORNER_RADIUS = 0.5  # matches radius_to_achieve_node in route_server_params.yaml


def point_to_segment(px, py, ax, ay, bx, by):
    abx, aby = bx - ax, by - ay
    apx, apy = px - ax, py - ay
    ab_len2 = abx * abx + aby * aby
    if ab_len2 == 0:
        return math.hypot(apx, apy), 0.0
    t = max(0.0, min(1.0, (apx * abx + apy * aby) / ab_len2))
    cx, cy = ax + t * abx, ay + t * aby
    dist = math.hypot(px - cx, py - cy)
    cross = abx * apy - aby * apx
    return dist, (dist if cross >= 0 else -dist)


def nearest_signed_distance(px, py, path):
    best_dist, best_signed = float('inf'), 0.0
    for i in range(len(path) - 1):
        ax, ay = path[i]
        bx, by = path[i + 1]
        d, s = point_to_segment(px, py, ax, ay, bx, by)
        if d < best_dist:
            best_dist, best_signed = d, s
    return best_dist, best_signed


def rms(vals):
    return math.sqrt(sum(v * v for v in vals) / len(vals)) if vals else 0.0


def analyze_run(run):
    traj, path, vel, nodes = run['trajectory'], run['reference_path'], run['velocity'], run['route_nodes']
    r = {'run_index': run['run_index'], 'start_id': run['start_id'], 'end_id': run['end_id'],
         'result': run['result'], 'n_traj_points': len(traj)}

    if traj and path:
        dists, signed = [], []
        for pt in traj:
            d, s = nearest_signed_distance(pt['x'], pt['y'], path)
            dists.append(d); signed.append(s)
        r['tracking_mean_error_m'] = round(stats.mean(dists), 4)
        r['tracking_rmse_m'] = round(rms(dists), 4)
        r['tracking_max_error_m'] = round(max(dists), 4)
        r['cross_track_mean_abs_m'] = round(stats.mean([abs(s) for s in signed]), 4)
        r['cross_track_rmse_m'] = round(rms(signed), 4)
        r['cross_track_max_m'] = round(max(dists), 4)
    else:
        for k in ['tracking_mean_error_m', 'tracking_rmse_m', 'tracking_max_error_m',
                  'cross_track_mean_abs_m', 'cross_track_rmse_m', 'cross_track_max_m']:
            r[k] = None

    if len(vel) >= 3:
        vxs, wzs, ts = [v['vx'] for v in vel], [v['wz'] for v in vel], [v['t'] for v in vel]
        accels = [(vxs[i] - vxs[i-1]) / (ts[i] - ts[i-1]) for i in range(1, len(vel)) if ts[i] - ts[i-1] > 1e-3]
        jerks = [(accels[i] - accels[i-1]) / 0.05 for i in range(1, len(accels))]
        r['vx_mean'] = round(stats.mean(vxs), 4)
        r['vx_std'] = round(stats.pstdev(vxs), 4)
        r['wz_std'] = round(stats.pstdev(wzs), 4)
        r['max_abs_accel'] = round(max((abs(a) for a in accels), default=0.0), 4)
        r['rms_jerk'] = round(rms(jerks), 4)
    else:
        for k in ['vx_mean', 'vx_std', 'wz_std', 'max_abs_accel', 'rms_jerk']:
            r[k] = None

    corners = []
    if traj and nodes and len(nodes) > 2:
        for node in nodes[1:-1]:
            nx, ny = node['x'], node['y']
            window = [pt for pt in traj if math.hypot(pt['x'] - nx, pt['y'] - ny) <= CORNER_RADIUS]
            if not window:
                continue
            ts_w = [pt['t'] for pt in window]
            ct = [nearest_signed_distance(pt['x'], pt['y'], path)[0] for pt in window] if path else [0.0]
            peak_wz = 0.0
            for pt in window:
                if vel:
                    closest = min(vel, key=lambda v: abs(v['t'] - pt['t']))
                    peak_wz = max(peak_wz, abs(closest['wz']))
            corners.append({'node_id': node['id'], 'dwell_time_s': round(max(ts_w) - min(ts_w), 3),
                             'max_cross_track_in_corner_m': round(max(ct), 4),
                             'peak_angular_velocity': round(peak_wz, 4)})
    r['corners'] = corners
    return r


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--logdir', type=str, default=os.path.expanduser('~/clearpath/eval_logs'))
    args = parser.parse_args()

    files = sorted(glob.glob(os.path.join(args.logdir, 'run_*.json')))
    if not files:
        print(f'No run logs found in {args.logdir}')
        return

    per_run = [analyze_run(json.load(open(f))) for f in files]

    print('=' * 70)
    print(f'PERFORMANCE EVALUATION — {len(per_run)} run(s) analyzed')
    print('=' * 70)
    for r in per_run:
        print(f"\nRun {r['run_index']:>3}  ({r['start_id']} -> {r['end_id']})  result={r['result']}")
        print(f"  Tracking accuracy : mean={r['tracking_mean_error_m']} m  rmse={r['tracking_rmse_m']} m  max={r['tracking_max_error_m']} m")
        print(f"  Cross-track error : mean|.|={r['cross_track_mean_abs_m']} m  rmse={r['cross_track_rmse_m']} m  max={r['cross_track_max_m']} m")
        print(f"  Velocity stability: vx_std={r['vx_std']}  wz_std={r['wz_std']}  max_accel={r['max_abs_accel']}  rms_jerk={r['rms_jerk']}")
        if r['corners']:
            print('  Corner handling:')
            for c in r['corners']:
                print(f"    node {c['node_id']}: dwell={c['dwell_time_s']}s  max_xte={c['max_cross_track_in_corner_m']}m  peak_wz={c['peak_angular_velocity']}")

    total = len(per_run)
    succ = sum(1 for r in per_run if r['result'] == 'SUCCEEDED')
    print('\n' + '-' * 70)
    print(f'ROUTE COMPLETION SUCCESS RATE: {succ}/{total} = {100.0*succ/total:.1f}%')

    by_pair = {}
    for r in per_run:
        key = (r['start_id'], r['end_id'])
        by_pair.setdefault(key, [0, 0])
        by_pair[key][1] += 1
        if r['result'] == 'SUCCEEDED':
            by_pair[key][0] += 1
    for key, (s, t) in by_pair.items():
        print(f'  {key[0]} -> {key[1]}: {s}/{t} = {100.0*s/t:.1f}%')

    summary_path = os.path.join(args.logdir, 'summary.json')
    with open(summary_path, 'w') as f:
        json.dump({'total_runs': total, 'success_count': succ,
                    'success_rate_pct': round(100.0*succ/total, 2), 'runs': per_run}, f, indent=2)
    print(f'\nFull summary written to: {summary_path}')

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(7, 6))
        ref = next((json.load(open(f)) for f in files if json.load(open(f))['reference_path']), None)
        if ref:
            rp = ref['reference_path']
            ax.plot([p[0] for p in rp], [p[1] for p in rp], 'k--', linewidth=1.5, label='Reference path')
        for f in files:
            run = json.load(open(f))
            if run['trajectory']:
                ax.plot([pt['x'] for pt in run['trajectory']], [pt['y'] for pt in run['trajectory']],
                        alpha=0.6, linewidth=1, label=f"Run {run['run_index']}")
        ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
        ax.set_title('Actual trajectory vs. reference path')
        ax.legend(fontsize=7); ax.axis('equal')
        plot_path = os.path.join(args.logdir, 'trajectory_overlay.png')
        fig.savefig(plot_path, dpi=150, bbox_inches='tight')
        print(f'Trajectory plot saved to: {plot_path}')
    except ImportError:
        print('matplotlib not available — skipping plot (pip install matplotlib --break-system-packages)')


if __name__ == '__main__':
    main()