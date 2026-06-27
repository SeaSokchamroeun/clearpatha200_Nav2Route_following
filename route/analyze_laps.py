#!/usr/bin/env python3
import argparse, glob, json, math, os, statistics as stats

CORNER_RADIUS = 0.5
EXPECTED_DT = 0.05      # 20 Hz controller_frequency
DT_MIN, DT_MAX = 0.02, 0.15   # plausible sample-spacing band for accel calc
NODE_TIME_WINDOW = 15.0       # seconds — keeps node-0 start/end from merging


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
        ax, ay = path[i]; bx, by = path[i + 1]
        d, s = point_to_segment(px, py, ax, ay, bx, by)
        if d < best_dist:
            best_dist, best_signed = d, s
    return best_dist, best_signed


def rms(vals):
    return math.sqrt(sum(v * v for v in vals) / len(vals)) if vals else 0.0


def analyze_lap(lap):
    traj, path, vel, node_log = lap['trajectory'], lap['reference_path'], lap['velocity'], lap['node_log']
    r = {'lap_index': lap['lap_index'], 'lap_result': lap['lap_result'],
         'edge_results': lap['edge_results'], 'n_traj_points': len(traj)}

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
        vxs = [v['vx'] for v in vel]; wzs = [v['wz'] for v in vel]; ts = [v['t'] for v in vel]
        accels = []
        for i in range(1, len(vel)):
            dt = ts[i] - ts[i-1]
            if DT_MIN <= dt <= DT_MAX:
                accels.append((vxs[i] - vxs[i-1]) / dt)
        jerks = [(accels[i] - accels[i-1]) / EXPECTED_DT for i in range(1, len(accels))]
        n_rejected = (len(vel) - 1) - len(accels)
        r['vx_mean'] = round(stats.mean(vxs), 4)
        r['vx_std'] = round(stats.pstdev(vxs), 4)
        r['wz_std'] = round(stats.pstdev(wzs), 4)
        r['max_abs_accel'] = round(max((abs(a) for a in accels), default=0.0), 4)
        r['rms_jerk'] = round(rms(jerks), 4)
        r['accel_samples_rejected'] = n_rejected
    else:
        for k in ['vx_mean', 'vx_std', 'wz_std', 'max_abs_accel', 'rms_jerk', 'accel_samples_rejected']:
            r[k] = None

    nodes = []
    for n in node_log:
        nid, nx, ny, t_arr = n['id'], n['x'], n['y'], n['t_arrival']
        window = [pt for pt in traj
                  if math.hypot(pt['x'] - nx, pt['y'] - ny) <= CORNER_RADIUS
                  and abs(pt['t'] - t_arr) <= NODE_TIME_WINDOW]
        if not window:
            continue
        ts_w = [pt['t'] for pt in window]
        ct = [nearest_signed_distance(pt['x'], pt['y'], path)[0] for pt in window] if path else [0.0]
        peak_wz = 0.0
        for pt in window:
            if vel:
                closest = min(vel, key=lambda v: abs(v['t'] - pt['t']))
                peak_wz = max(peak_wz, abs(closest['wz']))
        label = 'loop_closure' if nid == 0 else 'corner'
        nodes.append({'node_id': nid, 'label': label, 'dwell_time_s': round(max(ts_w) - min(ts_w), 3),
                      'max_cross_track_m': round(max(ct), 4), 'peak_angular_velocity': round(peak_wz, 4)})
    r['nodes'] = nodes
    return r


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--logdir', type=str, default=os.path.expanduser('~/clearpath/eval_logs'))
    args = parser.parse_args()

    files = sorted(glob.glob(os.path.join(args.logdir, 'lap_*.json')))
    if not files:
        print(f'No lap logs found in {args.logdir}')
        return

    laps = [analyze_lap(json.load(open(f))) for f in files]

    print('=' * 72)
    print(f'PERFORMANCE EVALUATION — {len(laps)} lap(s), full loop 0->1->2->3->0')
    print('=' * 72)
    for r in laps:
        print(f"\nLap {r['lap_index']:>3}  result={r['lap_result']}")
        print('  Edges: ' + ', '.join(f"{e['edge']}={e['result']}" for e in r['edge_results']))
        print(f"  Tracking accuracy : mean={r['tracking_mean_error_m']} m  rmse={r['tracking_rmse_m']} m  max={r['tracking_max_error_m']} m")
        print(f"  Cross-track error : mean|.|={r['cross_track_mean_abs_m']} m  rmse={r['cross_track_rmse_m']} m  max={r['cross_track_max_m']} m")
        print(f"  Velocity stability: vx_std={r['vx_std']}  wz_std={r['wz_std']}  max_accel={r['max_abs_accel']}  rms_jerk={r['rms_jerk']}"
              f"  (rejected {r['accel_samples_rejected']} implausible dt samples)")
        if r['nodes']:
            print('  Node transitions:')
            for n in r['nodes']:
                print(f"    node {n['node_id']} ({n['label']}): dwell={n['dwell_time_s']}s  max_xte={n['max_cross_track_m']}m  peak_wz={n['peak_angular_velocity']}")

    total = len(laps)
    lap_succ = sum(1 for r in laps if r['lap_result'] == 'SUCCEEDED')
    print('\n' + '-' * 72)
    print(f'LAP COMPLETION SUCCESS RATE: {lap_succ}/{total} = {100.0*lap_succ/total:.1f}%')

    edge_counter = {}
    for r in laps:
        for e in r['edge_results']:
            edge_counter.setdefault(e['edge'], [0, 0])
            edge_counter[e['edge']][1] += 1
            if e['result'] == 'SUCCEEDED':
                edge_counter[e['edge']][0] += 1
    print('Per-edge success rate:')
    for edge, (s, t) in edge_counter.items():
        print(f'  {edge}: {s}/{t} = {100.0*s/t:.1f}%')

    succ_laps = [r for r in laps if r['lap_result'] == 'SUCCEEDED']
    if succ_laps:
        print('\nAggregate over SUCCEEDED laps:')
        for key, label in [('tracking_rmse_m', 'Tracking RMSE'), ('cross_track_rmse_m', 'Cross-track RMSE'),
                            ('vx_std', 'vx std'), ('wz_std', 'wz std'), ('max_abs_accel', 'max |accel|')]:
            vals = [r[key] for r in succ_laps if r[key] is not None]
            if vals:
                print(f'  {label}: mean={stats.mean(vals):.4f}  min={min(vals):.4f}  max={max(vals):.4f}')

    summary_path = os.path.join(args.logdir, 'summary_laps.json')
    with open(summary_path, 'w') as f:
        json.dump({'total_laps': total, 'lap_success_count': lap_succ,
                   'lap_success_rate_pct': round(100.0*lap_succ/total, 2), 'laps': laps}, f, indent=2)
    print(f'\nFull summary written to: {summary_path}')

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(7, 6))
        ref = None
        for f in files:
            d = json.load(open(f))
            if d['reference_path']:
                ref = d['reference_path']; break
        if ref:
            ax.plot([p[0] for p in ref], [p[1] for p in ref], 'k--', linewidth=1.5, label='Reference loop')
        for f in files:
            d = json.load(open(f))
            if d['trajectory']:
                ax.plot([pt['x'] for pt in d['trajectory']], [pt['y'] for pt in d['trajectory']],
                        alpha=0.5, linewidth=1, label=f"Lap {d['lap_index']}")
        ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
        ax.set_title('Actual trajectory vs. reference loop')
        ax.legend(fontsize=7); ax.axis('equal')
        plot_path = os.path.join(args.logdir, 'loop_trajectory_overlay.png')
        fig.savefig(plot_path, dpi=150, bbox_inches='tight')
        print(f'Trajectory plot saved to: {plot_path}')
    except ImportError:
        print('matplotlib not available — skipping plot')


if __name__ == '__main__':
    main()