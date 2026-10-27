"""读取 metrics_recorder 输出的 CSV 并绘制轨迹/误差图。

    ros2 run path_follow_sim plot_result /tmp/path_follow_sim [--show]
默认保存 result.png；无显示环境不要加 --show。
"""
import argparse
import csv
import os


def _load(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    return {k: [float(r[k]) for r in rows] for k in rows[0]} if rows else {}


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('directory', nargs='?', default='/tmp/path_follow_sim')
    ap.add_argument('--show', action='store_true')
    args = ap.parse_args(argv)
    import matplotlib
    if not args.show:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    traj = _load(os.path.join(args.directory, 'trajectory.csv'))
    ref = _load(os.path.join(args.directory, 'path.csv'))
    if not traj or not ref:
        raise SystemExit('no data to plot')
    fig, ax = plt.subplots(1, 3, figsize=(16, 4.5))
    ax[0].plot(ref['x'], ref['y'], 'k--', label='reference')
    ax[0].plot(traj['x'], traj['y'], 'r', label='actual')
    ax[0].set_aspect('equal')
    ax[0].set_title('trajectory')
    ax[0].legend()
    ax[1].plot(traj['t'], traj['cross_track'], label='cross track [m]')
    ax[1].plot(traj['t'], traj['heading_error'], label='heading error [rad]')
    ax[1].set_title('tracking error')
    ax[1].legend()
    ax[2].plot(traj['t'], traj['vx'], label='vx')
    ax[2].plot(traj['t'], traj['vy'], label='vy')
    ax[2].plot(traj['t'], traj['w'], label='yaw rate')
    ax[2].set_title('body velocity')
    ax[2].legend()
    for a in ax:
        a.grid(True)
    out = os.path.join(args.directory, 'result.png')
    fig.tight_layout()
    fig.savefig(out, dpi=120)
    print(out)
    if args.show:
        plt.show()


if __name__ == '__main__':
    main()
