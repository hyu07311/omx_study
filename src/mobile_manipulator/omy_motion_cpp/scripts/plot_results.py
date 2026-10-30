#!/usr/bin/env python3
"""
omy_motion_cpp 결과 그래프 (README 용 PNG)
  ros2 run omy_motion_cpp plot_results.py [--dir ~/robot_ws/motion_cpp_results]
  ik/summary.csv            → ik_median_ms.png       방식별 중앙 시간 (로그 축) + 성공률
  rrt/summary.csv           → rrt_comparison.png     계획 시간 / 충돌 0 비율 / 경로 길이 (작은 그림 3 개)
  rrt/path_obstacle_*.csv   → rrt_paths.png          대표 경로 손끝 (위에서 본 그림)
  pick_place/trajectory.csv → pick_place_path.png    손끝 경로 (위 / 옆), 구간 종류별 색
"""
import argparse
import csv
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402

plt.rcParams['font.family'] = ['Noto Sans CJK KR', 'Noto Sans CJK JP', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

# 범주 색 (고정 순서): 1 파랑 2 주황 3 청록 4 노랑
BLUE, ORANGE, AQUA, YELLOW = '#2a78d6', '#eb6834', '#1baf7a', '#eda100'
INK, MUTED, GRID = '#262624', '#73726c', '#e4e3dc'
FAMILY_COLOR = {'C++': BLUE, 'Python': ORANGE, 'MoveIt KDL': AQUA, 'MoveIt LMA': YELLOW}
PLANNER_COLOR = {'ours': BLUE, 'moveit': ORANGE}
PLANNER_LABEL = {'ours': '우리 RRT-Connect (C++)', 'moveit': 'MoveIt OMPL RRTConnect'}


def read(path):
    with open(path, newline='') as f:
        return list(csv.DictReader(f))


def style(ax):
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(MUTED)
    ax.tick_params(colors=INK, labelsize=8)
    ax.grid(color=GRID, lw=0.8)
    ax.set_axisbelow(True)


def family(method):
    for k in ('MoveIt KDL', 'MoveIt LMA', 'Python', 'C++'):
        if method.startswith(k):
            return k
    return method


def plot_ik(d):
    rows = read(os.path.join(d, 'ik', 'summary.csv'))
    order = ['Python ik_with_restarts', 'C++ ikWithRestarts', 'Python ik (single)', 'C++ ik (single)',
             'MoveIt KDL', 'MoveIt LMA']
    rows.sort(key=lambda r: (order.index(r['method']) if r['method'] in order else 99, r['start']))
    fig, ax = plt.subplots(figsize=(8, 5), dpi=150)
    labels, y = [], 0
    for r in rows:
        ms, pct = float(r['median_ms']), float(r['success_pct'])
        ax.barh(y, ms, height=0.7, color=FAMILY_COLOR[family(r['method'])], edgecolor='white', linewidth=2)
        ax.text(ms * 1.15, y, f'{ms:.3g} ms · 성공 {pct:.1f} %', va='center', fontsize=7.5, color=INK)
        labels.append(f'{r["method"]}  ({r["start"]})')
        y += 1
    ax.set_yticks(range(len(labels)), labels)
    ax.invert_yaxis()
    ax.set_xscale('log')
    ax.set_xlim(right=max(float(r['median_ms']) for r in rows) * 30)
    ax.set_xlabel('목표 1 개당 중앙 시간 (ms, 로그 축)', color=INK)
    ax.set_title('IK 300 개 목표 - 같은 목표·같은 시작 자세, 프로세스 안 함수 호출 시간', fontsize=10, color=INK)
    style(ax)
    ax.grid(axis='y', visible=False)
    handles = [plt.Rectangle((0, 0), 1, 1, color=c) for c in FAMILY_COLOR.values()]
    ax.legend(handles, FAMILY_COLOR.keys(), fontsize=8, frameon=False, loc='lower right')
    fig.tight_layout()
    fig.savefig(os.path.join(d, 'ik_median_ms.png'))
    plt.close(fig)


def plot_rrt(d):
    rows = read(os.path.join(d, 'rrt', 'summary.csv'))
    cases = []
    for r in rows:
        c = (r['scene'], r['fraction'])
        if c not in cases:
            cases.append(c)
    names = [f'{"장애물" if s == "obstacle" else "빈 장면"}\n간격 {float(f) * 68.07:.3f} rad' for s, f in cases]
    metrics = [('median_ms', '계획 시간 중앙 (ms)'), ('collision_free_pct', '원래 크기 재검사 충돌 0 (%)'),
               ('median_length_rad', '경로 길이 중앙 Σ|Δq| (rad)')]
    fig, axes = plt.subplots(1, 3, figsize=(12, 4), dpi=150)
    w = 0.38
    for ax, (key, title) in zip(axes, metrics):
        for k, planner in enumerate(('ours', 'moveit')):
            vals = [float(next(r[key] for r in rows if (r['scene'], r['fraction']) == c and r['planner'] == planner))
                    for c in cases]
            xs = [i + (k - 0.5) * w for i in range(len(cases))]
            ax.bar(xs, vals, width=w, color=PLANNER_COLOR[planner], edgecolor='white', linewidth=2,
                   label=PLANNER_LABEL[planner])
            for x, v in zip(xs, vals):
                ax.text(x, v, f'{v:.3g}', ha='center', va='bottom', fontsize=7, color=INK)
        ax.set_xticks(range(len(cases)), names, fontsize=7.5)
        ax.set_title(title, fontsize=9, color=INK)
        style(ax)
        ax.grid(axis='x', visible=False)
    axes[1].set_ylim(0, 110)
    axes[0].legend(fontsize=7.5, frameon=False, loc='upper left')
    fig.suptitle('관절공간 RRT-Connect, 7주차 transfer 동작, 각 100 회 (같은 PlanningScene, 같은 간선 검사 간격)',
                 fontsize=10, color=INK)
    fig.tight_layout()
    fig.savefig(os.path.join(d, 'rrt_comparison.png'))
    plt.close(fig)


def box_xy(ax, center, size, label):
    ax.add_patch(Rectangle((center[0] - size[0] / 2, center[1] - size[1] / 2), size[0], size[1],
                           facecolor='#b5b3a9', edgecolor=MUTED, alpha=0.6, label=label))


def plot_rrt_paths(d):
    files = [(p, os.path.join(d, 'rrt', f'path_obstacle_0.0005_{p}.csv')) for p in ('ours', 'moveit')]
    if not all(os.path.exists(f) for _, f in files):
        return
    fig, ax = plt.subplots(figsize=(5.5, 5), dpi=150)
    box_xy(ax, (0.42, -0.10), (0.10, 0.10), '장애물 상자 (위에서 본 단면)')
    for planner, f in files:
        rows = read(f)
        ax.plot([float(r['x']) for r in rows], [float(r['y']) for r in rows], color=PLANNER_COLOR[planner],
                lw=2, label=PLANNER_LABEL[planner])
    ax.plot([0.45, 0.35], [0.0, -0.20], 'o', color=INK, ms=5)
    ax.annotate('시작 (0.45, 0)', (0.45, 0.0), xytext=(6, 4), textcoords='offset points', fontsize=7.5)
    ax.annotate('목표 (0.35, -0.20)', (0.35, -0.20), xytext=(6, -10), textcoords='offset points', fontsize=7.5)
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('대표 경로 (길이 중앙값), 손끝 위에서 본 그림', fontsize=9, color=INK)
    style(ax)
    ax.legend(fontsize=7, frameon=False, loc='upper left')
    fig.tight_layout()
    fig.savefig(os.path.join(d, 'rrt_paths.png'))
    plt.close(fig)


def plot_pick_place(d):
    path = os.path.join(d, 'pick_place', 'trajectory.csv')
    if not os.path.exists(path):
        return
    rows = read(path)
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.2), dpi=150, gridspec_kw={'width_ratios': [1, 1.6]})
    box_xy(ax1, (0.44, -0.127), (0.12, 0.02), '장애물 벽')
    colors = {'rrt_connect': BLUE, 'cartesian': ORANGE}
    labels = {'rrt_connect': '이동 - RRT-Connect', 'cartesian': '하강/상승 - Cartesian 직선'}
    segs = []
    for r in rows:
        if not segs or segs[-1][0] != r['segment']:
            segs.append((r['segment'], r['kind'], []))
        segs[-1][2].append(r)
    shown = set()
    for name, kind, pts in segs:
        if kind not in colors:
            continue
        lab = labels[kind] if kind not in shown else None
        shown.add(kind)
        ax1.plot([float(p['x']) for p in pts], [float(p['y']) for p in pts], color=colors[kind], lw=2, label=lab)
        ax2.plot([float(p['t']) for p in pts], [float(p['z']) for p in pts], color=colors[kind], lw=2)
    ax2.axhspan(0.10, 0.30, color='#b5b3a9', alpha=0.35, lw=0)
    ax2.text(float(rows[-1]['t']), 0.105, '벽 높이 0.10~0.30 m', fontsize=7, color=MUTED, ha='right')
    t0 = 0.0
    for name, kind, pts in segs:
        t_mid = (float(pts[0]['t']) + float(pts[-1]['t'])) / 2
        if kind != 'gripper':
            ax2.text(t_mid, float(max(p['z'] for p in pts)) + 0.012, name.replace('_', '\n'), ha='center',
                     fontsize=6, color=MUTED)
        t0 = float(pts[-1]['t'])
    ax1.set_aspect('equal')
    ax1.set_xlabel('x (m)')
    ax1.set_ylabel('y (m)')
    ax1.set_title('손끝 경로 (위에서 본 그림)', fontsize=9, color=INK)
    ax2.set_xlabel('시간 (s)')
    ax2.set_ylabel('손끝 높이 z (m)')
    ax2.set_title('손끝 높이 (시간 매개화 후, 0.01 s 마다)', fontsize=9, color=INK)
    ax2.set_ylim(0.05, 0.42)
    for ax in (ax1, ax2):
        style(ax)
    ax1.legend(fontsize=7, frameon=False, loc='upper center', bbox_to_anchor=(0.5, -0.18), ncol=1)
    fig.suptitle('픽앤플레이스 결합 경로 - 전체 궤적 재검사 충돌 0', fontsize=10, color=INK)
    fig.tight_layout()
    fig.savefig(os.path.join(d, 'pick_place_path.png'))
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dir', default=os.path.expanduser('~/robot_ws/motion_cpp_results'))
    d = ap.parse_args().dir
    plot_ik(d)
    plot_rrt(d)
    plot_rrt_paths(d)
    plot_pick_place(d)
    print(f'saved figures to {d}')


if __name__ == '__main__':
    main()
