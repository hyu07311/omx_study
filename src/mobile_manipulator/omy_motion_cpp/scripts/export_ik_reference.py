#!/usr/bin/env python3
"""
C++ 이식 비교용 기준 데이터 내보내기 (ROS 노드 없이 실행, move_group 필요 없음)
  목표 300 개는 7주차 omy_ik_benchmark.experiment_a_targets(kin, 300, seed=1) 그대로
  (numpy PCG64 난수를 C++ 에서 재현하지 않고 값 자체를 CSV 로 넘김 → 완전히 같은 목표)
  ik_with_restarts 는 매번 default_rng(0) 이라 무작위 시드 30 개가 항상 같음 → 그 30 개도 저장

  ros2 run omy_motion_cpp export_ik_reference.py [--n 300] [--skip-python] [--out DIR]
결과 - ~/robot_ws/motion_cpp_results/ik/
  omy.urdf              xacro 로 만든 URDF (C++ 벤치마크도 같은 파일을 읽음)
  targets.csv           i, 목표 관절각 g0..5, 목표 자세 (3x4 행 우선) T00..T23, 무작위 시작 far0..5, 가까운 시작 near0..5
  restart_seeds.csv     ik_with_restarts 의 무작위 시드 30 개
  python_results.csv    목표마다 Python ik 단일 / ik_with_restarts 의 성공, ms, 오차, 해
"""
import argparse
import csv
import math
import os
import sys
import time

import numpy as np
from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
import xacro

# 기존 Python 코드 (omy_bringup 에 설치된 스크립트) 를 그대로 import
sys.path.insert(0, os.path.join(get_package_prefix('omy_bringup'), 'lib', 'omy_bringup'))
from omy_ik_benchmark import experiment_a_targets  # noqa: E402
from omy_ik_benchmark import pose_error  # noqa: E402
from omy_ik_benchmark import POS_TOL  # noqa: E402
from omy_ik_benchmark import ROT_TOL  # noqa: E402
from omy_ik_demo import ik_with_restarts  # noqa: E402
from omy_ik_demo import Kinematics  # noqa: E402

RESTART_TRIES = 30   # ik_with_restarts 기본값
RESTART_SEED = 0


def build_urdf():
    path = os.path.join(get_package_share_directory('omy_description'), 'urdf', 'omy.urdf.xacro')
    # omy_moveit_config 의 setup_assistant 와 같은 인자
    return xacro.process_file(path, mappings={'use_sim': 'false', 'variant': 'f3m'}).toxml()


def restart_seeds(kin):
    """ik_with_restarts 안에서 만드는 무작위 시드와 같은 순서"""
    rng = np.random.default_rng(RESTART_SEED)
    lo, hi = np.maximum(kin.lower, -math.pi), np.minimum(kin.upper, math.pi)
    return [rng.uniform(lo, hi) for _ in range(RESTART_TRIES)]


def timed(fn):
    t0 = time.perf_counter()
    try:
        q = fn()
    except ValueError:
        q = None
    return q, (time.perf_counter() - t0) * 1000


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--n', type=int, default=300)
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--skip-python', action='store_true', help='목표만 내보내고 Python 시간은 재지 않음')
    ap.add_argument('--out', default=os.path.expanduser('~/robot_ws/motion_cpp_results/ik'))
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    urdf = build_urdf()
    with open(os.path.join(args.out, 'omy.urdf'), 'w') as f:
        f.write(urdf)
    kin = Kinematics(urdf)
    goals, targets, seeds_far, seeds_near = experiment_a_targets(kin, args.n, args.seed)
    # 7주차 벤치마크와 같이 가까운 시작은 관절 범위 안으로
    seeds_near = [np.clip(s, kin.lower, kin.upper) for s in seeds_near]

    with open(os.path.join(args.out, 'targets.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['i', *[f'g{k}' for k in range(6)], *[f'T{r}{c}' for r in range(3) for c in range(4)],
                    *[f'far{k}' for k in range(6)], *[f'near{k}' for k in range(6)]])
        for i, (g, t, sf, sn) in enumerate(zip(goals, targets, seeds_far, seeds_near)):
            w.writerow([i, *(repr(float(v)) for v in g), *(repr(float(v)) for v in t[:3, :].ravel()),
                        *(repr(float(v)) for v in sf), *(repr(float(v)) for v in sn)])
    with open(os.path.join(args.out, 'restart_seeds.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow([f'q{k}' for k in range(6)])
        w.writerows([[repr(float(v)) for v in s] for s in restart_seeds(kin)])
    print(f'{args.n} targets -> {args.out}/targets.csv')
    if args.skip_python:
        return

    rows = []
    for start, seeds in (('far', seeds_far), ('near', seeds_near)):
        for method, solve in (('single', lambda t, s: kin.ik(t, s)),
                              ('restarts', lambda t, s: ik_with_restarts(kin, t, s))):
            t_start = time.perf_counter()
            for i, (target, s) in enumerate(zip(targets, seeds)):
                q, ms = timed(lambda: solve(target, s))
                pos_err, rot_err = pose_error(kin, q, target) if q is not None else (math.nan, math.nan)
                ok = q is not None and pos_err < POS_TOL and rot_err < ROT_TOL
                rows.append([i, start, method, int(ok), f'{ms:.4f}', repr(pos_err), repr(rot_err),
                             *(repr(float(v)) for v in (q if q is not None else [math.nan] * 6))])
            done = [r for r in rows if r[1] == start and r[2] == method]
            print(f'python {method:8s} {start:4s}: {100 * sum(r[3] for r in done) / len(done):5.1f} %, '
                  f'median {np.median([float(r[4]) for r in done]):8.3f} ms '
                  f'({time.perf_counter() - t_start:.0f} s)', flush=True)

    with open(os.path.join(args.out, 'python_results.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['i', 'start', 'method', 'success', 'ms', 'pos_err', 'rot_err', *[f'q{k}' for k in range(6)]])
        w.writerows(rows)
    print(f'saved {args.out}/python_results.csv')


if __name__ == '__main__':
    main()
