#!/usr/bin/env python3
"""
SRDF 자기충돌 행렬(disable_collisions) 생성 - Setup Assistant 의 "Self-Collisions" 단계와 같은 규칙
  Adjacent - 기구학 트리에서 바로 이웃한 링크 (형상 없는 링크는 건너뜀)
  Default  - 0 자세에서 이미 겹치는 쌍
  Always   - 무작위 자세 대부분(95 % 이상)에서 겹치는 쌍
  Never    - 무작위 자세 N 개 중 한 번도 안 겹친 쌍

설치된 collisions_updater (Humble) 가 OMY 에서 멈추고 Panda 예제에서도 core dump 가 나서 직접 계산.
OMY 의 충돌 형상은 link0 (원기둥) 외에 모두 box 라서 OBB 분리축 판정(15 축)으로 계산.
원기둥은 외접 box 로 근사 - 실제보다 크므로 Never 판정은 안전하고, 근사한 링크가 낀 쌍은 Default/Always 로 끄지 않음.

  User     - --disable a:b 로 직접 지정 (예: 닫으면 서로 닿도록 만든 손가락)

  python3 gen_collision_matrix.py <urdf> <srdf 입력> <srdf 출력> [--trials 10000] [--disable a:b ...]
"""
import argparse
import math
import re
import xml.etree.ElementTree as ET

import numpy as np
from urdf_parser_py.urdf import URDF


def rpy_matrix(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def axis_angle(axis, angles):
    """(N,) 각도 → (N,3,3) 회전행렬 (로드리게스)"""
    k = np.asarray(axis, float) / np.linalg.norm(axis)
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    s, c = np.sin(angles)[:, None, None], np.cos(angles)[:, None, None]
    return np.eye(3) + s * kx + (1 - c) * (kx @ kx)


def origin_of(element):
    o = element.origin
    xyz = o.xyz if o is not None and o.xyz else [0, 0, 0]
    rpy = o.rpy if o is not None and o.rpy else [0, 0, 0]
    return rpy_matrix(rpy), np.array(xyz, float)


class Robot:

    def __init__(self, urdf_path):
        self.robot = URDF.from_xml_file(urdf_path)
        self.root = self.robot.get_root()
        # 링크별 box 목록 (회전, 중심, 반 크기)
        self.boxes = {}
        self.approx = set()   # 외접 box 로 근사한 링크
        for link in self.robot.links:
            for col in link.collisions:
                g = col.geometry
                if hasattr(g, 'size'):
                    half = np.array(g.size) / 2
                elif hasattr(g, 'length'):   # 원기둥 (z 축)
                    half = np.array([g.radius, g.radius, g.length / 2])
                    self.approx.add(link.name)
                else:
                    raise SystemExit(f'{link.name}: box / cylinder 외 충돌 형상은 지원 안 함')
                rot, pos = origin_of(col)
                self.boxes.setdefault(link.name, []).append((rot, pos, half))
        # 움직이는 관절 (mimic 제외) 과 범위
        self.active = [j for j in self.robot.joints
                       if j.type in ('revolute', 'continuous', 'prismatic') and j.mimic is None]

    def sample(self, n, rng):
        q = {}
        for j in self.active:
            lo = max(j.limit.lower, -math.pi) if j.limit else -math.pi
            hi = min(j.limit.upper, math.pi) if j.limit else math.pi
            q[j.name] = rng.uniform(lo, hi, n) if n > 1 else np.zeros(1)
        return q

    def fk(self, q, n):
        """모든 링크의 (N,3,3) 회전, (N,3) 위치"""
        rot = {self.root: np.broadcast_to(np.eye(3), (n, 3, 3))}
        pos = {self.root: np.zeros((n, 3))}
        stack = [self.root]
        while stack:
            parent = stack.pop()
            for jname, child in self.robot.child_map.get(parent, []):
                j = self.robot.joint_map[jname]
                r0, p0 = origin_of(j)
                r = rot[parent] @ r0
                p = pos[parent] + rot[parent] @ p0
                if j.type != 'fixed':
                    v = q[j.mimic.joint] * (j.mimic.multiplier or 1.0) + (j.mimic.offset or 0.0) \
                        if j.mimic else q[jname]
                    r = r @ axis_angle(j.axis, v)
                rot[child], pos[child] = r, p
                stack.append(child)
        return rot, pos

    def geometry_parent(self, link):
        """형상 있는 가장 가까운 조상 링크 (형상 없는 링크는 건너뜀)"""
        while link in self.robot.parent_map:
            link = self.robot.parent_map[link][1]
            if link in self.boxes:
                return link
        return None


def obb_overlap(ra, ca, ea, rb, cb, eb):
    """(N,) bool - OBB 두 개가 겹치는지 (분리축 15 개)"""
    eps = 1e-9
    r = np.einsum('nji,njk->nik', ra, rb)          # B 축을 A 좌표로
    t = np.einsum('nji,nj->ni', ra, cb - ca)        # 중심 차이를 A 좌표로
    abs_r = np.abs(r) + eps
    sep = np.zeros(len(t), bool)
    for i in range(3):   # A 축
        sep |= np.abs(t[:, i]) > ea[i] + abs_r[:, i, :] @ eb
    for i in range(3):   # B 축
        sep |= np.abs(np.einsum('nj,nj->n', t, r[:, :, i])) > abs_r[:, :, i] @ ea + eb[i]
    for i in range(3):   # A_i x B_j
        i1, i2 = (i + 1) % 3, (i + 2) % 3
        for j in range(3):
            j1, j2 = (j + 1) % 3, (j + 2) % 3
            ra_ = ea[i1] * abs_r[:, i2, j] + ea[i2] * abs_r[:, i1, j]
            rb_ = eb[j1] * abs_r[:, i, j2] + eb[j2] * abs_r[:, i, j1]
            sep |= np.abs(t[:, i2] * r[:, i1, j] - t[:, i1] * r[:, i2, j]) > ra_ + rb_
    return ~sep


def collide(robot, rot, pos, a, b):
    hit = np.zeros(len(pos[a]), bool)
    for ra, pa, ea in robot.boxes[a]:
        for rb, pb, eb in robot.boxes[b]:
            hit |= obb_overlap(rot[a] @ ra, pos[a] + rot[a] @ pa, ea,
                               rot[b] @ rb, pos[b] + rot[b] @ pb, eb)
    return hit


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('urdf')
    ap.add_argument('srdf_in')
    ap.add_argument('srdf_out')
    ap.add_argument('--trials', type=int, default=10000)
    ap.add_argument('--always', type=float, default=0.95, help='Always 판정 비율')
    ap.add_argument('--disable', nargs='*', default=[], help='직접 끌 쌍 link_a:link_b')
    args = ap.parse_args()

    robot = Robot(args.urdf)
    links = sorted(robot.boxes)
    pairs = [(a, b) for i, a in enumerate(links) for b in links[i + 1:]]

    reasons = {}
    for a, b in pairs:
        if robot.geometry_parent(a) == b or robot.geometry_parent(b) == a:
            reasons[(a, b)] = 'Adjacent'

    for arg in args.disable:
        p = tuple(sorted(arg.split(':')))
        if p not in pairs:
            raise SystemExit(f'unknown pair {arg}')
        reasons.setdefault(p, 'User')

    rot, pos = robot.fk(robot.sample(1, None), 1)
    exact = [p for p in pairs if not set(p) & robot.approx]
    for p in exact:
        if p not in reasons and collide(robot, rot, pos, *p)[0]:
            reasons[p] = 'Default'

    rng = np.random.default_rng(0)
    rot, pos = robot.fk(robot.sample(args.trials, rng), args.trials)
    counts = {}
    for p in pairs:
        if p not in reasons:
            rate = collide(robot, rot, pos, *p).mean()
            counts[p] = rate
            if rate >= args.always and p in exact:
                reasons[p] = 'Always'
            elif rate == 0.0:
                reasons[p] = 'Never'

    # 기존 disable_collisions 를 지우고 새로 씀
    text = open(args.srdf_in, encoding='utf-8').read()
    text = re.sub(r'\n?\s*<!-- self-collision matrix.*?-->', '', text, flags=re.S)
    text = re.sub(r'\n?\s*<disable_collisions [^>]*/>', '', text)
    # XML 주석 안에는 '--' 를 쓸 수 없음
    lines = [f'  <!-- self-collision matrix: scripts/gen_collision_matrix.py, trials {args.trials}'
             + (f', disable {" ".join(args.disable)}' if args.disable else '') + ' -->']
    lines += [f'  <disable_collisions link1="{a}" link2="{b}" reason="{reasons[(a, b)]}"/>'
              for a, b in pairs if (a, b) in reasons]
    text = text.replace('</robot>', '\n'.join(lines) + '\n</robot>')
    ET.fromstring(text.encode('utf-8'))  # XML 확인
    open(args.srdf_out, 'w', encoding='utf-8').write(text)

    summary = {}
    for r in reasons.values():
        summary[r] = summary.get(r, 0) + 1
    print(f'{len(links)} links, {len(pairs)} pairs, disabled {len(reasons)}: {summary}')
    print(f'approximated by bounding box: {sorted(robot.approx)}')
    print('checked (sometimes collide):')
    for (a, b), rate in sorted(counts.items(), key=lambda kv: -kv[1]):
        if (a, b) not in reasons:
            print(f'  {a:28s} {b:28s} {rate * 100:6.2f} %')


if __name__ == '__main__':
    main()
