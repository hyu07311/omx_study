# UR식 6축 모바일 매니퓰레이터

SCARA 버전과 같은 차동구동 베이스 위에 UR식 6축 팔을 올린 버전.
SCARA 버전 파일은 수정하지 않고, 새 파일에서 `common.xacro`, `base.xacro`,
`ros2_control.xacro`, `gazebo.xacro` 를 include 해서 치수·베이스·매크로 정의만 가져다 쓴다.

## 파일
| 파일 | 내용 |
|---|---|
| `urdf/mobile_manipulator_ur6.urdf.xacro` | 6축 버전 최상위 |
| `urdf/ur6_arm.xacro` | 6축 팔 치수(`ur_*`), 링크/관절, 그리퍼 |
| `urdf/ur6_ros2_control.xacro` | 바퀴 2 + joint1~6 + 그리퍼 |
| `urdf/ur6_gazebo.xacro` | 링크 마찰/색상, gazebo_ros2_control 플러그인 |
| `launch/display_ur6.launch.py` | RViz + 슬라이더 |
| `config/controllers_ur6.yaml` | arm_controller joints = joint1~6 |
| `launch/gazebo_ur6.launch.py` | Gazebo + 컨트롤러 |
| `scripts/ur6_ik_demo.py` | 야코비안 수치 IK 데모 |

## 관절 (0 자세 = 팔이 위로 곧게 선 자세)
| 관절 | 축 | 부모 기준 위치 (m) | 범위 (rad) |
|---|---|---|---|
| joint1 shoulder_pan | z | (0, 0, 0.10) | ±π |
| joint2 shoulder_lift | y | (0, 0.07, 0) | ±2.0 |
| joint3 elbow | y | (0, −0.06, 0.22) | ±2.6 |
| joint4 wrist1 | y | (0, 0, 0.20) | ±π |
| joint5 wrist2 | z | (0, 0.05, 0) | ±π |
| joint6 wrist3 | y | (0, 0, 0.05) | ±π |

`tool0` 은 wrist3 의 +y 쪽 0.03 m 에 있고 z축이 wrist3 의 +y 를 향한다. 그리퍼는 손가락이 tool0 의 +z 쪽으로 뻗게 붙어 있다.
그리퍼, 바퀴, 컨트롤러 이름은 SCARA 버전과 같다.

## 실행
```bash
cd ~/robot_ws
colcon build --symlink-install --packages-select mobile_manipulator_description mobile_manipulator_bringup
source install/setup.bash

ros2 launch mobile_manipulator_description display_ur6.launch.py   # RViz
ros2 launch mobile_manipulator_bringup gazebo_ur6.launch.py         # Gazebo
ros2 run mobile_manipulator_bringup ur6_ik_demo.py                  # IK 데모 (다른 터미널)
```

관절로 직접 명령하기
```bash
ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [joint1, joint2, joint3, joint4, joint5, joint6], points: [{positions: [0.0, 0.6, 1.2, -0.23, -1.57, 0.0], time_from_start: {sec: 3}}]}}"
```

## 수치 역기구학 (`ur6_ik_demo.py`)
1. `/robot_description` 의 URDF 를 `urdf_parser_py` 로 읽어 base_footprint → end_effector_link 관절 체인을 만든다 (치수 하드코딩 없음).
2. **순기구학**: T = Π (T_origin,i · Rot(axis_i, q_i))
3. **기하 야코비안** (6×6): 회전관절 i 의 열 = [ z_i × (p_e − p_i) ; z_i ]
4. **Damped least squares** 반복
   ```
   e  = [ p_target − p ; 축각(R_target · Rᵀ) ]
   dq = Jᵀ (J Jᵀ + λ² I)⁻¹ e        (λ = 0.05, 한 번에 최대 0.2 rad)
   q  ← clip(q + dq, 관절 한계)
   ```
   위치 오차 0.1 mm, 자세 오차 0.001 rad 이내면 종료한다.
5. 팔이 곧게 선 0 자세는 특이점이라 초기값으로 쓰기 어렵다. 그래서 먼저 준비 자세 `READY_POSE` 로 옮긴 뒤 IK 를 푼다.
   (q2+q3+q4 = π/2 이면 wrist2 축이 수평이 되고, q5 = −π/2 이면 그리퍼가 아래를 향한다.)

## SCARA vs 6축
| | SCARA (RRPR) | UR식 6축 |
|---|---|---|
| 자유도 | 4 (x, y, z, yaw) | 6 (위치 3 + 자세 3) |
| 중력 부하 | 수직 이동축에만 | 어깨·팔꿈치·손목1 (pitch 축) |
| 역기구학 | 코사인 법칙으로 닫힌 해 | 야코비안 반복 계산 (수치 해) |
| 특이점 | 팔이 완전히 펴질 때 | 팔이 펴질 때, 손목 축이 정렬될 때 등 여러 곳 |
| 작업 공간 | 원통형, 위에서 아래로 집기 | 구형, 임의의 방향에서 접근 |
