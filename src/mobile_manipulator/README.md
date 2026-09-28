# SCARA 모바일 매니퓰레이터

차동구동 모바일 베이스 위에 SCARA 팔(RRPR)과 평행 그리퍼를 올린 로봇을 URDF(xacro)로 직접 만들고,
RViz → Gazebo(Classic) → ros2_control 순서로 실행하는 예제. (ROS 2 Humble)
구조는 `open_manipulator_x_description` / `open_manipulator_x_bringup` 을 참고했다.

## 패키지
| 패키지 | 내용 |
|---|---|
| `mobile_manipulator_description` | URDF(xacro), RViz 표시용 launch |
| `mobile_manipulator_bringup` | 컨트롤러 설정, Gazebo launch, 역기구학 데모 노드 |

```
urdf/
├── mobile_manipulator.urdf.xacro  최상위 (include + 매크로 호출)
├── common.xacro                   치수·재질·관성 공식 매크로 (치수는 여기서만 수정)
├── base.xacro                     base_footprint, base_link, 좌우 바퀴(continuous), 캐스터
├── scara_arm.xacro                column, joint1~4 (R,R,P,R), 그리퍼(prismatic + mimic)
├── gazebo.xacro                   마찰/색상, gazebo_ros2_control 플러그인
└── ros2_control.xacro             바퀴=velocity, 팔·그리퍼=position
```

## 로봇 사양
| 관절 | 종류 | 축 | 범위 | 컨트롤러 |
|---|---|---|---|---|
| left/right_wheel_joint | continuous | y | - | diff_drive_controller |
| joint1 (어깨) | revolute | z | ±2.5 rad | arm_controller |
| joint2 (팔꿈치) | revolute | z | ±2.5 rad | arm_controller |
| joint3 (quill) | prismatic | -z (양수=하강) | 0 ~ 0.18 m | arm_controller |
| joint4 (손목) | revolute | z | ±π | arm_controller |
| gripper_left_joint | prismatic | y | 0 ~ 0.03 m | gripper_controller |
| gripper_right_joint | prismatic (mimic) | -y | left 따라감 | - |

팔 길이 L1 = 0.20 m, L2 = 0.15 m, joint3 = 0 일 때 손끝 높이 0.20 m.

## 빌드
```bash
cd ~/robot_ws
colcon build --symlink-install --packages-select mobile_manipulator_description mobile_manipulator_bringup
source install/setup.bash
```

## 1. URDF 확인 (RViz)
```bash
xacro src/mobile_manipulator/mobile_manipulator_description/urdf/mobile_manipulator.urdf.xacro > /tmp/mm.urdf
check_urdf /tmp/mm.urdf
ros2 launch mobile_manipulator_description display.launch.py   # 슬라이더로 관절 움직여보기
```

## 2. Gazebo + ros2_control
```bash
ros2 launch mobile_manipulator_bringup gazebo.launch.py          # start_rviz:=true 로 RViz 같이 실행
ros2 control list_controllers                                     # 4개 active 확인
```

주행
```bash
ros2 topic pub -r 10 /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.3}}"
# 키보드
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/diff_drive_controller/cmd_vel_unstamped
```

팔 / 그리퍼
```bash
ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [joint1, joint2, joint3, joint4], points: [{positions: [0.8, -1.2, 0.1, 0.5], time_from_start: {sec: 2}}]}}"
ros2 action send_goal /gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.02}}"
```

## 3. 역기구학 데모
```bash
ros2 run mobile_manipulator_bringup scara_demo.py
```
전진 → 목표 (x, y, z) 로 팔 이동 → quill 하강 → 그리퍼 닫기 → 상승 → 원위치.
`scara_ik()` 는 2링크 평면 역기구학(코사인 법칙) + quill 높이 + 손목 보상각으로 관절값을 계산한다.
```
q2 = ±acos((x² + y² − L1² − L2²) / (2·L1·L2))
q1 = atan2(y, x) − atan2(L2·sin q2, L1 + L2·cos q2)
q3 = 0.20 − z
q4 = yaw − q1 − q2
```

## 트러블슈팅
- **`gazebo_ros2_control: parser error Couldn't parse parameter override rule`**
  xacro 주석에 `콜론+공백`이 있으면 발생. URDF 전체가 YAML 파라미터로 전달되기 때문. 주석에서 콜론을 빼면 해결.
- **`Service /spawn_entity unavailable`**
  첫 실행 시 gzserver 기동이 느림. launch 에서 `-timeout 120` 으로 늘려둠.
  이전 gzserver 가 남아 있으면 새 gzserver 가 죽으니 `killall -9 gzserver gzclient` 후 재실행.
