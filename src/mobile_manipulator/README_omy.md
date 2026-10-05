# OpenMANIPULATOR-Y (OMY) URDF 직접 작성

ROBOTIS 최신 매니퓰레이터 OMY를 URDF(xacro)로 직접 작성하고, RViz → Gazebo(Classic) → ros2_control → IK 데모 순서로 구동한다. (ROS 2 Humble)
공식 `open_manipulator_description`은 Jazzy와 새 Gazebo(gz) 기준이라 Humble에서 바로 쓸 수 없다. 그래서 xacro, Gazebo 설정, 뎁스 카메라 플러그인을 새로 작성했다.

## 종류 (`variant` 인자)
| variant | 구성 |
|---|---|
| `f3m` (기본) | 6축 팔 + RH-P12-RN-A 그리퍼 + Intel RealSense D405 |
| `3m` | 6축 팔만 (손끝 장치 없음, link6 메시도 브래킷 없는 것) |

## 패키지
| 패키지 | 내용 |
|---|---|
| `omy_description` | URDF(xacro), 메시, RViz launch |
| `omy_bringup` | 컨트롤러 설정 (Gazebo / mock), Gazebo launch, RViz 전용 mock launch, 테이블 world, IK 데모, 카메라 집기 데모 |
| `omy_gazebo_plugins` | 그리퍼 링크 기구를 재현하는 Gazebo 플러그인 (C++) |

```
omy_description/urdf/
├── omy.urdf.xacro         최상위 (variant, use_sim, world 고정, 그리퍼/카메라 장착 위치)
├── omy_arm.xacro          치수 property, link0~6, joint1~6, 플랜지
├── rh_p12_rn_a.xacro      그리퍼 (손가락 2개 x 마디 2개, 구동 관절 1개 + mimic 3개)
├── d405.xacro             D405 본체/광학 프레임 + Gazebo 뎁스 센서 매크로
├── omy_ros2_control.xacro 팔 6축 + 그리퍼 (Gazebo 는 팔 velocity / 그리퍼 effort, mock 은 둘 다 position)
└── omy_gazebo.xacro       마찰/색상, gazebo_ros2_control, 카메라 센서
```
관성 매크로(`common.xacro`)와 `gazebo_link` 매크로는 `mobile_manipulator_description`에서 include해서 쓴다.

## 데이터 출처
| 항목 | 출처 |
|---|---|
| 관절 위치, 관절 범위 | OMY-F3M 사양서 / 도면 (joint3 ±150°, 나머지 ±360°) |
| 형상 (STL) | ROBOTIS 제공 CAD 메시 (`meshes/`, Apache 2.0) |
| 질량, COG, 관성 | ROBOTIS CAD 값 (팔), 그리퍼와 카메라는 바운딩 박스 기준 균일 밀도 공식 |
| 충돌 형상 | 각 STL의 바운딩 박스 (직접 계산) |
| 토크, 속도 한계 | DYNAMIXEL-Y 데이터시트. joint1·2는 YM080-230-A099-RH (61.4 Nm, 32.2 rpm), joint3~6은 YM070-210-A099-RH (31.7 Nm, 57.3 rpm) |
| D405 | 본체 42×42×23 mm, 시야각 87°, 측정 거리 0.07~0.5 m. 장착 위치는 ROBOTIS 모델 기준 link6 브래킷 |

## 관절 (0 자세 = 팔이 위로 곧게 선 자세, 손끝은 link6의 -y 방향)
| 관절 | 축 | 부모 기준 위치 (m) | 범위 | 구동기 |
|---|---|---|---|---|
| joint1 | z | (0, 0, 0.1715) | ±2π | YM080 |
| joint2 | y | (0, −0.1215, 0) | ±2π | YM080 |
| joint3 | y | (0, 0, 0.247) | ±150° | YM070 |
| joint4 | y | (0, 0.1215, 0.2195) | ±2π | YM070 |
| joint5 | z | (0, −0.113, 0) | ±2π | YM070 |
| joint6 | y | (0, 0, 0.1155) | ±2π | YM070 |
| 플랜지 (fixed) | - | (0, −0.103, 0) | - | - |
| end_effector_link (fixed) | - | 플랜지에서 (0, −0.141, 0) | - | - |

축 순서 z-y-y-y-z-y는 앞서 직접 만든 UR식 6축(`ur6_arm.xacro`)과 같다. 다른 점은 치수가 약 1.2배이고, 옆 방향 오프셋의 부호와 팔꿈치 오프셋의 위치(joint3 대신 joint4)이다.

## 빌드
```bash
cd ~/robot_ws
colcon build --symlink-install --packages-select omy_gazebo_plugins omy_description omy_bringup
source install/setup.bash     # GAZEBO_PLUGIN_PATH 에 omy_gazebo_plugins 가 추가됨
```
D405 카메라 시뮬레이션에는 `gazebo_plugins`가 필요하다.
```bash
sudo apt install ros-humble-gazebo-plugins
```

## 실행
Gazebo 와 RViz 를 함께 켜면 그래픽 부하가 큼 (Gazebo 화면 + D405 카메라 렌더링 + RViz 포인트클라우드 약 40만 점).
내장 그래픽에서는 화면이 깨지거나 느려질 수 있으므로 필요한 쪽만 켜는 것을 권장.

| 방식 | 명령 | 할 수 있는 것 |
|---|---|---|
| Gazebo 만 (기본) | `ros2 launch omy_bringup gazebo.launch.py` | 물리, 카메라, IK 데모, 카메라 집기 데모 |
| RViz 만 (mock) | `ros2 launch omy_bringup rviz_mock.launch.py` | 가짜 하드웨어 + 가짜 D405 (아래). IK 데모, 카메라 집기 데모 동작. 물리 없음 |
| RViz 만 (슬라이더) | `ros2 launch omy_description display.launch.py` | 슬라이더로 관절 하나씩 움직여 URDF 확인 |
| Gazebo 창 없이 + RViz | `ros2 launch omy_bringup gazebo.launch.py gui:=false start_rviz:=true` | Gazebo 기능은 그대로, 화면은 RViz 로만 (무거우면 RViz 에서 D405 PointCloud 표시 끄기) |

모든 launch 는 `variant:=3m` 으로 팔만 있는 모델을 띄울 수 있음.

```bash
# URDF 확인
xacro src/mobile_manipulator/omy_description/urdf/omy.urdf.xacro > /tmp/omy.urdf && check_urdf /tmp/omy.urdf

# 위 방식 중 하나를 띄운 뒤
ros2 control list_controllers                                   # 3개 active (3m 은 2개)
ros2 run omy_bringup omy_ik_demo.py                             # IK 데모 - Gazebo, RViz mock 둘 다 동작
ros2 run omy_bringup omy_pick_demo.py                           # 카메라 집기 데모 (빨간 큐브) - Gazebo, RViz mock 둘 다
ros2 run omy_bringup omy_pick_demo.py --ros-args -p color:=blue # 28.6° 돌아간 파란 큐브
ros2 service call /reset_scene std_srvs/srv/Trigger              # RViz mock 에서 큐브 원위치
```

### 카메라 영상 저장 (`save_d405.py`)
Gazebo, RViz mock 둘 다 같은 토픽이라 똑같이 동작. 저장 위치는 `~/robot_ws/d405_captures/<날짜_시각>/` (`-p output_dir:=...` 로 변경)

```bash
ros2 run omy_bringup save_d405.py                                   # 지금 프레임 한 장
ros2 run omy_bringup save_d405.py --ros-args -p mode:=video         # 동영상, Ctrl+C 로 끝냄
ros2 run omy_bringup omy_pick_demo.py --ros-args -p save:=true      # 집기 데모가 인식에 쓴 프레임 + detection.png
```

| 파일 | 내용 |
|---|---|
| `color.png` | 컬러 영상 |
| `depth_mm.png` | 16비트 깊이, mm 단위 (0 = 측정 없음). `cv2.imread(..., cv2.IMREAD_UNCHANGED)` 로 숫자 그대로 읽힘 |
| `depth_vis.png` | 깊이를 색으로 표시 (가까울수록 빨강). 색 범위는 그 프레임 깊이의 1~99 % (`camera_info.json` 의 `depth_vis_range_m`) |
| `points.ply` | 포인트클라우드 xyz + 색 (camera_depth_optical_frame 기준). MeshLab, CloudCompare, Open3D 로 열기 |
| `camera_info.json` | 해상도, fx/fy/cx/cy, 프레임, 시각 |
| `detection.png` | (집기 데모) 인식한 픽셀, 중심, link0 기준 위치와 yaw 를 영상 위에 표시 |
| `color.mp4`, `depth_vis.mp4` | (video) 프레임 속도는 들어오는 토픽 속도로 자동 설정, 깊이 색 범위는 처음 프레임 기준으로 고정 |

포인트클라우드 색은 컬러 영상에서 가져와서 Gazebo 의 R/B 뒤바뀜이 저장 파일에는 없음.

원본 메시지 그대로 녹화했다가 나중에 RViz 로 다시 보려면 rosbag (포인트클라우드는 1 프레임 약 6.5 MB 라 뺌)
```bash
ros2 bag record /d405/color/image_raw /d405/color/depth/image_raw /d405/color/camera_info /tf /tf_static /joint_states
ros2 bag play <bag 폴더>
```

### RViz 만 쓸 때의 가짜 D405 (`fake_d405_scene.py`)
RViz 는 보여주기만 하는 프로그램이라 카메라 영상을 만들 수 없다. Gazebo 가 하던 일을 이 노드가 대신한다.
- **장면** - table.world 와 같은 바닥, 테이블, 빨간/파란 큐브를 `/scene_markers` 로 RViz 에 표시
- **카메라** - `camera_depth_optical_frame` TF 에서 848×480 픽셀마다 광선을 쏴서(ray casting) 상자와 만나는 점을 계산.
  Gazebo 와 같은 토픽 (`/d405/color/image_raw`, `/d405/color/depth/image_raw`, `/d405/color/points`, `camera_info`), 약 4 Hz
- **집기** - 그리퍼가 닫히는 순간 손끝(end_effector_link) 3 cm 안에 큐브가 있으면 손에 붙이고, 열면 아래 테이블/바닥 위에 내려놓음

같은 자세에서 Gazebo 카메라와 비교하면 깊이 범위 (0.319 ~ 0.459 m), 빨간 큐브 픽셀 수 (3136), 측정 위치가 같았다.
Gazebo 와 달리 포인트클라우드 색(R/B)도 정상.

한계 - 물리가 없으므로 집기는 판정일 뿐 (손가락이 큐브를 통과해 끝까지 닫힘, 미끄러짐 없음, 큐브끼리 겹칠 수 있음).
로봇 자신은 카메라에 찍히지 않음. 렌더링에 CPU 를 쓰므로 무거우면 launch 대신 노드를 직접 띄우며 해상도를 낮춤
(`-p width:=424 -p height:=240`).
카메라 토픽 (`gazebo_plugins` 설치 후)
```
/d405/color/image_raw   /d405/color/depth/image_raw   /d405/color/points   (frame = camera_depth_optical_frame)
```

관절로 직접 명령하기
```bash
ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [joint1, joint2, joint3, joint4, joint5, joint6], points: [{positions: [0.286, 0.454, 0.361, 0.755, 1.571, -1.857], time_from_start: {sec: 3}}]}}"
ros2 action send_goal /gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 1.1, max_effort: 2.0}}"
```

## Gazebo 에서 집기가 되게 만든 방법
처음에는 팔과 그리퍼 모두 position 명령으로 구동했는데, 큐브를 들지 못하고 밀기만 했다. 원인과 해결은 다음과 같다.

| 문제 | 원인 | 해결 |
|---|---|---|
| 손가락이 물체를 뚫고 들어가고 쥐는 힘이 없음 | gazebo_ros2_control 의 position 명령은 매 스텝 관절 각도를 덮어씀 | 그리퍼를 **effort** 로 (`effort_controllers/GripperActionController`, 오차 → PID → 토크, `max_effort` 로 제한) |
| effort 로 바꾸자 손가락 속도가 NaN (발산) | 손가락 링크 관성 1e-5 kg m² 에 관절 감쇠 0.7 을 explicit 으로 적용 → 1 ms 스텝에서 불안정 | 손가락 관절에 `<implicitSpringDamper>` |
| 손끝 마디가 관절 한계까지 꺾임, 양쪽 손가락이 어긋나 물체를 한쪽으로 밀어냄 | ros2_control mimic 은 effort 모드에서 같은 토크만 줄 뿐 각도를 맞추지 않음 | **`omy_gazebo_plugins/parallel_finger`** - 매 스텝 rh_l1, rh_r2, rh_l2 각도를 rh_r1_joint 실제 각도에 맞춤 (실제 그리퍼의 링크 기구) |
| 쥐었는데 들어 올리면 미끄러져 떨어짐 | 팔도 position 이라 순간이동 → 링크 속도 0 → 마찰이 물체를 끌어올리지 못함 | 팔을 **velocity** 명령 + JTC PID (`gains`, p = 20) - 추종 오차 1 mm 이하 |
| 가끔 실패 (3회 중 2회) | 큐브 윗부분 1.8 cm 만 쥠 | 집는 높이를 1.5 cm 낮춤 → 3 cm 쥠 |

참고 - 설치된 gazebo_ros2_control (0.4.10) 의 position PID 는 플러그인 전체에 한 번에 적용되고 mimic 관절에는 명령이 전달되지 않아서 쓰지 않았다.

## 검증 결과 (2026-10-05)
| 항목 | 결과 |
|---|---|
| `check_urdf` (f3m / 3m) | 통과 |
| ROBOTIS 공식 URDF와 FK 비교 (무작위 1000 자세) | end_effector_link, camera_link 위치 오차 0 mm. 그리퍼 손끝 0.09 mm (공식은 π/2 대신 1.57을 씀) |
| 전체 질량 | 13.37 kg (사양 13.5 kg) |
| 수평 도달 거리 (joint1 축 ~ 플랜지 끝) | 0.582 m (사양 580 mm) |
| Gazebo 컨트롤러 (f3m 3개 / 3m 2개) | 모두 active |
| 팔 추종 오차 (velocity + PID, TF 측정) | 0.6 mm 이하 |
| IK 데모 (집어서 옮기기) | 5회 연속 성공, 놓은 위치 (0.359~0.362, −0.198) / 목표 (0.35, −0.20) |
| D405 토픽 | 848×480, 약 15 Hz, 포인트클라우드는 영상과 같은 순서 (organized) |
| D405 로 잰 큐브 위치 | 빨강 (0.450, −0.000, 0.140) yaw 0.0° / 실제 (0.45, 0, 0.14) 0° |
|  | 파랑 (0.380, 0.180, 0.140) yaw 29.0° / 실제 (0.38, 0.18, 0.14) 28.6° |
| 카메라 집기 데모 | 빨강, 파랑 모두 집어서 (0.35, −0.20) 에 내려놓음 |
| IK 시간 (데모 목표) | 24 ~ 250 ms (개선 전 yaw 90° 목표 3.6 s) |

## 알려진 한계
- **손끝 높이 한계 (구조)** - 그리퍼를 아래로 향하면 손끝 높이는 약 0.39 m 가 한계 (joint4 최대 높이 0.638 m - 손끝 길이 0.244 m). 소프트웨어 문제가 아니라 팔 치수에서 나오는 값.
- **포인트클라우드 색** - Gazebo Classic 뎁스 카메라 플러그인이 `rgb` 필드의 R/B 를 바꿔서 내보냄. 영상(`image_raw`, rgb8)은 정상이라 집기 데모는 색을 영상에서 판단하고 좌표만 포인트클라우드에서 가져옴. RViz 에서 포인트클라우드 색이 반대로 보이는 것도 이 때문.
- **놓는 위치 약 1 cm 오차** - 팔 오차가 아니라 쥐는 동안 큐브가 손끝 패드 폭 방향으로 밀린 만큼.
- **그리퍼 goal 이 끝나지 않음** - 물체를 쥐면 손가락이 미세하게 떨려서 stall 판정이 안 남. 데모는 3 s 기다린 뒤 다음 동작으로 넘어감 (힘은 계속 유지).
- **무작위 목표 IK** - 작업 공간 안 무작위 목표 200개 중 75% 해를 찾음 (실패한 것 중에는 도달 불가능한 목표도 섞여 있음), 실패 시 최대 약 7 s.
- 실제 하드웨어(Ethernet 컨트롤러, RealSense 드라이버)는 다루지 않음.
