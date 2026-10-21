# 시뮬레이션·하드웨어 실행 분리

`system.launch.py`는 실행 환경과 운용 모드를 독립적으로 선택한다.
기존 `ros2 launch vehicle_bringup system.launch.py mode:=navigation`은 계속 Gazebo로 실행한다.
실제 센서·모터 드라이버는 아직 구현되지 않았으며 `backend:=hardware`만 지정하면 명확한 오류로 종료한다.
시험용 모사 드라이버는 `hardware_driver:=mock`으로 명시적으로 선택한다.

## 실행

```bash
source env.sh

# 디스플레이 없는 시뮬레이션: GPU LiDAR의 실제 scan 생성까지 지원.
ros2 launch vehicle_bringup system.launch.py \
  backend:=simulation mode:=navigation \
  rviz:=false gazebo_gui:=false headless_rendering:=true

# Gazebo와 /clock을 사용하지 않는 모사 하드웨어.
ros2 launch vehicle_bringup system.launch.py \
  backend:=hardware hardware_driver:=mock mode:=navigation
```

| 인자 | 기본값과 의미 |
|---|---|
| `backend` | `simulation`, 또는 `hardware` |
| `mode` | `navigation`, 또는 `manual`, `mapping` |
| `use_sim_time` | `auto`: simulation=true, hardware=false. 모순되는 명시 값은 실행 전 거부 |
| `rviz` | `auto`: simulation에서 켜고 hardware에서 끈다. true/false로 개별 선택 |
| `gazebo_gui` | true. simulation에서만 사용 |
| `headless_rendering` | false. GPU LiDAR를 EGL로 렌더링할 때 true |
| `world` | simulation의 evaluation.sdf |
| `map` | localization의 evaluation.yaml. navigation 및 모사 scan 생성에 사용 |
| `spawn_x/y/z` | -2.0 / 0.0 / 0.02 |
| `spawn_roll/pitch/yaw` | 모두 0.0 |
| `safety_params_file` | bringup/config/safety.yaml |
| `slam_params_file`, `amcl_params_file` | localization의 기존 설정 |
| `nav2_params_file`, `controller_params_file` | planning/control의 기존 설정 |
| `lidar_config_file`, `simulation_config_file` | simulation/config의 센서 설정과 시뮬레이션 윤거 보정 |
| `manager_odom_topic` | odom. manager만의 오도메트리 경계를 시험할 때 사용 |

manual은 backend·모델·guard, mapping은 여기에 SLAM,
navigation은 localization·Nav2·manager를 실행한다. mapping은 자율 주행 스택을 실행하지 않는다.
spawn은 Gazebo world 기준이고 AMCL 초기 자세는 map 기준이다. 지도와 spawn을 바꾸면 초기 정합도 함께 확인한다.
GUI를 끄는 것과 센서 렌더링을 끄는 것은 별개이며, headless에서도 GPU/EGL 실행 환경이 필요하다.

## 패키지와 모델

- description: 형상·센서 장착 TF·메시·모델 발행·RViz 리소스.
- simulation: 월드·Gazebo 실행·spawn·bridge·시뮬레이션 센서 설정·구동 보정·검증.
- drivers: 하드웨어 진입점과 명시적으로 선택하는 모사 드라이버.
- bringup: backend 선택과 공통 guard·localization·mapping·Nav2 조합.

`vehicle_simulation → vehicle_description` 방향으로 의존하며 description은 Gazebo 패키지에 필수 의존하지 않는다.
선택하지 않은 backend의 패키지 조회도 실행하지 않는다.
`vehicle_description gazebo.launch.py`는 새 `vehicle_simulation gazebo.launch.py`로 전달하는 호환 진입점이다.
이 명령에는 simulation 패키지를 별도로 설치해야 한다.

공통 xacro의 기본값은 `simulation:=false`다. 직접 Gazebo 모델을 생성하는 기존 도구는 다음 인자를 추가한다.

```bash
xacro <vehicle_description>/urdf/vehicle.urdf.xacro simulation:=true
```

`gazebo.xacro`는 모델 가까이에 두되 simulation일 때만 읽는다.
물리 윤거 0.1203 m는 description/config/vehicle.yaml에 남고, 기존 시뮬레이션 보정 계수 1.57은
simulation/config/simulation.yaml로 이동했다. LiDAR 측정 설정은 simulation/config/lidar.yaml로 이동했다.
기존 형상·관성·바퀴 배치·보정값은 재튜닝하지 않았다.

## 시간·명령·TF 계약

backend의 시간 선택은 사용자 Nav2/controller YAML보다 우선한다.
planner/controller의 공통 노드 생성 함수와 YAML 변환을 통합·독립 launch에서 함께 사용한다.
중첩 costmap에도 bool 값을 명시적으로 추가하므로 사용자 설정에서 시간 키를 생략해도 동작한다.
기존 독립 planner/controller/costmap launch는 개별 디버깅용이며 통합 Nav2와 함께 실행하지 않는다.
독립 launch는 `use_sim_time`과 `nav2_params_file`, controller는 `controller_params_file`도 받는다.
기존 단독 실행 호환성을 위해 시간 기본값은 true다.

센서 stamp와 TF에는 ROS 시각을 사용한다. guard·manager·수신기의 기존 steady-clock timeout은 유지한다.

| 계약 | 발행 책임 |
|---|---|
| `/scan`, `/odom`, `/joint_states` | 선택한 backend |
| `odom → base_footprint` | 선택한 backend의 단일 오도메트리 공급자 |
| `base_footprint → base_link → 센서` | robot_state_publisher |
| `map → odom` | navigation의 AMCL 또는 mapping의 SLAM |
| `/cmd_vel` | command_guard의 기존 최종 출력 |
| `/actuator/command` | command_guard, backend 수신 |
| `/actuator/status`, `/actuator/arm` | backend |
| `/clock`, `/ground_truth/odom` | simulation 전용 |

실제 구동 경로는 `/actuator/command`다. backend가 `/cmd_vel`을 추가로 소비하여 guard의 토큰 검사를 우회하지 않는다.
Nav2의 `/cmd_vel_auto → manager → /navigation_command → guard` 경로는 유지한다.
bridge는 기존 기본 reliable/volatile QoS와 GZ→ROS 방향을 유지한다.
모사 센서의 best-effort/volatile QoS는 공통 sensor-data 구독자와 호환된다.
scan bridge를 별도 프로세스로 유지하여 기존 센서 중단 시험을 사용할 수 있다.

## 모사 드라이버

모사 드라이버는 카메라 pan/tilt의 중립 joint state도 제공하여 광학 frame까지 TF를 연결한다.
모사 드라이버는 wall clock, P5 8-bit trinary PGM 지도와 일치하는 raycast scan,
명령 기반 평면 운동학, joint state, odom과 TF를 제공한다.
지도 origin의 회전은 지원하지 않으며 지원하지 않는 지도 형식은 오류로 처리한다.
모사 scan은 odom 원점과 map 원점의 차량 자세가 일치하는 시험용 설정을 사용한다.

구동기 인터페이스는 session·generation·sequence·heartbeat freshness·명령 만료와
ARM/DISARM/ESTOP/CLEAR_ESTOP을 처리한다. 시작·재시작 상태는 disarmed다.
이 구현은 ROS 연결 시험용이며 모터 전자회로·충돌·실제 제동·물리적 watchdog 보장을 검증하지 않는다.
정상 센서 장애 후에는 기존 안전 정책에 따라 정지를 유지하고 새 목표가 필요하다.
정지·액션 결과가 불확실하여 recovery가 required인 경우에는 명시적인 recover도 필요하다.

장애 주입은 `/mock_hardware`의 `publish_scan`, `publish_odom`, `publish_status` bool 파라미터로 한다.
`/mock_hardware/restart_actuator` Trigger 서비스는 pose를 유지하며 구동기 세션을 새로 만든다.
모사 하드웨어는 `/clock`과 ground truth를 발행하지 않는다.

## 검증 실행

```bash
source env.sh
colcon build --symlink-install
source env.sh
export ROS_DOMAIN_ID=91
export ROS_LOG_DIR=/tmp/vehicle-005-colcon-logs
colcon test --packages-skip vehicle_perception
colcon test-result --verbose
```

캐시 없는 별도 설치 빌드는 `--cmake-args -DPython3_EXECUTABLE=/usr/bin/python3`를 지정하여
ROS 인터페이스 생성에 배포판 Python과 apt 의존성을 사용한다. 가상환경에 ROS 빌드 도구를 별도 설치하지 않는다.

vehicle_perception은 테스트가 없는 패키지라 제외한다. 기본 시험의 opt-in skip은 실제 통과로 계산하지 않는다.
backend 통합 시험은 프로세스와 ROS domain을 격리하고 설치된 system.launch.py를 실행한다.
시험용 domain은 hardware 30~32, simulation 40~42, 주행 회귀는 60~79를 사용한다.
Linux의 기본 ephemeral UDP 포트와 겹치지 않는 0~101 범위에서 분리한다.
[ROS 2 domain 선택 기준](https://raw.githubusercontent.com/ros2/ros2_documentation/jazzy/source/Concepts/Intermediate/About-Domain-ID.rst)을 따른다.
정리 함수는 정확히 일치하는 시험 partition의 프로세스만 종료하며, launch와 분리된 Gazebo 자식도 정리한다.
먼저 해당 partition의 `/server_control`에 stop을 요청한 뒤 launch와 잔여 프로세스를 종료한다.
이 종료 서비스는 [Gazebo Server API](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1Server.html)를 따른다.
분리된 자식 종료와 다른 partition 보존을 별도 시험한다.

```bash
source env.sh
python3 -c "import os, pytest; os.environ['RUN_BACKEND_INTEGRATION']='1'; raise SystemExit(pytest.main(['-q', 'src/vehicle_bringup/test/test_backend_integration.py']))"
```

정상 목표는 Nav2 lifecycle 활성화·센서 최신성·정지 준비를 확인한 후 제출한다.
모사 시험은 수동 명령의 arm·timeout·비상정지와 Nav2 목표, scan/odom/status 중단 및 구동기 재시작을 검증한다.
TF 시험은 관측한 parent/child 관계와 예상 발행 노드 집합을 확인한다.
동일 edge의 DDS writer별 소유권을 직접 추적하는 시험은 아니다.

기존 Gazebo 주행·안전·복구·센서·watchdog 시험은 다음 도구로 매 case 새 시스템에서 실행할 수 있다.
Gazebo 시험은 서로 병행하지 않는다. JUnit과 실행 로그만 출력 경로에 저장한다.
출력 디렉터리는 새 경로여야 한다.
실패한 case도 기록한 뒤 나머지를 실행하며, 하나라도 실패하면 최종 종료 코드는 1이다.

```bash
source env.sh
python3 src/vehicle_simulation/test/run_backend_regression.py \
  --output log/backend_regression
```

기본 case는 s01/s02/s03 각 3회, s05/s06 각 1회, 안전 전환·복구·센서·watchdog다.
`--cases s02`처럼 일부만 선택할 수 있다. [회귀 기준](t9_4_regression.md)에 따라
액션 결과·상태 전이·정지를 판정한다. 도착 오차와 접촉 여부는 자동 판정하지 않는다.
`--startup-timeout`은 준비 대기 한도이며 기본 120초다. 목표 제출 이후의 기존 120초 판정과 구분한다.
