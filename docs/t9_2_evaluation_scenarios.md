# T9.2 반복 평가 시나리오

## 목적

동일한 월드·지도·설정·시작 조건으로 평가를 반복한다.
각 시나리오는 준비, 실행, 기대 결과, 정리 절차를 명시한다.

2026-10-19 기준 s01~s08의 실행 절차와 수동 평가 결과를 작성했다.
2026-10-20 위치 오차 합격 기준을 0.10 m로 확정하고, 도착 정밀도 튜닝 후
s01~s03 각 3회와 진행 판정 변경 후 s03 추가 3회의 기록을 확인했다.
현재 기준·최종 설정·결과는 [1차 시뮬레이션 검증 결과](simulation_validation.md)를 따른다.
아래의 2026-10-19 결과는 당시 기준과 설정으로 수행한 이력이다.

## 공통 조건

- 각 실행 전 기존 시스템을 종료하고 새로 시작한다.
- 이전 목표, 추가 장애물, 비상정지 상태를 다음 실행에 남기지 않는다.
- 기본 월드: `vehicle_simulation/worlds/evaluation.sdf`
- 기본 지도: `vehicle_localization/maps/evaluation.yaml`
- 기본 설정:
  - `vehicle_bringup/config/safety.yaml`
  - `vehicle_localization/config/amcl.yaml`
  - `vehicle_planning/config/nav2.yaml`
  - `vehicle_control/config/controller.yaml`
- 목표 위치와 방향은 `map` 좌표계의 m, rad 단위를 사용한다.
- Gazebo 좌표와 `map` 좌표의 정렬을 확인한 뒤 목표를 확정한다.
- 지도·스캔 정합과 TF 연결을 확인하고 목표를 전송한다.
- 데이터 중단 시험에서는 중단 대상 이외의 데이터가 계속
  수신되는지도 확인한다.

## 공통 판정 기준

아래 수치는 이슈 #3의 현재 평가 기준이다. 위치 기준은 2026-10-20에
초기 제안 0.15 m에서 0.10 m로 확정했다.

- 정상 목표는 Nav2 성공 결과와 실제 정지를 함께 확인한다.
- 목표 전송 서비스의 `success=true`는 요청 제출이며 도착 성공이 아니다.
- 정상 도착 위치 오차는 0.10 m 이하이다.
- 정상 도착 방향 오차는 10도 이하이다.
- 최종 정지는 선속도 0.02 m/s 이하,
  각속도 절댓값 0.05 rad/s 이하를 1초 유지하는 것으로 판정한다.
- 정상 주행 중 충돌은 0회여야 한다.
- 실패·취소 시험도 결과와 실제 정지를 함께 확인한다.
- 실행 제한 시간을 넘기면 평가 실패로 기록한다.
  이후 시험 정리를 위한 취소를 시스템의 자발적 실패 반환과 구분한다.
- 실행하지 않은 시험과 skip된 테스트는 통과로 기록하지 않는다.

제어 설정의 위치 허용 오차 0.05 m·방향 허용 오차 약 3도는 튜닝값이며,
평가 합격 기준인 위치 0.10 m·방향 10도와 구분한다.

## 시나리오 목록

| ID | 시나리오 | 핵심 확인 |
|---|---|---|
| s01 | 직선 목표 | 성공 결과, 위치 오차, 정지 |
| s02 | 회전이 필요한 목표 | 성공 결과, 위치·방향 오차, 정지 |
| s03 | 정적 장애물 우회 | 우회 경로 주행, 충돌 없음, 도착 |
| s04 | 주행 중 장애물 추가 | 장애물 관측, 경로·속도 대응, 충돌 없음 |
| s05 | 도달 불가능한 목표 | 제한 시간 내 실패 반환, 정지 |
| s06 | 주행 취소 | 취소 결과, 정지, 잔여 명령 해제 |
| s07 | 명령 중단 | 타임아웃에 따른 0 명령 출력, 실제 정지 |
| s08 | 센서 데이터 중단 | 이상 감지, 주행 차단, 복구 후 자동 재출발 없음 |

## 반복 횟수

- s01~s03은 각각 독립적으로 3회 실행한다.
- s04는 주행 중 추가 조건으로 독립적으로 3회 실행한다.
- s05~s08은 먼저 각각 1회 실행한다.
- 매 실행마다 통과·실패·미실행과 관측 내용을 기록한다.

## 기본 주행 설정과 변경 이력

다음은 2026-10-19 수동 평가에서 조정한 기본 주행 설정이다.
이후 도착 정밀도·AMCL 갱신·진행 판정 설정을 조정했다.
현재 최종값은 [검증 요약의 설정 표](simulation_validation.md)를 참고한다.
설정을 변경한 뒤의 결과는 이전 설정에서의 결과와 구분한다.

| 파일 | 항목 | 값 |
|---|---|---|
| `vehicle_control/config/controller.yaml` | `follow_path.forward_prune_distance` | `0.6` m |
| 같은 파일 | `follow_path.PathAlign.scale` | `32.0` |
| 같은 파일 | `follow_path.GoalAlign.scale` | `24.0` |
| 같은 파일 | `follow_path.sim_time` | `1.5` s |
| 같은 파일 | `follow_path.limit_vel_cmd_in_traj` | `true` |
| `vehicle_planning/config/nav2.yaml` | global/local `footprint_padding` | `0.05` m |
| 같은 파일 | global/local `inflation_radius` | `0.55` m |
| 같은 파일 | global/local `cost_scaling_factor` | `3.0` |
| `vehicle_bringup/config/safety.yaml` | `command_timeout_sec` | `0.5` s |

`vehicle_description/launch/gazebo.launch.py`는 스캔 전용 `scan_bridge`를
별도 프로세스로 실행한다. s08에서는 이 프로세스만 일시 정지한다.

## 정상 주행 평가: s01~s03

### 좌표와 목표

현재 launch의 Gazebo 시작 평면 자세 `(x, y, yaw)`는 `(-2, 0, 0)`이고,
AMCL의 `map` 기준 초기 자세는 `(0, 0, 0)`이다.
저장된 지도에서도 중앙 장애물이 대략 `x=2, y=0`에 있으므로,
목표 선정에는 `x_map ≈ x_world + 2`, `y_map ≈ y_world` 관계를 사용한다.
이는 파일에서 확인한 배치 관계이며 실제 실행 중의 정합을 보장하지 않는다.
지도 YAML의 `origin`은 지도 이미지 원점이며 차량 초기 자세가 아니다.

2026-10-20 실제 자세 기반 오차 평가는
[오프라인 형상 정합](t9_3_world_map_alignment.md)에서 확인한
`world→map=(2.036, 0.020, 0)`을 공통으로 사용했다.
셀 중심 정합의 불확실성을 포함한 참고값이며 목표에 맞춰 변환을 바꾸지 않는다.

각 실행의 시작 자세는 `map` 기준 `(0, 0, 0)`으로 한다.
시작 위치 오차 0.15 m 이하, 방향 오차 10도 이하와 실제 정지를 확인한다.
조건이 맞지 않으면 목표를 보내지 않고 초기 위치와 지도·스캔 정합을 확인한다.
시작 위치 허용 오차는 도착 위치 합격 기준인 0.10 m와 별개다.

| ID | 목표 x [m] | 목표 y [m] | 목표 yaw [rad] | 의도 |
|---|---|---|---|---|
| s01 | 1.0 | 0.0 | 0.0 | 장애물 앞 열린 공간에서 전방 1 m 이동 |
| s02 | 0.0 | 1.0 | 1.5707963267948966 | +y 방향으로 이동하려면 회전이 필요하며, 도착 방향은 +y |
| s03 | 4.0 | 0.0 | 0.0 | 시작점과 목표 사이의 중앙 장애물을 우회 |

저장된 지도에서 시작점과 세 목표의 셀은 빈 공간이다.
이는 정적 확인 결과이며 경로 계획·주행 성공은 아래 절차로 검증한다.

### 공통 준비

빌드된 워크스페이스 루트에서 실행한다.
각 터미널에서 먼저 환경을 로드한다.

```bash
source env.sh
```

터미널 A에서 월드·지도·설정을 명시해 시스템을 시작한다.

```bash
ros2 launch vehicle_bringup system.launch.py \
  mode:=navigation \
  world:="$PWD/src/vehicle_simulation/worlds/evaluation.sdf" \
  map:="$PWD/src/vehicle_localization/maps/evaluation.yaml" \
  safety_params_file:="$PWD/src/vehicle_bringup/config/safety.yaml" \
  amcl_params_file:="$PWD/src/vehicle_localization/config/amcl.yaml" \
  nav2_params_file:="$PWD/src/vehicle_planning/config/nav2.yaml" \
  controller_params_file:="$PWD/src/vehicle_control/config/controller.yaml"
```

RViz에서 지도·스캔·차체의 위치와 방향이 일치하는지 확인한다.
Nav2 서버가 active 상태이고 `navigation_manager`가 `waiting`인지 확인한다.
이어서 아래 명령으로 TF와 속도를 확인한다.

터미널 B에서 초기 위치와 방향을 확인한다.
TF 출력이 계속 갱신되고 시작 자세 조건을 만족해야 한다.

```bash
ros2 run tf2_ros tf2_echo map base_footprint \
  --ros-args -p use_sim_time:=true
```

터미널 C에서 오도메트리 속도를 지속적으로 관측한다.
평면 속도는 `hypot(linear.x, linear.y)`, 회전 속도는 `abs(angular.z)`로 판단한다.
시작 전과 종료 시 공통 정지 조건을 확인한다.

```bash
ros2 topic echo /odom --field twist.twist
```

이후 목표 전송은 터미널 D에서 수행한다.
각 실행의 제한 시간은 목표 전송 서비스가 `success=true`를 반환한 시점부터
최종 상태와 정지 확인까지 실시간으로 120초이다.
이는 평가 측 초기 상한이며 Nav2에 120초 타임아웃을 설정하는 것은 아니다.
평가 중에는 시뮬레이션을 일시 정지하지 않는다.
전송이 거부되면 이유를 기록하고 해당 시도를 정상 도착 성공 횟수에 포함하지 않는다.

### s01: 직선 목표

공통 준비 후 다음 목표를 한 번 전송한다.

```bash
ros2 service call /navigation_manager/set_goal \
  vehicle_interfaces/srv/SetNavigationGoal \
  '{x: 1.0, y: 0.0, yaw: 0.0}'
```

전방의 열린 공간을 따라 이동하고 목표에서 정지하는지 확인한다.
중앙 장애물 앞에 있는 목표이므로 장애물을 우회할 필요가 없다.

### s02: 회전이 필요한 목표

시스템을 재시작하고 공통 준비를 다시 수행한 뒤 다음 목표를 한 번 전송한다.

```bash
ros2 service call /navigation_manager/set_goal \
  vehicle_interfaces/srv/SetNavigationGoal \
  '{x: 0.0, y: 1.0, yaw: 1.5707963267948966}'
```

차체가 회전하여 +y 방향으로 이동하고, 마지막에 +y 방향을 향해 정지하는지 확인한다.
회전과 전진의 세부 순서를 고정하지 않고 실제 경로와 최종 자세를 평가한다.

### s03: 정적 장애물 우회

시스템을 재시작하고 공통 준비를 다시 수행한 뒤 다음 목표를 한 번 전송한다.

```bash
ros2 service call /navigation_manager/set_goal \
  vehicle_interfaces/srv/SetNavigationGoal \
  '{x: 4.0, y: 0.0, yaw: 0.0}'
```

중앙 장애물은 `map` 좌표로 대략 `x=1.7~2.3, y=-0.6~0.6`을 차지하며,
시작점과 목표를 잇는 직선을 막고 있다.
RViz의 계획 경로와 Gazebo의 실제 이동이 장애물을 우회하는지 확인한다.
우회 방향은 고정하지 않으며 중앙 장애물이나 벽과 접촉하지 않아야 한다.

### 공통 결과 확인과 종료

정상 종료 시 터미널 A의 `navigation_manager`에 다음 로그가 출력된다.

```text
Action result: succeeded
State: stopping
Vehicle stop confirmed
State: succeeded
```

이와 함께 TF 출력으로 최종 위치·방향 오차를 확인하고,
오도메트리 속도가 정지 조건을 1초 이상 만족하는지 확인한다.
위치 오차는 `hypot(x - goal_x, y - goal_y)`이며 방향 오차는
`abs(atan2(sin(yaw - goal_yaw), cos(yaw - goal_yaw)))`로 ±π 경계를 고려한다.
방향 허용값 10도는 약 0.17453 rad이다.
정지 유지 시간을 오도메트리 timestamp로 확인하려면
`ros2 topic echo /odom`으로 헤더를 포함해 관측한다.

TF 기반 오차는 위치 추정상의 오차로 기록한다.
Gazebo 기준 위치에 따른 실제 도착 오차 측정·기록은 T9.3에서 정비한다.
접촉 여부는 Gazebo에서 눈으로 확인했다고 명시하며 자동 접촉 검출 결과로 취급하지 않는다.

제한 시간을 넘긴 시도는 실패로 기록하고, 여전히 주행 중이면 다음 서비스로 취소를 요청한다.

```bash
ros2 service call /navigation_manager/cancel std_srvs/srv/Trigger '{}'
```

취소 요청 응답에 더해 최종 결과와 실제 정지를 확인한다.
정지를 확인할 수 없으면 미확인으로 기록한다.
확인 후 터미널 A와 관측 명령을 Ctrl+C로 종료하고 Gazebo도 종료되었는지 확인한다.
다음 시도는 시스템 시작부터 다시 진행한다.

### 2026-10-19 평가 기록

각 시나리오를 3회씩 실행하고 다음 형식으로 실행마다 한 줄을 추가한다.
결과는 모든 항목을 관측한 뒤 기록하며 위치·방향 오차는 TF 기반 추정값으로 명시한다.
아래 표의 통과는 당시 위치 기준 0.15 m에 따른 판정이다.
0.10 m 기준의 최신 평가로 소급하지 않으며, 새 결과는
[2026-10-20 검증 요약](simulation_validation.md)에 구분해 기록했다.

| ID | 회차 | 결과 | 수락→정지 확인 [s] | 위치 오차 [m] | 방향 오차 [deg] | 1초 정지 | 접촉(육안) | 비고 |
|---|---|---|---|---|---|---|---|---|
| s01 | 1 | 통과 | 7.02 | 0.1166 | 2.41 | 확인 | 없음(사용자 육안) | 정지 관측 3.82 s, 최종 설정 |
| s01 | 2 | 통과 | 7.96 | 0.1193 | 7.87 | 확인 | 없음(사용자 육안) | 정지 관측 1.62 s, 최종 설정 |
| s01 | 3 | 통과 | 7.55 | 0.1224 | 7.95 | 확인 | 없음(사용자 육안) | 정지 관측 3.80 s, 최종 설정 |
| s02 | 1 | 통과 | 10.74 | 0.0836 | 7.82 | 확인 | 없음(사용자 육안) | 정지 관측 1.30 s, 최종 설정 |
| s02 | 2 | 통과 | 11.09 | 0.0774 | 7.79 | 확인 | 없음(사용자 육안) | 정지 관측 3.24 s, 최종 설정 |
| s02 | 3 | 통과 | 10.80 | 0.0777 | 7.65 | 확인 | 없음(사용자 육안) | 정지 관측 3.96 s, 최종 설정 |
| s03 | 1 | 통과 | 29.31 | 0.1164 | 8.63 | 확인 | 없음(사용자 육안) | 정지 관측 3.60 s, 최종 설정 |
| s03 | 2 | 통과 | 29.81 | 0.1109 | 7.44 | 확인 | 없음(사용자 육안) | 정지 관측 4.56 s, 최종 설정 |
| s03 | 3 | 통과 | 29.76 | 0.1360 | 9.12 | 확인 | 없음(사용자 육안) | 정지 관측 4.14 s, 최종 설정 |

시간 열은 로그의 `Goal accepted`부터 `Vehicle stop confirmed`까지의 실시간이다.
서비스 제출부터 세는 공통 제한 시간과는 시작 시점이 다르다.
s01~s03의 TF 오차는 각 `navigation_allowed` true→false 전환 약 1.2초 후의
정지 상태에서 `map -> odom -> base_footprint`를 합성해 계산했다.
정지 관측 시간은 결과 이후 기록된 연속 정지 구간의 오도메트리 timestamp 기준이다.

## s04: 주행 중 장애물 추가

### 준비와 실행

기본 navigation 모드로 시스템을 새로 시작하고 공통 준비를 수행한다.
사용할 모델은 `src/vehicle_simulation/worlds/dynamic_obstacle.sdf`이며,
크기는 `0.5 × 0.4 × 1.0` m인 정적 장애물이다.
이 시험은 움직이는 장애물 추적이 아니라 주행 중 새로 생긴 장애물에 대한 대응이다.

관측 터미널에서 `map -> base_footprint` TF와 `/odom`을 확인한다.
목표 전송 터미널에서 다음 목표를 보낸다.

```bash
ros2 service call /navigation_manager/set_goal \
  vehicle_interfaces/srv/SetNavigationGoal \
  '{x: 0.0, y: 1.5, yaw: 1.5707963267948966}'
```

주행 중 장애물 추가용 터미널에서 다음 명령을 한 번 실행한다.
반복 시에는 출발 회전 중 yaw가 처음 `0.3 rad`를 넘고 `y_map < 0.2 m`인
시점을 기준으로 한다. 주입 시각과 당시 TF 자세를 기록한다.
수동 명령 실행에는 지연이 있으므로 기준을 놓친 실행은 별도로 기록한다.

```bash
ros2 run ros_gz_sim create \
  -world evaluation \
  -file "$PWD/src/vehicle_simulation/worlds/dynamic_obstacle.sdf" \
  -name dynamic_obstacle \
  -x -2.0 -y 0.8 -z 0.5
```

장애물은 Gazebo 좌표에서 중심 `(-2.0, 0.8)`에 생성되며,
대략 `map` 중심 `(0.0, 0.8)`에 해당한다.
실제 지도 정합은 실행 중 스캔과 TF로 확인한다.

### 판정과 정리

- 장애물이 스캔과 장애물 지도에 관측되고 경로·속도가 대응한다.
- 장애물과 외벽에 접촉하지 않고 우회한다.
- 120초 이내 `succeeded`와 공통 도착·정지 기준을 만족한다.
- 장애물을 목표 전송 전에 추가한 실행은 사전 배치 비교 실행으로 기록한다.
- 3회 모두 시스템 재시작부터 수행하여 이전 장애물과 목표를 제거한다.

문제가 재현되면 목표 전송 전에 다음 기록을 시작하고,
최종 정지 확인 후 기록 터미널만 Ctrl+C로 종료한다.
다음 기록에서는 기존 경로와 겹치지 않는 새 출력 디렉터리를 사용한다.

```bash
ros2 bag record -o /tmp/s04_dynamic_probe \
  /scan /tf /tf_static /clock \
  /plan /transformed_global_plan /local_plan \
  /local_costmap/costmap /local_costmap/published_footprint \
  /cmd_vel_auto /cmd_vel /odom /ground_truth/odom /navigation_allowed
```

## s05: 도달 불가능한 목표

기본 navigation 모드로 새로 시작하고, 공통 준비 후 폐쇄 구역 내부를 목표로 보낸다.

```bash
ros2 service call /navigation_manager/set_goal \
  vehicle_interfaces/srv/SetNavigationGoal \
  '{x: 4.0, y: 1.1, yaw: 0.0}'
```

현재 지도에서 이 목표 셀은 미관측 영역이고, planner의 `allow_unknown`은 `false`다.
따라서 `GOAL_OCCUPIED(206)`로 거부될 수 있다.
해당 결과는 **미관측 목표 거부**로 기록한다.
빈 공간으로 확인된 목표까지의 경로 단절만을 검증한 결과로 취급하지 않는다.

120초 이내 실패 결과를 반환하고, 접촉 없이 공통 정지 기준을 만족하면 통과다.
예상되는 최종 흐름은 다음과 같다.

```text
Action result: failed
State: stopping
Navigation failed: status=6, error_code=206, ...
Vehicle stop confirmed
State: failed
```

제한 시간 초과 시 공통 취소 명령으로 정리하고, 자발적 실패 반환과 구분한다.

## s06: 주행 중 취소

기본 navigation 모드로 새로 시작하고 공통 준비 후 직선 목표를 보낸다.

```bash
ros2 service call /navigation_manager/set_goal \
  vehicle_interfaces/srv/SetNavigationGoal \
  '{x: 1.0, y: 0.0, yaw: 0.0}'
```

`running` 상태에서 차량이 움직이는 것을 확인하고, 도착 전에 취소한다.

```bash
ros2 service call /navigation_manager/cancel std_srvs/srv/Trigger '{}'
```

요청 수락 이후 `Action result: canceled`, `Vehicle stop confirmed`,
최종 `State: canceled`를 확인한다. 실제 속도도 공통 정지 기준을 만족해야 한다.
새 목표를 보내기 전까지 정지가 유지돼야 한다.
취소 서비스의 `success=true`만으로 통과 처리하지 않는다.

## s07: 명령 중단 시 정지

### 준비와 실행

기존 시스템을 종료하고 **manual 모드**로 새로 시작한다.
이 시험은 수동 명령 발행을 중단하여 공통 command guard의 타임아웃을 확인한다.

```bash
ros2 launch vehicle_bringup system.launch.py mode:=manual
```

별도 터미널에서 `/odom`과 `/cmd_vel`을 각각 관측한다.

```bash
ros2 topic echo /odom
```

```bash
ros2 topic echo /cmd_vel
```

명령용 터미널에서 10Hz로 전진 명령을 보낸다.

```bash
ros2 topic pub -r 10 /cmd_vel_manual geometry_msgs/msg/Twist \
  '{linear: {x: 0.1}, angular: {z: 0.0}}'
```

차량이 움직인 뒤 1~2초 후 **명령 발행 터미널만 Ctrl+C**로 종료한다.
시스템, Gazebo, 관측 명령은 계속 실행한다.

### 판정과 정리

- 명령 중단 전 `/cmd_vel`과 실제 움직임이 확인돼야 한다.
- 마지막 입력 수신 후 설정된 0.5초 타임아웃과 0.05초 검사 주기에 따라
  0 명령을 출력하고 `속도 명령 타임아웃으로 정지합니다`를 기록한다.
- `/cmd_vel`의 모든 성분이 0으로 바뀌고 유지된다.
- 실제 속도가 공통 정지 기준을 1초 이상 만족하고 재출발하지 않는다.

수동 모드에서는 `navigation_manager`의 정지 확인 로그가 없으므로
`/odom`의 timestamp와 속도로 실제 정지를 확인한다.
육안·터미널 확인만 한 경우 타임아웃의 정확한 실측 시간은 미측정으로 기록한다.
관측을 마친 뒤 시스템을 종료한다.

## s08: 스캔 중단과 복구

### 준비

기본 navigation 모드로 새로 시작하고 공통 준비를 수행한다.
스캔 전용 `scan_bridge`가 별도 프로세스인지 확인한다.
이 브리지에는 `/scan`만 있어야 하며 `/odom`, `/clock`, `/cmd_vel`, `/tf`는
기존 브리지에서 계속 처리돼야 한다.

명령용 터미널에서 프로세스를 찾고, 해당 실행의 PID를 입력해 저장한다.
`ps` 출력의 인자가 `/scan` 브리지와 `__node:=scan_bridge`인지 확인한다.
시스템 재시작 후에는 PID를 다시 확인한다.

```bash
pgrep -af '[p]arameter_bridge.*__node:=scan_bridge'
read -r -p 'scan_bridge PID: ' scan_bridge_pid
ps -p "$scan_bridge_pid" -o pid,args
```

기록용 터미널에서 목표 전송 전에 기록을 시작한다.

```bash
ros2 bag record -o /tmp/s08_scan_loss \
  /scan /odom /ground_truth/odom /clock \
  /cmd_vel_auto /cmd_vel /navigation_allowed
```

### 실행

PID를 저장한 명령용 터미널에서 직선 목표를 전송한다.

```bash
ros2 service call /navigation_manager/set_goal \
  vehicle_interfaces/srv/SetNavigationGoal \
  '{x: 1.0, y: 0.0, yaw: 0.0}'
```

차량이 움직이기 시작하면 도착 전에 스캔 브리지만 일시 정지한다.

```bash
kill -STOP "$scan_bridge_pid"
```

`Scan is unavailable, invalid, or stale` 안전 오류와 취소·정지를 확인한다.
`Action result: canceled` 뒤 최종 상태가 `failed`인 것은
센서 이상으로 안전 취소했기 때문이다.

```text
Navigation safety failure: Scan is unavailable, invalid, or stale
State: canceling
...
Action result: canceled
State: stopping
Vehicle stop confirmed
State: failed
```

정지 확인 후 같은 터미널에서 스캔 브리지를 재개한다.

```bash
kill -CONT "$scan_bridge_pid"
```

새 목표를 보내지 않고 스캔 수신 복구와 3초 이상 정지 유지를 관측한다.
이후 기록 터미널만 Ctrl+C로 종료하여 bag 파일을 정상적으로 닫는다.

### 판정과 정리

- 중단 직전 차량이 주행 중이어야 한다.
- `/scan` 중단 동안 `/odom`과 `/clock`은 계속 수신되고 timestamp도 증가한다.
- 스캔 최신성 한계 0.5초와 안전 검사 주기 0.1초에 따라 이상을 감지한다.
- `/navigation_allowed`가 false가 되고 `/cmd_vel`이 0으로 바뀐다.
- 실패 결과와 공통 정지 기준을 확인한다.
- 복구 후 스캔 timestamp가 다시 증가해도 `/navigation_allowed`는 false이고,
  새 목표 없이 차량이 재출발하지 않는다.

스캔 브리지가 일시 정지된 채 남았다면 종료 전에 `SIGCONT`로 재개한다.
정리 후 시스템을 종료한다.

## 2026-10-19 확인 결과

아래는 대화에서 보고된 결과와 직접 읽은 로그·bag의 관측을 구분한 기록이다.
측정하지 않은 위치·방향 오차와 주입 시각을 성공 보고만으로 채우지 않는다.

| ID | 확인 결과 | 근거와 범위 |
|---|---|---|
| s01 | 최종 설정으로 3회 통과 | 한 bag에 담긴 독립적인 3회 주행·시계 초기화와 로그를 확인. TF 위치 오차 최대 0.1224 m, 방향 오차 최대 7.95°, 실제 정지 1.62 s 이상 |
| s02 | 최종 설정으로 3회 통과 | TF 위치 오차 최대 0.0836 m, 방향 오차 최대 7.82°, 실제 정지 1.30 s 이상. 접촉 없음은 사용자 육안 확인 |
| s03 | 최종 설정으로 3회 통과 | TF 위치 오차 최대 0.1360 m, 방향 오차 최대 9.12°, 실제 정지 3.60 s 이상. 중앙 장애물 우회와 접촉 없음 확인 |
| s04 | 주행 중 추가 조건 3회 성공 보고 | 사용자 확인: 접촉 없는 우회, 성공 결과, 정지. 회차별 주입 시각·수치 오차는 미기록 |
| s05 | 미관측 목표 거부·정지 통과 | 로그: `status=6`, `error_code=206`, `Vehicle stop confirmed`, 최종 `failed` |
| s06 | 취소·정지 통과 | 로그: `canceled`, `Vehicle stop confirmed`, 최종 `canceled` |
| s07 | 명령 중단·정지 통과 | 타임아웃 로그와 사용자 확인: 0 명령, 실제 속도 기준 1초 이상 유지 |
| s08 | 스캔 단독 중단·복구 후 정지 유지 통과 | 아래 bag 분석과 안전 취소·정지 로그 |

### s01 기록 분석

`/tmp/t9_2_s01_r1/t9_2_s01_r1_0.mcap` 한 파일에 3회 기록이 들어 있다.
시뮬레이션 시계의 초기화와 `/navigation_allowed`의 주행 허용 구간을 기준으로
회차를 분리했다. 각 시작 TF는 `(0, 0, 0)`으로 확인됐다.
각각의 ROS 로그에서 목표 수락, 성공 결과, 정지 확인, 최종 `succeeded`를 확인했다.
세 번째 실행의 준비 중 거부된 요청은 성공 회차에 포함하지 않았다.

위 기록란의 위치·방향 오차는 TF 기반 추정값이다.
`/odom`과 `/ground_truth/odom` 모두 결과 이후 속도가 공통 정지 기준 안에
연속으로 유지되는 구간을 확인했다. 접촉 여부는 사용자 육안 확인에 따른다.

### s02 기록 분석

`/tmp/t9_2_s02_r1/t9_2_s02_r1_0.mcap` 한 파일에 담긴 3회 주행을
시뮬레이션 시계 초기화와 `/navigation_allowed` 주행 허용 구간으로 분리했다.
각 시작 TF는 `(0, 0, 0)`이며 목표는 `(0, 1, π/2)`다.
위치 오차는 `hypot(x, y - 1)`, 방향 오차는
`abs(atan2(sin(yaw - π/2), cos(yaw - π/2)))`로 계산했다.

3회 모두 로그의 성공 결과·정지 확인·최종 `succeeded`와
`/odom`, `/ground_truth/odom`의 연속 정지 구간을 확인했다.
위치·방향·정지 수치는 모두 당시 기준을 만족한다.
bag에는 접촉 검출 토픽이 없으며, 접촉 없음은 사용자 육안 확인으로 기록했다.

### s03 기록 분석

`/tmp/t9_2_s03_r1/t9_2_s03_r1_0.mcap` 한 파일에 담긴 3회 주행을
시뮬레이션 시계 초기화와 `/navigation_allowed` 주행 허용 구간으로 분리했다.
각 시작 TF는 `(0, 0, 0)`이며 목표는 `(4, 0, 0)`이다.
위치 오차는 `hypot(x - 4, y)`, 방향 오차는
`abs(atan2(sin(yaw), cos(yaw)))`로 계산했다.

3회 모두 로그의 성공 결과·정지 확인·최종 `succeeded`와
`/odom`, `/ground_truth/odom`의 연속 정지 구간을 확인했다.
기록된 실제 모델 중심 궤적의 최소 y는 회차별 약 -1.275, -1.266, -1.268 m로,
중앙 장애물의 남쪽을 우회했다. 이 궤적 확인을 자동 접촉 검출로 취급하지 않는다.
장애물·외벽과 접촉하지 않았다는 사용자 육안 확인을 세 회차에 반영했다.

### s08 기록 분석

`/tmp/s08_scan_loss/s08_scan_loss_0.mcap`에서 읽은 구간 기준이다.
시간 간격은 bag 수신 시각으로 계산했으며 프로세스 신호 전달 시각의 실측값은 아니다.

- 연속 스캔 사이 중단 구간: 약 3.62초.
- 중단 구간에도 `/odom`, `/ground_truth/odom`, `/clock` 수신이 지속됐다.
  오도메트리와 시계 timestamp가 계속 증가했다.
- 마지막 스캔 수신 약 0.54초 뒤 `/navigation_allowed`가 false가 됐다.
- 로그에서 안전 취소, `Vehicle stop confirmed`, 최종 `State: failed`를 확인했다.
- 스캔 복구 후 읽은 약 83초 동안 `/cmd_vel`의 모든 성분과
  `/odom`, `/ground_truth/odom`의 선속도 크기·각속도가 0으로 유지됐다.
- 같은 복구 구간에서 `/navigation_allowed`는 계속 false였다.

`/tmp`의 bag과 사용자 홈의 ROS 로그는 로컬 진단 자료이며 저장소에 포함되지 않는다.
장기 보관과 실제 위치 오차 산출은 T9.3의 기록 체계에서 정비한다.

### 당시 평가 완료 범위

s01~s08의 시나리오 절차와 이번 수동 평가 결과를 정리했다.
당시 controller·costmap 설정으로 s01~s03을 각각 독립적으로 3회 실행했고,
9회 모두 TF 위치 오차 0.15 m 이하, 방향 오차 10도 이하,
1초 이상 실제 정지와 성공 결과를 확인했다. 접촉 없음은 사용자 육안 확인이다.
수치 검증은 TF 기반 추정값이며, 실제 모델 자세를 사용한 정지 확인과 구분한다.

s04는 3회 성공 보고, s05~s08은 위에 명시한 실패 반환·취소·정지·복구 동작을
확인했다. s04 회차별 주입 시각과 도착 오차는 미기록으로 남긴다.
s05는 미관측 목표 거부를 검증한 범위로 기록한다.
실제 도착 위치 오차의 산출과 기록 장기 보관은 T9.3에서 정비한다.

이후 실제 자세 기반 도착 오차 산출과 실행별 기록 보관을 수행했다.
0.10 m 기준의 평가 및 마무리 지연 개선 후 결과는
[1차 시뮬레이션 검증 결과](simulation_validation.md)에 정리했다.
