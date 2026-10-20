# 구동기 watchdog 계약과 시뮬레이션 검증

구현 범위는 ROS 2 Jazzy / Gazebo Harmonic의 구동기 모사이다.
실차 제어기·통신 드라이버·펌웨어와 물리 제동 검증은 아직 없다.
시뮬레이션 시험 통과로 이슈 002의 실차 완료 조건을 충족했다고 판단하지 않는다.

## 명령 경로

```text
수동 입력 / navigation_manager → command_guard → /actuator/command
                                                ↓
                           vehicle::WatchdogDiffDrive → 네 바퀴 출력
```

`/cmd_vel`은 기존 `Twist` 관측·기록 인터페이스로 유지한다. 발행자는 guard 하나다.
구동 플러그인은 `/cmd_vel`이나 Gazebo의 기존 velocity/enable 토픽을 구독하지 않는다.
기존 `/cmd_vel` ROS→Gazebo 브리지를 제거했다. `/odom`, `/tf`, ground truth,
물리 치수와 계산용 윤거 보정은 유지한다.

`vehicle_actuator_sim`의 C++ System 플러그인이 ROS 명령을 직접 수신한다.
watchdog은 구동기 자신의 `std::chrono::steady_clock`으로 판단하며,
매 `PreUpdate`에서 바퀴 명령을 쓰기 전에 만료를 확인한다. 만료 시 목표와
속도 제한기 이력을 제거하고 모든 구동 바퀴에 0 속도를 쓴다.
ROS executor가 멈춰도 물리 업데이트의 검사가 수행된다.
ROS timer에서도 만료 상태를 갱신하지만 바퀴 출력 변경은 물리 업데이트가 담당한다.

Gazebo 전체가 정지하면 모사 물리와 watchdog 코드도 실행되지 않는다.
재개 첫 물리 업데이트에서 만료를 검사하고 이전 속도를 적용하지 않는다.
이때 정지 중 경과 시간은 0.3초의 실행 중 응답 지연과 구분한다.
실차에서는 이 검사와 정지 출력이 PC와 독립적인 제어기에서 실행되어야 한다.

## 프로토콜

명령과 상태는 reliable / volatile / keep-last 1이다. DDS 보안·발행자 인증은
이 계약의 범위가 아니며, 내부 서비스를 외부 운영 클라이언트가 직접 호출하지 않는다.

| 인터페이스 | 계약 |
|---|---|
| `/actuator/command` (`ActuatorCommand`) | 구동기 세션·활성화 세대·guard 세션·증가 순서·heartbeat·평면 속도 |
| `/actuator/status` (`ActuatorStatus`) | 세션·세대·heartbeat, 활성/비상정지/pause/정지 확인, 차단 사유, 마지막 수락 번호와 출력·시간 진단 |
| `/actuator/arm` (`ArmActuator`) | 내부 `DISARM`, `ARM`, `ESTOP`, `CLEAR_ESTOP` 연산. 현재 세션·세대·heartbeat 필요 |
| `/prepare_actuator` (`PrepareActuator`) | manager→guard 내부 요청. 현재 두 세션·전환 번호와 재사용하지 않은 목표 ID 필요 |

- 구동기 부팅 시 새 세션으로 차단 상태에서 시작한다. 활성화·차단 시 세대를
  바꾸므로 이전 명령과 지연된 활성화 요청은 새 허가를 얻지 못한다.
- 현재 활성 세대와 소유 guard가 일치하는 증가 순서 명령만 수락한다.
  heartbeat는 구동기가 발행한 시점부터 자신의 단조 시계로 나이를 계산한다.
  발신자와 수신자의 ROS 시계를 동기화할 필요가 없다.
- 중복·역순·이전 세션·만료 heartbeat는 watchdog을 갱신하지 않는다.
  서로 다른 순서의 오래된 명령도 heartbeat 유효기간 밖에서는 갱신하지 못한다.
- 현재 세대 명령의 NaN/Inf, 허용 속도 초과, 사용하지 않는 축의 0 이외 값은
  즉시 차단한다. guard의 입력 제한과 별도로 수신기에서 검사한다.
- `ARM`은 비상정지가 해제되고 pause가 아니며 바퀴 오도메트리의 정지가
  연속 1초 확인된 경우만 성공한다. 물리 업데이트가 0.2초 이상 끊기면 정지
  확인을 무효화한다. 활성화는 0 출력으로 시작하고 watchdog도 시작한다.
- 활성화 후 새 입력이 필요하다. guard는 이전 입력을 폐기하며 자동 명령은
  활성화 완료 이후 생성된 stamp도 요구한다.
- `ESTOP`은 별도로 래치한다. 모드 선택·명령·`ARM`은 해제할 수 없다.
  `CLEAR_ESTOP`도 차단 상태를 유지하며 별도 활성화를 요구한다.
- guard는 차단/비상정지 연산을 순서대로 전송한다. 빠른 활성화→해제가
  같은 세대를 사용해 한쪽 요청을 잃지 않도록 응답과 새 상태를 확인한다.

## 설정과 운영

통합 실행은 `vehicle_bringup/config/safety.yaml` 또는 `safety_params_file`로
지정한 **같은 파일**을 guard와 플러그인에 전달한다. 독립 Gazebo 실행의 기본은
`vehicle_actuator_sim/config/safety.yaml`이다. 두 기본 파일의 값은 동일하게 유지한다.

| 파라미터 | 시뮬레이션 기본값 |
|---|---:|
| `actuator_output_period_sec` | 0.05초 (명령·상태 20 Hz) |
| `actuator_watchdog_sec` | 0.3초 |
| `actuator_heartbeat_timeout_sec` | 0.2초 |
| `actuator_status_timeout_sec` | 0.2초 |
| `command_timeout_sec` | 0.5초 |
| `navigation_timeout_sec` | 0.5초 (guard 기본값) |
| 선속도 / 각속도 한계 | 0.2 m/s / 0.6 rad/s |

파라미터는 유한한 양수이며 `출력 주기 < 각 freshness 기간 < watchdog`이어야 한다.
반복 출력은 입력 수신 시각이나 자동 명령 생성 시각을 갱신하지 않는다.
매 출력에 입력 만료·자동 허가·비상정지·구동기 상태를 다시 확인한다.
따라서 입력 생산자 장애의 정지 지연은 모터 watchdog 0.3초와 같지 않다.
허용 지터·통신 지연과 실차 제동 거리는 별도로 측정해 실차 값을 확정해야 한다.

시작·watchdog·구동기 재시작·pause 후 재활성화 절차:

1. 비상정지가 활성화되어 있으면 `/set_emergency_stop(false)`로 명시적으로 해제한다.
   이 서비스의 성공은 guard의 로컬 상태 변경·정지 명령 발행을 뜻한다.
   구동기 전송은 비동기이며 통신 실패 시 구동기는 차단을 유지한다.
2. 수동: 정지 확인 후 `/set_auto_mode(false)`를 호출한다. 서비스는 구동기 활성화
   응답까지 받아야 성공하며, 실패 사유를 응답한다. 성공 이후 새 수동 입력으로 움직인다.
3. 자동: auto 모드를 선택한 뒤 **새 목표**를 제출한다. manager는 기존 준비 조건·
   이전 목표 종료·연속 1초 정지 확인 후 guard에 준비를 요청한다. 성공한 경우에만
   Nav2 목표를 제출한다. 준비 중 제어 상태가 바뀌면 제출하지 않는다.

launch의 `initial_mode`는 활성화 요청이 아니다. 통신 복구와 비상정지 해제도
활성화 요청이 아니다. 일시정지 진입 시 허가를 폐기하며 재개만으로 움직이지 않는다.
`navigation_reset_required`의 기존 복구 제한도 그대로 적용한다.

## 빌드와 시험

ROS 자체 의존성은 배포판의 apt/rosdep 패키지를 사용한다.
새 CMake 패키지에서 가상환경 Python에 `catkin_pkg`가 없으면 시스템 Python을 지정한다.

```bash
source env.sh
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
colcon test --packages-select vehicle_actuator_sim vehicle_control vehicle_planning vehicle_description vehicle_bringup vehicle_simulation
colcon test-result --verbose
```

구동기 상태 머신의 C++ 테스트, 실제 guard/link/manager를 모사 수신기와 연결하는
ROS 테스트, 기존 명령 선택·목표 동기화 회귀를 분리한다. 기존 선택 테스트의
구동기 대역은 수신기 독립 정지 검증으로 사용하지 않는다.

아래 시험은 자체 headless Gazebo와 guard를 시작하고 소유한 프로세스만 신호로 제어한다.
다른 ROS 세션과 겹치지 않는 domain을 선택한다. 별도 Gazebo transport partition도 생성한다.

```bash
source env.sh
ROS_LOG_DIR=/tmp/watchdog-ros-log ROS_DOMAIN_ID=75 \
RUN_ACTUATOR_WATCHDOG_INTEGRATION=1 \
ACTUATOR_WATCHDOG_REPORT=evaluation_runs/watchdog/report.json \
python -m pytest src/vehicle_simulation/test/test_actuator_watchdog.py -q -s
```

검증 항목은 입력 생산자 중단, guard `SIGINT`/`SIGKILL`/`SIGSTOP`, 짧고 긴 pause,
시뮬레이터 `SIGSTOP`/`SIGCONT`, 중복 패킷을 반복하는 명령 채널 중단,
수신기 재시작·이전 세션 재전송, NaN 명령, 비상정지 별도 해제와 명시적 재활성화이다. 잘못된 수치·이전 세션·역순·오래된
서로 다른 명령은 수신기 C++ 시험으로 별도 확인한다.

시간 진단은 수신기 단조 시계의 마지막 유효 수신·만료·0 출력 적용을 기록한다.
실제 정지는 ground truth의 평면 속도 ≤0.02 m/s·각속도 절댓값 ≤0.05 rad/s를
연속 1초 확인한다. `observed_stop_sec`와 이동 거리는 ground truth 수신 표본에
따른 관측값이며 전송 지연과 표본 간격을 포함한다. 실차 제동 거리로 해석하지 않는다.

2026-10-20 검증에서 guard 장애·명령 채널 중단의 만료 및 0 출력 적용은
마지막 유효 수신 후 0.300~0.301초, 실제 정지 관측은 0.318~0.335초였다.
0.1 m/s 입력에서 관측 이동 거리는 약 0.029~0.030 m였다.
입력 생산자 중단은 guard가 약 0.54~0.55초 후 0 출력을 적용했다.
시뮬레이터 0.8초 정지는 재개 첫 업데이트에 0 출력을 적용했고,
마지막 유효 수신부터 약 0.819초로 별도 기록했다.
구동기 재시작 중 이전 세션 재전송과 현재 세대 NaN 명령의 실제 정지 출력도 확인했다.
원본 요약은 `evaluation_runs/watchdog/report.json`, 시험 로그는 pytest 임시 디렉터리에 있다.

전체 워크스페이스 빌드와 일반 Python/ROS 시험 171개·수신기 C++ 시험 9개가 통과했다.
외부 시스템이 필요한 기본 skip은 11개이며, 별도 Gazebo watchdog·Gazebo/Nav2
통합 시험 각 1개도 통과했다. 전체 `colcon test`의 `vehicle_drivers`,
`vehicle_perception`은 기존 빈 패키지로 시험이 없어 exit 5를 반환한다.
테스트가 있는 패키지는 `--packages-skip vehicle_drivers vehicle_perception`으로 확인한다.

Gazebo/Nav2 회귀에서는 수동 전환·비상정지 후 각각 1.307초·1.082초에 이전 목표 종료와
정지를 확인하고 3초 정지를 유지했다. 새 목표의 재출발·정상 취소·정상 성공도 확인했다.
요약은 `evaluation_runs/watchdog/navigation_sync_result.json`에 보관한다.
기존 `test_safety_navigation_sync.py`는 새 evaluation 시스템의 노드·서비스가 모두 준비된
후 `RUN_SAFETY_SYNC_INTEGRATION=1`로 실행한다. 구동기 watchdog 시험과 달리 이 시험은
시스템을 직접 시작하지 않는다.

## 실차 후속 완료 조건

`vehicle_drivers`와 펌웨어는 아직 구현하지 않았다. 제어기와 통신 방식을 정한 뒤
동일한 상태 전이·재전송 거부·수신기 단조 시계 계약을 이식하고 CRC/프레이밍 등
실제 통신 형식을 확정한다. PC 통신이 끊긴 시험 자료는 제어기 자체 기록이나
독립 계측으로 남긴다. 브레이크/출력 차단, 허용 지터, 정지 거리 기준과 펌웨어 버전을
확정하고 바퀴를 띄운 시험 및 저속 지상 시험을 각각 통과해야 이슈 002를 닫는다.
