# T1.4 명령 선택과 실차 watchdog 요구사항

## 명령 경로

- 수동 입력 `/cmd_vel_manual`과 관측 출력 `/cmd_vel`: `geometry_msgs/msg/Twist`.
- 실제 구동 출력은 `/actuator/command`: `vehicle_interfaces/msg/ActuatorCommand`.
  Gazebo watchdog 플러그인이 직접 수신하며 `/cmd_vel` 브리지는 사용하지 않는다.
- Nav2 입력 `/cmd_vel_auto`: `geometry_msgs/msg/TwistStamped`.
  controller와 behavior server 모두 `enable_stamped_cmd_vel=true`로 실행한다.
- manager → guard 입력 `/navigation_command`: `vehicle_interfaces/msg/NavigationCommand`.
  목표 ID, guard/manager 세션, 전환 번호, heartbeat를 속도 명령과 함께 전달한다.
- `/navigation_permission`: `vehicle_interfaces/msg/NavigationPermission`.
- `/navigation_allowed`의 `Bool`은 관찰용이다. 이 토픽만 발행해서 주행을 허가할 수 없다.
- 최종 출력 발행자는 `command_guard` 하나로 유지한다.
- `system.launch.py`의 navigation 모드는 auto, manual/mapping 모드는 manual로 시작한다.
- `/set_auto_mode`의 `true`는 auto, `false`는 manual을 선택한다.

```text
수동 조종 → /cmd_vel_manual ────────────────────────────────┐
                                                          ├→ command_guard → /actuator/command → 구동기
Nav2 → /cmd_vel_auto → navigation_manager → /navigation_command ┘
```

## 모드 전환과 새 목표

- 모든 모드 선택·비상정지 요청은 guard의 전환 번호를 증가시키고,
  기존 허가·명령 시각을 즉시 무효화하며 0 명령을 발행한다.
  같은 모드를 다시 선택하거나 비상정지를 해제하는 요청도 포함한다.
- manager는 `/control_state`의 모드·비상정지·전환 번호를 확인하고,
  실행/수락 대기 중인 목표가 있으면 허가를 차단하고 취소를 요청한다.
  짧은 수동→자동 왕복으로 중간 상태를 놓쳐도 전환 번호 변경을 감지한다.
- 자동 복귀나 비상정지 해제만으로 이전 목표를 재개하지 않는다.
  별도 재개 기능은 제공하지 않으며 새 목표를 제출해야 한다.
- 새 목표는 이전 액션 결과 수신과 `STOPPING` 이후 정지 확인이 끝나야 받는다.
  auto·비상정지 해제·센서/TF 준비·최신 제어 상태가 필요하다.
  최초 목표도 최신 연속 odom으로 1초 정지를 확인한 뒤 받는다.
- 수동 입력은 명시적 manual 선택과 구동기 활성화 성공 이후 새 입력으로 조작한다.
  초기 launch 모드 선택은 활성화가 아니다. 모드 선택은 비상정지를 해제하지 않는다.
- 비상정지 서비스 성공은 로컬 상태 변경과 0 명령 발행을 뜻한다.
  수동 모드 서비스는 구동기 정지 확인·활성화 응답까지 받아야 성공한다.
  auto 선택은 모드만 선택하며 새 목표의 준비 절차에서 구동기를 활성화한다.
  Nav2 취소 완료는 별도로 확인한다.
- 취소 거부·5초 타임아웃·결과 유실·정지 확인 실패에서는 차단을 유지한다.
  이벤트와 로그에 이유를 남긴다. 타임아웃·결과 통신 오류 이후에는
  [명시적 내비게이션 복구](navigation_fault_recovery.md)를 요청해야 한다.
  이전 목표 종료를 확인할 수 없는 경우에는 차단과 아래 재시작 절차를 유지한다.

## 세션과 지연 메시지

- guard가 UUID 세션과 전환 번호를 소유하며 0.1초마다 제어 상태 heartbeat를 발행한다.
  manager는 `/register_navigation`으로 자신의 새 프로세스 세션을 등록한다.
  초기 등록에는 주행 허가가 필요하지 않아 두 노드가 서로 기다리지 않는다.
- 허가와 속도 명령은 현재 두 세션·전환 번호·목표 ID와 일치해야 한다.
  허가는 증가하는 순서 번호로 역순 도착을 거부한다.
- guard는 자신이 보낸 heartbeat를 기준으로 허가의 왕복 나이를 검사한다.
  수신 시각만 갱신한 오래된 허가는 사용할 수 없다. 기본 유효 시간은 steady clock 0.5초이다.
- 허가 단절 시 guard는 출력을 차단하고 전환 번호를 갱신한다.
  manager도 제어 상태가 0.5초 끊기면 목표를 취소하고 차단을 유지한다.
- manager는 새 목표 수락 이전에 생성된 Nav2 명령을 거부한다.
  guard도 명령 생성 시각을 검사한다. 두 노드는 같은 ROS 시계를 사용해야 하며,
  시뮬레이션 launch는 양쪽 모두 `use_sim_time=true`로 실행한다.
  watchdog과 heartbeat 타이머는 시뮬레이션 일시정지와 무관한 steady clock을 쓴다.
- guard 재시작은 세션을 바꾸므로 실행 중 manager가 목표를 취소한다.
  manager만 재시작하면 새 등록으로 이전 허가를 폐기하고 `navigation_reset_required`를 설정한다.
  새 manager는 이전 목표 종료를 증명할 수 없어 새 목표도 거부한다.
  복구하려면 기존 Nav2를 종료하여 목표 실행을 끝내고 Nav2·guard·manager를 함께 재시작한다.
  guard만 재시작해서 이 차단을 우회하면 안 된다.
- Nav2 액션 서버는 이 manager가 목표를 관리한다. 외부에서 Nav2에 직접 목표를 보내거나
  서버를 공유하는 운영 방식은 지원하지 않는다.

## 현재 command_guard 설정과 동작

- 명령 타임아웃: 0.5초
- 타임아웃 검사 주기: 0.05초
- 최대 선속도: 0.2 m/s
- 최대 각속도: 0.6 rad/s
- 명령 타임아웃 측정과 검사 타이머는 steady clock을 사용한다.
- 주행 중에는 유효한 입력을 저장하고 steady clock 20 Hz로 구동 명령을 반복한다.
  매 출력에서 원래 입력 freshness·자동 허가·비상정지·구동기 상태를 다시 검사한다.
- 구동기 상태와 heartbeat가 0.2초 만료되거나 구동기가 차단되면 guard는
  입력과 자동 허가를 폐기하고 전환 번호를 갱신한다.
- guard가 멈추면 Gazebo 플러그인의 독립 watchdog이 마지막 유효 명령 후
  0.3초 만료를 판단하고 첫 물리 업데이트에서 바퀴 출력을 0으로 바꾼다.
- 시작·watchdog·pause·재시작 뒤에는 수동 재선택 또는 새 자동 목표가 필요하다.
  비상정지 해제나 통신 복구만으로 활성화하지 않는다.
- 전체 프로토콜·설정·검증 및 실차 후속 범위는
  [구동기 watchdog 문서](actuator_watchdog.md)를 따른다.

## 실차 모터 제어기 watchdog 요구사항

다음 수치는 초기 제안값이며 하드웨어와 통신 지연 측정 후 확정한다.
아래 실차 값은 아직 확정되지 않았다. 시뮬레이션에는 20 Hz·0.3초를 적용했다.

| 항목 | 초기 제안 |
|---|---|
| 정상 주행 명령 송신 주기 | 10 Hz 이상 |
| 모터 제어기 watchdog 타임아웃 | 0.3초 |

현재 guard는 20 Hz로 출력하지만 입력 freshness는 원래 수신 시각으로 유지한다. 정상 명령의 최대 수신 간격과 통신 지연을 측정해
watchdog 타임아웃을 확정한다.

- watchdog은 ROS 노드나 PC의 정상 동작에 의존하지 않고 모터 제어기에서
  직접 수행한다.
- 마지막 유효 명령 이후 타임아웃이 지나면 모터 정지 처리를 시작한다.
- 시간 측정은 제어기 내부의 단조 증가 시계를 사용한다.
- 잘못된 명령은 watchdog을 갱신하지 않고 정지 처리한다.
  유효성 검사는 메시지 형식, 유한한 속도 값 및 허용 범위를 포함한다.
- 제어기 시작 시 모터 출력은 정지 상태로 유지한다.
- watchdog 정지 후에는 통신 복구만으로 자동 재출발하지 않는다.
- 명시적 재활성화와 그 이후의 새 유효 명령이 있어야 주행을 허용한다.
  재활성화 상태 전이는 시뮬레이션 계약을 따르며 실제 통신 형식은 실차 단계에서 정의한다.
- 제어기 비상정지는 명령 수신이나 모드 선택으로 해제되지 않는다.
- 정지 시 브레이크 또는 출력 차단 방식은 모터 제어기 사양에 맞게
  확정한다.

## 실차 검증 항목

하드웨어 구현 단계(H2)에서 다음 시험을 수행한다.

- 정상 주행 중 `command_guard` 프로세스 종료
- 정상 주행 중 PC와 모터 제어기 사이 통신 단절
- 비정상 명령 수신
- 통신 복구 후 자동 재출발하지 않는지 확인
- 명시적 재활성화 이후 새 명령으로 주행하는지 확인
- 정상 송신 주기와 예상 통신 지연에서 불필요한 watchdog 정지가
  발생하지 않는지 확인

각 시험에서 마지막 유효 명령 수신부터 정지 처리 시작까지의 시간과
실제 차량 정지까지의 시간을 별도로 측정한다.

## 세션 연동 검증 결과 (2026-10-20)

- 관련 패키지 빌드 완료. `vehicle_control` 65개, `vehicle_planning` 78개,
  `vehicle_bringup` 3개, `vehicle_simulation` 일반 회귀 8개 통과: 총 154개.
  외부 시스템이 필요한 2개 시험은 기본 실행에서는 건너뛴다.
- 추가 Gazebo·Nav2 통합 시험 1개 통과. 수동 전환과 비상정지 활성화 후
  각각 0.2초 만에 자동 복귀·정지 해제를 수행했다.

| 전환 | 액션 종료와 정지 확인까지 | 이후 정지 관찰 | 0 출력 표본 | 실제 정지 표본 |
|---|---:|---:|---:|---:|
| auto → manual → auto | 1.096초 | 3초 | 90 | 150 |
| 비상정지 활성화 → 해제 | 1.072초 | 3초 | 90 | 150 |

두 경우 모두 `canceling → stopping → failed`로 종료했고 새 목표 전까지 정지를 유지했다.
실제 정지는 `/ground_truth/odom`의 선속도 0.02 m/s 이하·각속도 0.05 rad/s 이하로 별도 판정했다.
이후 새 목표에서 움직임, 정상 취소, 정상 목표 성공까지 확인했다.

재현은 새 evaluation navigation 시스템을 실행한 뒤 같은 ROS 도메인에서 수행한다.
테스트 종료 시 비상정지를 활성화한다.

```bash
source env.sh
RUN_SAFETY_SYNC_INTEGRATION=1 python -m pytest src/vehicle_simulation/test/test_safety_navigation_sync.py -q -s
```

노드 재시작·늦은 수락·취소 거부·heartbeat 단절·지연 메시지는
`test_navigation_control_sync.py`의 ROS 노드 통합 시험에서 검증했다.
이 경계 조건의 Gazebo 강제 장애 주입 및 실차 물리 시험은 별도이다.

## 이전 구현 검증 기록

다음은 세션 연동 이전 구현에 대해 사용자가 실행하고 확인한 기록이다.
현재 자동 입력의 메시지 타입과 재출발 정책은 위 계약을 따른다.

- `vehicle_control` 단위·통합 테스트: 48개 통과
- 수동·자율 명령 동시 발행 시 선택된 모드의 명령만 출력
- 양방향 모드 전환 후 이전 입력이 계속 들어와도 정지 출력 유지
- 선택된 모드의 새 입력을 받으면 해당 명령 전달
- 전체 실행에서 `/cmd_vel` 발행자 하나(`command_guard`) 확인
- Gazebo에서 수동 → 자율 전환 시 0 출력 유지 및 차량 정지 확인
- Gazebo에서 자율 → 수동 전환 시 0 출력 유지 및 차량 정지 확인

Gazebo 구동기 watchdog은 구현·장애 주입 검증을 마쳤다.
실차 watchdog은 요구사항 단계이며 드라이버·펌웨어 구현과 물리 검증은 후속 작업이다.
