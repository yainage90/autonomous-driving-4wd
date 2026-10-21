# T9.4 핵심 자동 검증 및 회귀 검증

## 목적과 검증 범위

명령 보호·목표 관리 테스트를 사용하고, 액션 결과 수신부터
실제 정지까지의 상태 전이와 고정 월드의 대표 주행 검증을 보강했다.

- 명령 제한, 타임아웃, 비상정지 우선순위.
- 성공·실패·취소 결과와 정지 확인 이후의 최종 상태 전환.
- evaluation 월드에서 직선·회전·우회 목표의 반복 실행.
- 센서 복구 후 자동 재출발 방지 등 기존 방어 동작 유지.

시나리오는 [T9.2](t9_2_evaluation_scenarios.md)를 따른다.
자동 판정은 액션 결과·상태 전이·Gazebo 속도 기반 정지이며,
도착 위치·방향 오차와 충돌 여부는 별도 확인이 필요하다.

## 실행 조건

- ROS 2 Jazzy, Gazebo Harmonic, Python 3.12.
- 가상환경 pytest는 `pyproject.toml`과 `uv.lock`에서 `7.4.4`로 고정한다.
- 기본 월드: `vehicle_simulation/worlds/evaluation.sdf`.
- 기본 지도: `vehicle_localization/maps/evaluation.yaml`.
- 기본 파라미터: `vehicle_bringup/config/safety.yaml`,
  `vehicle_localization/config/amcl.yaml`, `vehicle_planning/config/nav2.yaml`,
  `vehicle_control/config/controller.yaml`.
- 매 주행 시험 전 시스템을 종료하고 새로 시작한다.
- 지도·스캔 정합, Nav2 활성화, `navigation_manager`의 `waiting` 상태를 확인한다.
- 다른 조종 도구에서 목표나 속도 명령을 보내지 않는다.

## 패키지 테스트

이미 빌드된 워크스페이스 루트에서 실행한다.
주행 활성화 환경변수 `RUN_NAVIGATION_INTEGRATION`은 설정하지 않는다.

```bash
source env.sh
colcon build --packages-select vehicle_control vehicle_planning vehicle_simulation --symlink-install
source env.sh
colcon test --packages-select vehicle_control vehicle_planning vehicle_simulation
```

패키지별 결과는 다음 명령으로 확인한다.

```bash
colcon test-result --test-result-base build/vehicle_control --verbose
colcon test-result --test-result-base build/vehicle_planning --verbose
colcon test-result --test-result-base build/vehicle_simulation --verbose
```

## 주행 통합 테스트

### 실행 절차

터미널 A에서 매 회차 시스템을 새로 시작한다.
기본 evaluation 월드·지도·파라미터를 사용한다.

```bash
source env.sh
ros2 launch vehicle_bringup system.launch.py mode:=navigation
```

준비 상태를 확인한 뒤 터미널 B에서 시나리오 하나를 실행한다.

```bash
source env.sh
python3 -c "import os, pytest; os.environ.update(RUN_NAVIGATION_INTEGRATION='1', NAVIGATION_SCENARIO='s01'); raise SystemExit(pytest.main(['src/vehicle_simulation/test/test_navigation_integration.py', '-v', '-s']))"
```

`NAVIGATION_SCENARIO`는 `s01`, `s02`, `s03`, `s05`, `s06`을 지원한다.
한 번에 하나만 실행한다.

테스트 종료 시 fixture가 비상정지를 요청한다.
다음 시험은 터미널 A의 시스템 종료·재시작부터 수행하여
이전 목표·차량 위치·비상정지 상태를 남기지 않는다.
종료 정리의 비상정지 요청은 테스트 본문의 실제 정지 판정을 대신하지 않는다.

### 자동 판정

1. Gazebo `world` 기준 시작 자세 `(-2, 0, 0)`에서 위치 오차 0.15 m 이하,
   방향 오차 10도 이하와 1초 연속 정지를 확인한다.
2. 목표 제출 이벤트와 `attempt_id`를 확인하고 단일 목표·액션 결과를 검증한다.
3. `sending → running → stopping → 최종 상태` 순서를 확인한다.
4. s06은 `running`과 실제 움직임 확인 후 취소를 요청하고,
   `running → canceling → stopping → canceled`를 확인한다.
5. `safety_failure`와 `stop_timeout`이 발생하면 실패로 처리한다.
6. 최종 상태 이후 Gazebo 오도메트리로 1초 연속 정지를 독립적으로 확인한다.

1번의 0.15 m는 테스트 시작 위치 허용 범위이며 현재 도착 위치 합격 기준인
0.10 m와 별개다. 이 테스트만으로 도착 오차 0.10 m를 자동 판정하지 않는다.

정지 기준은 평면 속도 `hypot(linear.x, linear.y) ≤ 0.02 m/s`,
각속도 `abs(angular.z) ≤ 0.05 rad/s`다.
수신 경과 시간과 ROS 측정 경과 시간은 각각 0.5초 이하여야 하며,
미래 측정 시각 허용 범위는 0.1초다.
정지 유지 시간은 오도메트리 헤더 시각으로 계산하고,
표본 시각 증가와 0.5초 이하의 표본 간격을 확인한다.
각 정지 확인 호출 이후의 새 표본을 사용한다.

대기 제한은 실시간 `monotonic` 기준으로 계산한다.
목표 제출 서비스 응답 이후 최종 상태와 실제 정지 확인까지 120초를 기준으로
판정하며, 최종 상태 이후의 독립 정지 확인은 남은 시간 안에서 최대 5초 기다린다.
서비스의 `success=true`만으로 주행 성공·취소 완료를 판정하지 않는다.

## 기존 검증 재사용과 실패 추적

| 검증 대상 | 대표 테스트 |
|---|---|
| 명령 속도 제한 | `test_command_speeds_are_limited` |
| 비상정지 우선순위 | `test_mode_request_preserves_emergency_stop` |
| 타임아웃 경계 | `test_command_timeout_boundary` |
| 데이터 복구 후 주행 차단 유지 | `test_safety_fault_keeps_motion_blocked_after_data_recovers` |
| 오류 후 늦은 목표 수락 처리 | `test_late_acceptance_after_fault_waits_for_result_and_stop` |
| 스캔 중단 시 자율 명령 차단 | `test_scan_loss_stops_auto_commands_and_stays_stopped` |

이번에 추가한 `test_result_waits_for_stop_before_terminal_state`는
성공·실패·취소 모두 정지 확인 전에는 최종 상태로 전환하지 않는지 확인한다.
기존 테스트의 목적을 반복하는 별도 테스트는 추가하지 않았다.

실패 시 pytest assertion과 `navigation_events.json`을 함께 확인한다.

- 시작 조건 실패: 데이터 최신성·발행자·서비스·시작 자세 확인.
- `safety_failure`: 이벤트의 `reason` 확인.
- `stop_timeout`: manager의 오도메트리 정지 확인 실패 원인 확인.
- Gazebo 정지 확인 실패: 실제 속도·데이터 최신성·표본 간격 확인.
- 결과·상태 전이 불일치: `attempt_id`별 액션 결과와 이벤트 순서 확인.

이벤트 JSON은 테스트 성공·실패 시 pytest 임시 디렉터리에 생성되고 경로가 출력된다.
임시 자료는 이후 pytest 실행에서 정리될 수 있으므로 장기 보관할 자료는
출력된 경로에서 별도 복사한다. JUnit XML만으로 센서·TF 자료가 보관되지는 않는다.

실제 오류가 재현되면 원인에 해당하는 기존 테스트를 보강하거나 최소 회귀 케이스를
추가하고, 재현 조건·수정 내용·검증 결과를 기록한다.
위 테스트 목록을 모두 이번에 발생한 오류의 이력으로 해석하지 않는다.
