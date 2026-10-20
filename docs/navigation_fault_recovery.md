# 내비게이션 오류 후 명시적 복구

이슈 [004](../issues/004_navigation_fault_recovery.md)의 복구 서비스는 동일 manager가
이전 액션 종료를 확인할 수 있는 경우에 한해 노드 재시작 없이 준비 상태로 복귀한다.
복구 완료만으로 움직이지 않으며, 별도의 새 목표가 필요하다.

## 적용 범위

정지 확인·목표 응답·취소 타임아웃, 결과 조회·수신 실패, 종료 여부가 불명확한
액션 결과에서 복구 상태를 `required`로 고정한다. 늦은 응답이나 정상 센서 복귀만으로
해제하지 않는다. 기존 정상 성공·실패·취소의 정지 확인 흐름은 유지한다.
모드 전환·비상정지·센서 이상도 취소와 정지 확인을 정상 완료하면 기존처럼 새 목표를
받을 수 있다. 그 과정에서 위 타임아웃·통신 실패가 발생하면 명시적 복구가 필요하다.

복구 상태는 기존 주행 상태와 별도로 `none`, `required`, `recovering`을 사용한다.
`required`와 `recovering`에서는 주행 상태가 무엇이든 자동 명령과 새 목표를 차단한다.

## 사용 방법

ROS 환경을 로드하고 현재 상태부터 확인한다.

```bash
source env.sh
ros2 topic echo /navigation_manager/recovery_status vehicle_interfaces/msg/NavigationRecoveryStatus --qos-durability transient_local
```

상태에는 manager 세션, 주행·복구 상태, 최초 오류 원인·Unix 발생 시각(ns),
시도 ID·액션 goal UUID, 액션 확인 상태, 정지·데이터·제어 준비 여부,
현재 차단 사유와 마지막 복구 결과가 포함된다. reliable·transient local·depth 1로
시작 및 0.1초마다 발행하여 새 관찰자도 현재 상태를 확인할 수 있다.
`stop_confirmed`는 현재 관찰 구간의 정지 여부이며, 복구 요청과 액션 종료 확인 후
관찰 구간이 새로 시작된다. `data_ready`는 지도·scan·odom·TF 조건을 뜻한다.
`control_ready`는 최신 제어 상태·auto·비상정지 해제·manager 등록·재시작 차단 조건이다.

안전 조건을 복원한 뒤 명시적으로 요청한다.

```bash
ros2 service call /navigation_manager/recover std_srvs/srv/Trigger '{}'
```

- 서비스 `success: true`는 요청 접수 또는 이미 처리 중임을 뜻한다. 완료 응답이 아니다.
- 상태 토픽의 `recovery_state: none`, `last_result: succeeded`,
  `navigation_state: waiting`으로 완료를 확인한다.
- 이후 새 목표를 제출하면 기존 구동기 준비·명시적 활성화 절차가 수행된다.
- 복구 중 반복 요청은 취소·결과 조회·정지 관찰·5초 제한 시간을 중복 시작하지 않는다.
  완료 후 새 목표가 없는 `WAITING`에서 반복 요청해도 아무 동작 없이 성공한다.
- 정상 목표 실행 중에는 복구 요청을 거부한다.

## 복구 조건과 실패 처리

1. auto 모드, 비상정지 해제, 최신 제어 상태와 센서·TF 준비를 확인한다.
2. 알려진 이전 목표에만 취소를 요청하고 최종 액션 결과를 기다린다.
   취소 수락 자체는 종료 증거가 아니다. 실패했던 결과 조회는 해당 handle로 재시도한다.
3. 요청 시점과 액션 종료 확인 시점 중 늦은 시점 이후에 생성·수신된 odom으로
   연속 1초 정지를 확인한다. 이동·시각 역행·수신 단절은 정지 관찰을 초기화한다.
4. 센서·TF와 동일 guard 세션·epoch를 재확인하고 이전 callback을 무효화한 뒤 준비 상태로 복귀한다.

복구 시도는 steady clock 5초 제한이다. 제한 초과, 센서·TF·제어 조건 상실,
복구 중 guard 세션·epoch 변경이나 ROS 시각 역행 시 `required`와 최초 오류를 유지하고
`last_result: failed` 및 추가 실패 이유를 제공한다. 조건을 복원한 뒤 다시 요청할 수 있다.
비상정지를 해제하거나 제어 모드를 바꾸거나 구동기를 활성화하는 기능은 포함하지 않는다.

## 노드 재시작이 필요한 경우

`acceptance_unknown`은 전송 UUID를 알아도 목표 수락·종료를 증명할 수 없는 상태이다.
서버가 발견되거나 `STATUS_UNKNOWN` 결과가 왔다는 이유로 복구하지 않는다.
늦은 수락·거절이 도착하면 해당 목표의 정리 절차로 진행할 수 있다.
응답이 끝내 오지 않으면 Nav2·guard·manager를 함께 종료·재시작하는 기존 절차가 필요하다.
manager 재시작의 `navigation_reset_required`도 서비스로 지우지 않는다.
기존 Nav2를 먼저 종료하여 목표 실행을 끝내야 하며, guard만 재시작해 차단을 우회하면 안 된다.
전체 목표 일괄 취소와 자동 Nav2 재시작은 구현하지 않는다.

## 기록과 검증

기록기에 `/navigation_manager/recovery_status`를 포함한다.
이벤트 스키마 버전 1은 유지하고 `recovery_required`, `recovery_started`,
`recovery_failed`, `recovery_completed`를 추가한다. 평가기는 복구 완료를
주행 성공으로 분류하지 않는다. 단일 시도 평가에 주행 최종 상태가 없으면 기존처럼 거부한다.

단위 시험은 `vehicle_planning/test/test_navigation_recovery.py`, 실제 ROS 서비스·액션 시험은
`test_navigation_recovery_integration.py`에 있다. Gazebo 시험은 별도 ROS domain에서
새 프로세스로 다음과 같이 실행한다. 두 터미널에서 같은 환경과 domain을 사용한다.

```bash
source env.sh
export ROS_DOMAIN_ID=94
export GZ_PARTITION=navigation_recovery_004
export ROS_LOG_DIR=/tmp/navigation-recovery-gazebo-log
ros2 launch src/vehicle_simulation/test/recovery_system.launch.py
```

```bash
source env.sh
export ROS_DOMAIN_ID=94
export GZ_PARTITION=navigation_recovery_004
export ROS_LOG_DIR=/tmp/navigation-recovery-gazebo-log
python3 -c "import os, pytest; os.environ['RUN_RECOVERY_GAZEBO']='1'; raise SystemExit(pytest.main(['-q', 'src/vehicle_simulation/test/test_navigation_recovery_gazebo.py']))"
```

테스트 launch는 headless Gazebo·Nav2·guard·manager를 실행하고 manager의 odom만
테스트 중계 토픽으로 연결한다. 시험이 액션 종료 후 중계를 차단하여 실제 5초 정지 타임아웃을
만들고, 데이터 복귀·명시적 복구·정지 유지·새 목표 성공을 검증한다.
시험 종료 후 launch도 종료하여 다음 시험에 목표·비상정지 상태를 남기지 않는다.

### 2026-10-20 검증 결과

| 검증 | 결과 |
|---|---|
| 변경 관련 패키지 시험 | 5개 패키지 빌드·테스트 통과, 옵트인 Gazebo 시험은 별도 실행 |
| 복구 단위 시험 | 26개 통과: 시간 경계·신선도·중복 요청·대상 취소·결과 재조회·이전 콜백 격리 |
| 실제 ROS 서비스·액션 시험 | 2개 통과: 정지 타임아웃 후 새 목표, 늦은 수락 후 명시적 복구·durable 상태 조회 |
| Gazebo 복구 시나리오 | 통과: 실제 5초 정지 타임아웃 → 데이터 복귀 후에도 차단 → recover → 정지 유지 → 새 목표 성공 |
| 기존 Gazebo 안전 전환 | 통과: 수동 전환·비상정지 후 실패·3초 정지 유지, 새 목표 취소·정상 성공 |
| 기존 Gazebo 독립 watchdog | 단독 재실행 통과 |

Gazebo 복구 시험의 보고 값은 `fault: Vehicle stop was not confirmed within 5.0 s`,
`recovered_stationary_without_goal: true`, `new_goal: succeeded`였다.
시험 코드는 [복구 단위 시험](../src/vehicle_planning/test/test_navigation_recovery.py),
[ROS 통합 시험](../src/vehicle_planning/test/test_navigation_recovery_integration.py),
[Gazebo 복구 시험](../src/vehicle_simulation/test/test_navigation_recovery_gazebo.py)에 있다.
기존 watchdog 시험의 첫 병행 실행은 구동기 재활성화 단계에서
`Fresh actuator state was not received`로 실패했고, 다른 Gazebo 시험 종료 후 단독 재실행은
37.91초에 통과했다. 이번 변경은 구동기·guard 구현을 수정하지 않는다.
