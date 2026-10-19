# T9.3 관측 자료 기록

## 목적과 지원 범위

실행별 입력 조건과 관측 자료를 보관하고, Gazebo 실제 자세로 최종 위치·방향
오차를 계산한다. 반복 평가 절차와 판정 기준은
[T9.2 반복 평가 시나리오](t9_2_evaluation_scenarios.md)를 따른다.

- `vehicle_bringup`의 `record_run`: 입력 사본 보관, 실행 명령 생성, bag 기록.
- `vehicle_planning`의 `navigation_manager`: 목표·수락 여부·액션 결과·상태 이벤트 발행.
- `vehicle_simulation`의 `evaluate_run`: bag에서 실제 자세를 읽어 오차 계산.

기록 명령은 `navigation`, `manual` 모드를 지원한다. 현재 월드·지도·파라미터는
아래에 명시한 기본 evaluation 파일로 고정되어 있으며 별도 선택 인자는 없다.
분석 명령은 목표 제출과 최종 상태가 각각 하나인 navigation 기록을 지원한다.
manual 기록, 목표가 여러 개인 기록, 최종 상태가 없는 기록은 분석하지 않는다.
분석이 거부된 경우에도 원본 bag은 남아 있다.

## 보관 자료와 실행 조건

```text
evaluation_runs/<실행 이름>/
├── COLCON_IGNORE
├── inputs/src/
├── source_manifest.json
├── run.json
├── start_system.sh
├── bag/
└── analysis.json
```

| 자료 | 내용 |
|---|---|
| `inputs/src/` | 기록 준비 시점의 소스·지도·설정 사본 |
| `source_manifest.json` | Git 커밋, 작업 트리 상태, 사본 파일별 SHA-256 |
| `run.json` | 시나리오, 모드, 예정 목표·기대 시작 자세, launch·bag 명령 |
| `start_system.sh` | 사본의 월드·지도·파라미터로 시스템을 시작하는 명령 |
| `bag/` | MCAP 관측 자료와 rosbag 메타데이터 |
| `analysis.json` | 실제 시작·최종 자세, 액션 결과·최종 상태, 변환 조건과 오차 |

`--run-dir`을 생략하면 `<workspace>/evaluation_runs/run_<UTC 시각>/`을 사용한다.
자동 이름은 마이크로초까지 포함한다. `--workspace`의 기본값은 현재 작업 디렉터리다.
시나리오·회차로 이름을 정하려면 `--run-dir evaluation_runs/s01_r1`처럼 지정한다.
명시한 상대 경로는 현재 작업 디렉터리를 기준으로 해석한다.

같은 실행 디렉터리는 재사용할 수 없다. 준비 도중 실패해 일부 파일만 남아도
다음 시도에는 새 실행 이름을 사용한다. `COLCON_IGNORE`는 복사한 패키지가
colcon의 중복 패키지로 검색되는 것을 막는다. 이 파일이 없는 과거 기록에는
해당 실행 폴더에 `COLCON_IGNORE`를 추가한다. `evaluation_runs/` 자체에도
`COLCON_IGNORE`를 두어 보관 자료를 패키지 검색에서 제외한다.

생성한 launch 명령은 `inputs/src/` 아래의 다음 파일을 사용한다.

- `vehicle_simulation/worlds/evaluation.sdf`
- `vehicle_localization/maps/evaluation.yaml`과 YAML에서 참조하는 지도 이미지
- `vehicle_bringup/config/safety.yaml`
- `vehicle_localization/config/amcl.yaml`
- `vehicle_planning/config/nav2.yaml`
- `vehicle_control/config/controller.yaml`

노드·launch·차량 모델은 빌드된 패키지를 사용한다. 소스 사본만으로 실행 당시
설치된 모든 코드·의존성 버전이 보장되지는 않는다. 관련 패키지를 빌드한 뒤
기록을 준비하고, 복사 중 소스를 편집하거나 실행 중 설정을 변경하지 않는다.

## 빌드와 기록 절차

모든 명령은 워크스페이스 루트에서 실행한다. 각 터미널에서 `source env.sh`를
실행한다. 기존 시스템을 종료하고, 매 회차 시스템을 새로 시작한다.

```bash
source env.sh
colcon build --packages-select vehicle_planning vehicle_bringup vehicle_simulation --symlink-install
source env.sh
```

터미널 A에서 기록을 시작한다. 아래 예시는 s01의 첫 실행이다.
이미 디렉터리가 있으면 `s01_r2` 등 새 이름을 사용하고 이후 경로도 맞춘다.

```bash
ros2 run vehicle_bringup record_run \
  --run-dir evaluation_runs/s01_r1 \
  --scenario s01 \
  --goal 1.0 0.0 0.0 \
  --start-map 0.0 0.0 0.0
```

`--goal`과 `--start-map`은 map 기준의 예정 조건이며 단위는 m, rad이다.
목표 전송이나 초기 자세 설정을 수행하는 인자가 아니다. 실제 전송 목표는
이벤트에서, 실제 시작 자세는 목표 제출 직전의 Gazebo 오도메트리에서 읽는다.
시나리오 이름도 메타데이터이며 시나리오 동작을 자동 실행하지 않는다.
`--run-dir`을 생략했다면 출력된 실행 경로를 이후 시스템 실행·분석 명령에 사용한다.

터미널 B에서 생성된 명령으로 시스템을 시작한다.

```bash
bash evaluation_runs/s01_r1/start_system.sh
```

지도·스캔 정합, 시작 자세, 실제 정지와 navigation 준비 상태를 확인한다.
터미널 A에서 `/navigation_manager/events` 구독이 시작된 것을 확인한 뒤
터미널 C에서 목표를 전송한다.

```bash
ros2 service call /navigation_manager/set_goal \
  vehicle_interfaces/srv/SetNavigationGoal \
  '{x: 1.0, y: 0.0, yaw: 0.0}'
```

결과와 실제 정지를 확인하고 A를 Ctrl+C로 종료한 뒤 B의 시스템을 종료한다.
기록 대상은 `/clock`, `/scan`, `/odom`, `/ground_truth/odom`, `/tf`, `/tf_static`,
`/cmd_vel_manual`, `/cmd_vel_auto`, `/cmd_vel`, `/navigation_allowed`,
`/navigation_manager/events`, `/rosout`이다.

```bash
ros2 bag info evaluation_runs/s01_r1/bag
```

필수 관측 토픽에 메시지가 있는지 확인한다. 발행하지 않은 입력 토픽은
기록되지 않을 수 있다. 예를 들어 자율주행에서 수동 명령을 발행하지 않았다면
`/cmd_vel_manual`이 없는 것은 정상이다. `recording_qos.yaml`은 스캔을
best effort로 구독하고, 고정 TF를 transient local로 구독하도록 설정한다.

## 이벤트와 분석 기준

이벤트는 `/navigation_manager/events`의 `std_msgs/msg/String`에 JSON으로 담긴다.
공통 필드는 `schema_version`, `attempt_id`, `event`, `ros_time_ns`,
`wall_time_ns`, `monotonic_ns`, `data`이다. 목표 시도별 ID로 자료를 연결한다.

이벤트 종류는 `goal_submitted`, `goal_response`, `state_changed`,
`action_result`, `safety_failure`, `stop_timeout`이다. `action_result`에는
Nav2의 status, error_code, error_msg와 분류한 outcome을 기록한다.
액션 결과와 정지 확인 후 manager 최종 상태는 구분한다.
센서 이상으로 안전 취소한 경우 액션은 canceled, 최종 상태는 failed일 수 있다.

- bag 수신 시각은 시스템 시각이다.
- 자세 선택은 이벤트 ROS 시각과 오도메트리 헤더 시각을 비교한다.
- 소요 시간은 같은 프로세스의 monotonic 시각 차이로 계산한다.
- 시작·최종 자세는 해당 이벤트 시각 이하의 가장 최근 표본이다.
- 0.1초보다 오래된 표본과 증가하지 않는 오도메트리 timestamp는 오류 처리한다.
- 최종 오차는 성공·실패·취소 여부와 별개로 최종 상태 시점의 자세에서 계산한다.

`/ground_truth/odom`의 `world → base_footprint` 자세를 map으로 변환한 뒤
실제 제출 목표와 비교한다. 지도 YAML의 origin은 world→map 변환이 아니다.
`--map-from-world TX TY YAW`는 world 좌표를 YAW만큼 회전한 뒤
map 기준 `(TX, TY)`를 더하는 변환이다.

아래는 이번 지도에 대한 수동 측정 변환을 적용한 참고 분석 명령이다.
다른 지도나 월드의 일반적인 기본값으로 사용하지 않는다.

```bash
ros2 run vehicle_simulation evaluate_run \
  --run-dir evaluation_runs/s01_r1 \
  --map-from-world 2.04446024 0.02889593 0.00875412 \
  --alignment-note "RViz A/B/C 3점 근사 정합. 잔차 RMS 0.0492m, A-B 거리 차이 0.0994m. 정밀 검증 전."
```

독립적으로 좌표 정합을 검증한 경우에만 `--alignment-verified`를 지정한다.
이 옵션은 검증 수행 여부를 사용자가 표시하는 것으로, 자동 검증 기능이 아니다.
재분석하면 `analysis.json`이 갱신된다. 변환별 분석을 별도로 유지하려면
갱신 전에 기존 결과를 다른 이름으로 보관한다.

## 2026-10-20 s01 관측 결과

기존 관측의 원본 경로는 `scratch/runs/s01_r1`이다. 이는 저장 경로 변경 전에
생성한 기록이며 그대로 보존했다. 위의 `evaluation_runs/s01_r1` 명령은 신규
기록 예시다. 기존 원본을 재분석하려면 분석 명령의 `--run-dir`에
`scratch/runs/s01_r1`을 지정한다. 목표는 map `(1, 0, 0)`이고
기대 시작 자세는 map `(0, 0, 0)`이다. 실제 제출 목표도 예정 목표와 일치했다.

| 항목 | 관측값 |
|---|---|
| 액션 결과 | succeeded, status 4, error_code 0 |
| manager 최종 상태 | succeeded |
| 목표 제출 → 최종 상태 | 7.6091초, monotonic 시각 차이 |
| 시작 자세 표본 시각 | ROS 25.820초 |
| 최종 자세 표본 시각 | ROS 33.420초 |
| 실제 시작 world 자세 | 약 `(-2, 0, 0)` |
| 실제 최종 world 자세 | 약 `(-1.190767, -0.011219, 0.154250)` |
| 3점 변환 적용 후 시작 map 자세 | 약 `(0.044537, 0.011388, 0.008754)` |
| 3점 변환 적용 후 최종 map 자세 | 약 `(0.853837, 0.007253, 0.163004)` |

자세 단위는 m, rad이다. bag에는 메시지 44,464개가 기록되었으며,
`/scan` 359개, `/odom`과 `/ground_truth/odom` 각각 1,794개,
`/tf_static` 1개, `/navigation_manager/events` 7개를 확인했다.
이벤트에서 목표 제출·수락·running·액션 성공·stopping·최종 성공 흐름을 확인했다.

### 좌표 정합의 근거와 불확실성

외벽은 밝은 자유 공간과 검은 점유 영역 사이의 안쪽 경계를 선택하고,
중앙 장애물은 주변 자유 공간과 맞닿는 모서리를 선택했다.
RViz Publish Point로 읽은 map 좌표와 SDF 기준점의 대응은 다음과 같다.

| 기준점 | world x | world y | map x | map y |
|---|---|---|---|---|
| A: 남서 외벽 안쪽 | -2.95 | -1.95 | -0.890425861 | -1.905845046 |
| B: 북서 외벽 안쪽 | -2.95 | 1.95 | -0.890879869 | 1.894753933 |
| C: 중앙 장애물 왼쪽 위 | -0.30 | 0.60 | 1.709671617 | 0.643481076 |

3점을 같은 가중치로 사용해 축척 변경 없는 최소제곱 회전·이동 정합을 계산했다.
평균을 뺀 두 점 집합의 공분산 행렬을 SVD로 분해해 반사 없는 회전을 구하고,
이동량은 `map 평균 - 회전 × world 평균`으로 계산했다.

- 이동량: `(2.04446024, 0.02889593)`m.
- 회전각: `0.00875412`rad, 약 0.502도.
- A/B/C 잔차: 각각 약 0.0411m, 0.0663m, 0.0342m.
- 잔차 RMS: 약 0.0492m. 이 값은 변환의 정확도 보증이나 오차 상한이 아니다.
- A–B 거리: world 3.9m, map 측정 약 3.8006m. 회전·이동만으로 차이를 없앨 수 없다.
- 지도 해상도: 0.05m. 격자화·경계 선택·지도 작성 오차의 영향을 구분하지 못했다.

| 적용한 world→map 변환 `(tx, ty, yaw)` | 위치 오차 | 방향 오차 |
|---|---|---|
| `(2, 0, 0)` | 0.1911m | 8.84도 |
| 위의 3점 근사 정합 | 0.1463m | 9.34도 |

변환에 따라 0.15m 위치 기준의 판정이 바뀌므로 실제 도착 정확도의 통과 여부는
확정하지 않는다. 현재 분석의 `alignment_verified`는 false이다.
목표에 맞추기 위해 변환을 선택하거나 매 순간 AMCL 자세에 맞춰 바꾸지 않는다.

## 자동 테스트와 남은 검증

문서 작성 시 기존 로컬 `build/<패키지>/pytest.xml`에서 다음 결과를 확인했다.
문서 작성 과정에서 빌드·테스트를 새로 실행한 결과는 아니다.

| 패키지 | 테스트 | 오류 | 실패 | skip |
|---|---|---|---|---|
| vehicle_planning | 75 | 0 | 0 | 0 |
| vehicle_bringup | 1 | 0 | 0 | 0 |
| vehicle_simulation | 8 | 0 | 0 | 0 |

기록 기능 테스트는 입력 사본 유지와 경로 재사용 거부를 확인한다.
분석 기능 테스트는 회전·이동, 각도 경계, 표본 시각 선택, 시간 초기화,
액션 취소와 최종 실패의 구분을 확인한다. 관련 패키지 재검증 명령은 다음과 같다.

```bash
source env.sh
colcon test --packages-select vehicle_planning vehicle_bringup vehicle_simulation
colcon test-result --verbose
```

저장 위치를 `evaluation_runs/`로 변경한 뒤 `vehicle_bringup`을 다시 빌드하고
테스트했다. 기본 출력 경로와 명시적 `--run-dir` 유지 동작을 보강했으며,
해당 패키지의 결과는 3 tests, 0 errors, 0 failures, 0 skipped였다.

분석기는 Gazebo 속도의 1초 정지 유지와 충돌을 판정하지 않는다.
`ground_truth_stop_check`, `collision_check`는 `not_evaluated`로 저장된다.
manager의 succeeded 상태는 독립적인 Gazebo 속도·접촉 검증을 대신하지 않는다.
이번 기록 체계로 다른 시나리오를 실행한 검증과 좌표 정합의 정밀 검증은 남아 있다.

## 자료 보관

평가 원본은 `evaluation_runs/`에 보관하며 Git에서 제외한다. `scratch/`는
기타 로컬 작업 자료에 사용한다. 장기 보관할 때는 실행 폴더 전체를 별도 보관소에
복사하고, 저장소에는 절차·결과·남은 문제를 남긴다. 생성된 실행 스크립트와
명령 메타데이터에는 절대 경로가 들어가므로 다른 위치로 복사한 뒤 재실행하려면
경로를 확인해야 한다. bag 재생과 시뮬레이션에서 새 주행을 실행하는 것은 구분한다.
