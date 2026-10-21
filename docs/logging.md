# 로그와 선택적 주행 기록

노드의 상태·경고·오류 로그는 평소에도 출력된다. ROS 로그 파일은 기본적으로
`~/.ros/log`에 저장되며 `ROS_LOG_DIR`로 변경할 수 있다.
토픽 데이터를 저장하는 rosbag 기록은 기본적으로 꺼져 있다.

## 기록 시작

워크스페이스 루트에서 실행한다.

```bash
source env.sh
ros2 launch vehicle_bringup system.launch.py record:=true
```

기본 navigation 시뮬레이션과 함께 기록이 시작된다. 수동·mapping 모드에서도
같은 옵션을 사용할 수 있다. 모사 하드웨어에서는 다음처럼 실행한다.

```bash
ros2 launch vehicle_bringup system.launch.py backend:=hardware hardware_driver:=mock mode:=manual record:=true
```

시작 메시지에 출력되는 `log/bags/<UTC 시각>_<고유값>/`에 MCAP 파일과
`metadata.yaml`을 저장한다. 상대 경로는 명령을 실행한 디렉터리 기준이다.
기록 경로를 바꿔도 ROS 텍스트 로그의 위치는 바뀌지 않는다.

| 인자 | 기본값 | 의미 |
|---|---|---|
| `record` | `false` | 토픽 기록 활성화 |
| `record_dir` | 자동 생성 | 새 bag 디렉터리. 기존 경로는 사용 불가 |
| `record_duration_sec` | `300` | recorder 시작부터 실제 시간 기준 기록 제한, 초 |
| `record_max_bag_size` | `268435456` | 파일 분할 기준, 바이트(256 MiB) |

예를 들어 60초 동안 명시한 새 경로에 기록하려면:

```bash
ros2 launch vehicle_bringup system.launch.py record:=true record_duration_sec:=60 record_dir:=log/bags/manual_check_01
```

제한 시간이 지나면 recorder만 종료되고 주행은 계속된다. Gazebo를 일시정지해도
기록 시간은 흐른다. 전체 launch를 Ctrl+C로 종료하면 기록도 함께 마감한다.
파일 마감에는 시간이 걸릴 수 있으므로 종료 메시지를 기다린다.
시간 제한은 데이터 수신 준비 완료 시점이 아닌 recorder 프로세스 시작부터 계산한다.

기록 프로세스가 조기에 종료되거나 실패하면 저장 경로·종료 코드가 경고로 출력된다.
기록 실패로 목표를 취소하거나 주행 노드를 종료하지 않는다.

## 데이터와 저장량

기록 대상은 [recording.yaml](../src/vehicle_bringup/config/recording.yaml)에 있다.
센서·TF·odom·지도·제어 명령·구동기 상태·주행 이벤트·`/rosout`을 기록하고,
시뮬레이션에서만 `/clock`과 `/ground_truth/odom`을 추가한다.
현재 모드에서 발행하지 않는 토픽이 bag에 없는 것은 정상이다.
늦게 시작하는 발행자는 자동으로 발견한다.

bag 수신 시각은 시스템 시각이다. 시뮬레이션 측정 시각은 메시지 헤더와 `/clock`으로
확인한다. 기본 캐시는 16 MiB이며 rosbag 이중 버퍼와 저장 플러그인 때문에
전체 메모리 사용량은 이 값보다 클 수 있다.

**256 MiB는 파일 분할 기준이며 총 저장량 제한이 아니다.** 기록을 반복하면
자료는 누적된다. 짧은 제한 시간을 사용하고 필요 없는 실행 폴더는 직접 정리한다.
`log/`는 Git에서 제외된다. 소스 사본·해시·성능 분석 보고서는 만들지 않는다.

종료된 기록의 토픽·메시지 수는 다음처럼 확인한다.

```bash
ros2 bag info log/bags/manual_check_01
```

## 개발 검증

일반 패키지 테스트는 recorder를 실행하지 않는다. 실제 파일 기록·종료와
simulation/mock hardware 확인은 다른 Gazebo 시험과 겹치지 않게 명시적으로 실행한다.
시험 자료는 임시 경로에 만들고 종료 후 삭제한다.

```bash
source env.sh
python3 -c "import os, pytest; os.environ['RUN_RECORDING_INTEGRATION']='1'; raise SystemExit(pytest.main(['src/vehicle_bringup/test/test_recording_integration.py', '-q']))"
```
