# AGENTS.md

## 환경

- ROS 2 Jazzy 버전을 사용한다.
- 기본 셸은 bash이며, ROS 2 명령을 실행하기 전에 환경을 로드한다.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash  # 워크스페이스를 빌드한 뒤
```

## 빌드 및 테스트

- 워크스페이스 루트에서 다음 명령으로 빌드한다.

```bash
colcon build --symlink-install
```

- 특정 패키지 변경 시에는 우선 해당 패키지를 빌드하고 테스트한다.

```bash
colcon build --packages-select <package_name> --symlink-install
colcon test --packages-select <package_name>
colcon test-result --verbose
```

## ROS 2 개발 규칙

- Python 노드는 `rclpy`, C++ 노드는 `rclcpp` 관례를 따른다.
- 토픽, 서비스, 액션, 파라미터 이름은 소문자와 `_`를 사용한다.
- 메시지 타입, QoS, 토픽 이름, TF frame을 변경하면 관련 노드와 launch 파일도 함께 확인한다.
- 좌표계는 ROS 표준을 따른다: `x`는 전방, `y`는 좌측, `z`는 위쪽이다.
- TF frame 이름은 일관되게 유지하며, 호환성을 깨는 이름 변경은 피한다.

## 자율주행 안전

- 실제 차량을 움직일 수 있는 노드나 launch 설정의 기본 동작은 안전 정지여야 한다.
- 속도, 조향, 가속도 제한값을 변경할 때는 단위와 영향 범위를 명시한다.
- 하드웨어가 없는 환경에서는 시뮬레이션 또는 mock 인터페이스를 우선 사용한다.
- 센서 입력이 끊기거나 유효하지 않으면 제어 명령을 제한하거나 차량을 정지시킨다.

## 변경 원칙

- 기존 사용자 변경 사항을 임의로 되돌리지 않는다.
- 요청과 무관한 대규모 포맷팅이나 리팩터링은 하지 않는다.
- 새 패키지, 노드, launch 파일을 추가하면 실행 방법을 README에 반영한다.
