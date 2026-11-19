# TurtleBot3 Waffle Pi 기본 Bringup 기록

2026-11-19 팀 하드웨어 환경 설정 기록이다. Raspberry Pi에서 ROS 2 Jazzy 실행, OpenCR 연결, LDS-02 회전과 센서 데이터 수신을 확인했다.

| 항목 | 확인한 구성 |
|---|---|
| 로봇·SBC | TurtleBot3 Waffle Pi, Raspberry Pi 4 Model B Rev 1.5, RAM 4GB, microSD 16GB |
| OS·ROS | Ubuntu 24.04.5 LTS ARM64, ROS 2 Jazzy (`ros-jazzy-ros-base`), Python 3.12.3 |
| 로봇 워크스페이스 | 로봇 계정의 `~/ros2_ws`. ROBOTIS TurtleBot3 Jazzy 소스, `ld08_driver`, colcon 사용 |
| OpenCR | OpenCR 1.0, `waffle.opencr` V230127R1, 당시 `/dev/ttyACM0` |
| LiDAR | LDS-02, CP2102, 당시 `/dev/ttyUSB0` |

장치 경로는 실제 연결에서 관찰한 값이다. 다시 연결하거나 장치를 바꾼 경우 실제 포트를 먼저 확인한다. OpenCR 펌웨어 업로드는 CRC 검사와 펌웨어 시작까지 확인했다. 당시 ARM64에서 업데이트 도구를 실행하기 위해 `armhf`와 `libc6:armhf`를 사용했다. 펌웨어를 다시 설치해야 할 때는 C 담당과 장치·펌웨어를 확인하고 공식 절차를 따른다. [ROBOTIS 설치·OpenCR 안내](https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/)

## 준비된 로봇에서 기본 상태 확인

팀에서 확인한 현재 IP로 로봇에 접속한다. 예시는 `ssh robot@<로봇-IP>`이며, 꺾쇠를 포함한 부분을 실제 주소로 바꾼다. 다음 명령은 **로봇의 Bash**에서 실행한다. 로봇 워크스페이스는 빌드된 상태를 전제로 한다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
export LDS_MODEL=LDS-02
ros2 launch turtlebot3_bringup robot.launch.py
```

이 Launch 명령은 ROBOTIS Jazzy 기본 절차다. 팀 공용 패키지의 `amr_bringup`은 별도 개발 영역이다. [ROBOTIS Bringup 안내](https://emanual.robotis.com/docs/en/platform/turtlebot3/bringup/)

별도 로봇 터미널에서 같은 ROS 환경을 적용하고, Bringup 실행 중 아래 읽기 명령으로 확인한다.

```bash
ros2 node list
ros2 topic hz /scan
ros2 topic echo /odom --once
ros2 topic type /cmd_vel
```

- 로봇에서 Talker/Listener와 노드 검색이 성공했다. `/scan`의 거리 배열을 약 **9.42 Hz**로 수신했고 `/odom` 메시지도 수신했다. 이 수치는 당시 관측값이다. `/scan` 배열에 포함된 일부 `nan`은 유효 거리에서 제외한다.
- 관찰한 주요 토픽은 `/scan`, `/odom`, `/imu`, `/joint_states`, `/tf`, `/tf_static`, `/battery_state`, `/sensor_state`, `/cmd_vel`이다. 토픽 목록에서 확인한 것과 데이터·TF를 검증한 것은 구분한다.
- **D의 실차 연동 기준**은 관측된 `/cmd_vel` 타입인 `geometry_msgs/msg/TwistStamped`다. 연결 전 Nav2 출력 타입과 현재 로봇 구독 타입을 대조한다. [ROBOTIS Jazzy 속도 메시지 안내](https://emanual.robotis.com/docs/en/platform/turtlebot3/bringup/)
- A↔D의 별도 프로세스 MOCK은 기존 통합 순서대로 진행한다. 실차 검증에 앞서 C·D가 개발 PC와 로봇 사이의 DDS, TF, 안전 정지 수단을 확인한다. [A↔D 인수인계](INTEGRATION_A_D_HANDOFF.md) · [실차 검증 기준](SIM_TO_REAL.md)

**미검증**은 바퀴 구동 및 속도 명령, SLAM·지도, Nav2 주행·장애물 회피, 물류 미션, 재부팅 후 자동 Bringup, 다른 PC와의 DDS 통신이다. 따라서 실차 Gate 0 전체 통과 판정은 보류한다.

## 설정 중 해결한 문제

- 패키지 버전이 어긋난 APT 의존성 오류는 누락된 `noble-updates` 저장소를 추가한 뒤 해결했다.
- LDS-02 드라이버는 검색됐지만 센서가 돌지 않고 `/scan`이 오지 않는 문제는 센서 케이블을 연결한 뒤 해결됐다.
