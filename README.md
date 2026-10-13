# ur10_proxskin_test

브라우저에서 보는 간단한 실행 가이드: [how_to_use.html](how_to_use.html) (명령어 복사·모드별 경로 미리보기).

## 웹 버튼으로 실행

터미널에서 로컬 웹 서버를 한 번 실행합니다. 기존 실험 launch는 먼저 종료합니다.

```bash
source /opt/ros/humble/setup.bash
source /home/ur/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=125
ros2 run ur10_proxskin_test web_control
```

명령을 실행하면 기본 브라우저에서 [http://127.0.0.1:8765](http://127.0.0.1:8765)가 자동으로 열립니다.
**드라이버 실행 → 실험 메뉴 실행 → 모드 선택 → 실행** 순서로 누릅니다.
드라이버 버튼은 `ros2 run urcb2_driver single_arm`, 메뉴 버튼은 `ros2 launch ur10_proxskin_test proxskin_soc_test.launch.py`를 실행합니다.
이미 같은 ROS_DOMAIN_ID로 드라이버를 켰다면 메뉴 실행부터 진행합니다.

[control.html](control.html)은 시작점 설정(y/n), Z 이동, 정렬(a), 조정 완료(e), stroke/왕복 횟수/정지 시간 입력, 중단과 CSV 다운로드를 제공합니다.
CSV 그래프는 X축 시간(s), 왼쪽 Y축 2열 로봇 거리(mm), 오른쪽 Y축 3열 센서 raw 값을 표시합니다.
각 체크박스로 선을 켜고 끌 수 있으며, 마우스를 올리면 해당 시간과 값을 확인할 수 있습니다.
CSV 목록에서 이전 파일도 클릭해 표시할 수 있습니다. 선택한 파일은 새 기록이 생겨도 유지됩니다. 최신 CSV 보기를 누르면 최신 파일 자동 갱신으로 돌아갑니다. 실행 중에는 새로고침으로 현재까지 저장된 데이터를 확인합니다.
그래프 위에서 휠로 확대·축소, 드래그로 이동합니다. 확대/축소 버튼도 사용할 수 있으며, 전체 보기 또는 더블클릭으로 범위를 초기화합니다.
CSV의 모든 행을 생략 없이 표시합니다. 확대하면 해당 구간의 원본 점을 모두 확인할 수 있습니다. 센서 빈칸은 선을 끊어 표시하며 원본 CSV는 그대로 유지됩니다.
HTML 파일을 직접 열면 서버 실행 방법이 표시됩니다. 실제 제어는 위 localhost 주소에서 합니다.
서버 실행과 실험 메뉴 열기는 이동 명령을 보내지 않습니다. 모드 선택 후 실행 버튼으로 동작을 시작합니다.
작업이 끝나면 모드 선택 화면으로 돌아옵니다. 2~4번 중단 시 CSV를 닫고 1번에서 저장한 시작점으로 복귀한 뒤 메뉴로 돌아옵니다.
복귀 중에는 상태가 표시되고, 다시 중단을 누르면 복귀도 멈춥니다. 0/1번 중단은 현재 작업만 취소합니다.

브라우저 탭을 닫아도 진행 중인 작업은 계속됩니다. 서버 터미널의 Ctrl+C는 웹에서 실행한 실험과 드라이버를 종료합니다.
외부 터미널에서 실행한 드라이버는 해당 터미널에서 종료합니다. 서버는 127.0.0.1에만 연결되며 다른 컴퓨터의 접속을 받지 않습니다.
같은 ROS_DOMAIN_ID에서 별도 실험 launch와 웹 제어를 동시에 실행하지 마세요.
다른 포트는 `ros2 run ur10_proxskin_test web_control --port 8766`으로 지정합니다.
브라우저 자동 실행을 끄려면 `--no-browser`를 붙입니다.

## 로봇 거리·XZ 평면 실험 메뉴

| 번호 | 동작 |
| --- | --- |
| **0 / base** | J1/J6 현재 각도 유지, J2~J5 `[-90, -90, -90, 90]`도로 이동. 도착 후 실험 기준을 만들려면 1번을 선택합니다. |
| **1 / setup** | 위치 조정·툴 Z 기울기 정렬·입력한 거리(mm) 기준 저장. 정렬 중 XYZ/J6 유지. |
| **2 / vertical** | 저장한 시작점으로 복귀 → 입력한 거리만큼 베이스 -Z 하강 → 시작점 복귀. X/Y와 TCP 방향 고정. |
| **3 / columns** | 각 X 위치에서 2번 Z 왕복을 수행한 뒤, 시작 Z 높이에서 다음 X 열로 이동. Y와 TCP 방향 고정. |
| **4 / stepped** | 2번 경로에서 하강·상승 모두 10mm마다 정지. 정지 시간 선택(기본 2초). |

### 1번 설정 흐름

1. `Use current position as start? (y/n, q to cancel)`을 묻습니다.
2. `y`: `Start distance (mm):`에 원하는 양수 거리를 입력합니다(`100`, `200`, `300`, `125.5` 등, 소수 허용).
   마지막으로 현재 TCP XYZ와 J6를 유지한 채 툴 Z를 월드 Z에 평행하게 정렬합니다.
   실제 도달한 XYZ/RPY와 6개 관절값, 거리 기준을 저장합니다.
3. `n`: 펜던트로 베이스 Z축 위치를 조정하거나, 터미널에서 `+10`, `-1`처럼 정수 mm를 입력합니다.
   터미널 조정은 그 시점의 X/Y, TCP 방향, J6를 유지합니다. 입력을 기다리는 동안에는 유지 명령을 보내지 않습니다.
4. `Z adjustment:`에서 `a`: 그 시점의 현재 XYZ/J6를 유지하며 툴 Z 기울기를 정렬한 뒤 조정을 계속합니다.
   이 동작은 시작점 파일을 저장하지 않습니다. 펜던트로 움직인 뒤에도 현재 피드백을 기준으로 정렬합니다.
5. `e`: 위치 조정에서 나와 다시 현재 위치 사용 여부를 묻습니다. `q`는 저장 없이 설정을 종료합니다.

1번 정렬은 툴 Z 방향만 맞추며, yaw/X/Y 방향을 90도 단위로 정렬하지 않습니다.
J6 각도를 고정하며 월드 yaw 각도 자체를 지정하지 않습니다. `a`로 정렬했어도 거리 입력 후 최종 정렬을 다시 확인합니다.

1번에서 기본 자세로 자동 이동하지 않습니다. 기본 자세로 시작하려면 먼저 0번을 선택합니다.
기존 5번의 수동 Z 조정은 1번에 통합됐습니다. 메뉴에서 별도의 센서 0cm 파일을 만들지 않습니다.

### 실행 순서

두 터미널에서 ROS 환경을 읽고 같은 `ROS_DOMAIN_ID`를 사용합니다.

```bash
source /opt/ros/humble/setup.bash
source /home/ur/ros2_ws/install/setup.bash
```

드라이버 터미널:

```bash
ros2 run urcb2_driver single_arm
```

실험 터미널:

```bash
ros2 launch ur10_proxskin_test proxskin_soc_test.launch.py
```

선택 순서: **0번 기본 자세 → 1번 위치/거리 설정 → 2·3·4번 실험**.
모든 메뉴 동작이 끝나면 번호 선택 화면으로 돌아옵니다. 2~4번에서 왕복 횟수를 입력합니다.
메뉴의 2~4번 실행 중에는 **q 키**로 취소할 수 있습니다(Enter 불필요).
시작점 복귀, 경로 계산, 이동, 끝점/정지 대기 중에도 적용됩니다.
취소하면 실험 이동을 멈추고 그때까지의 CSV를 닫은 뒤 1번에서 저장한 6개 관절 위치로 저속 복귀합니다.
실험 시작 전과 같은 관절 복귀 경로를 사용하므로 복귀 중에는 TCP XYZ와 방향이 함께 바뀔 수 있습니다.
도착 후 모드 선택 화면으로 돌아가며, 복귀 동작은 CSV에 추가하지 않습니다. 드라이버는 계속 실행합니다.
복귀 중 q를 다시 누르면 복귀도 멈추고 메뉴로 돌아옵니다. 피드백/안전 검사 실패 시에는 복귀를 중단합니다.
터미널 입력 상태는 메뉴로 복귀하기 전에 원래대로 복원됩니다.
거리 기준은 1번에서 저장한 값을 사용하며, 실험 때마다 새로 입력하지 않습니다.
예를 들어 `200`을 저장하면 그 위치를 200mm로 기록하고, 베이스 -Z로 200mm 내려간 위치를 0mm로 기록합니다.
4번도 입력한 거리의 끝점까지 이동합니다. 거리가 10mm의 배수가 아니면 끝점과 가장 가까운 10mm 정지 위치 사이의 구간이 짧아집니다.

시작점 파일은 `/home/ur/.ros/ur10_proxskin_test/experiment_start.json`입니다.
버전 2에 `start_distance_mm`, `tcp_pose`, `joint_rad`, `orientation_aligned`, `robot_name`, 저장 시각을 포함합니다.
이전 파일에는 거리 기준이 없으므로 새 1번으로 다시 저장해야 합니다.
2~4번은 시작 전에 저장한 6개 관절값으로 복귀하며, 복귀 중에는 XYZ가 바뀔 수 있습니다.

1번 정렬은 기본 5°/s, 관절 속도 0.1rad/s입니다. 정렬 경로 전체의 IK를 먼저 확인합니다.
현재 TF 모델의 `world`/`base_link`와 컨트롤러 `base`는 Z 기준 180도 차이가 있어 X/Y 방향이 반대이고 Z 방향은 같습니다.
툴 Z 정렬은 월드 Z를 기준으로, 이동은 컨트롤러 base 축을 기준으로 합니다.

### 실제 끝점 도달 확인

하단·상단·X와 Z 이동이 전환되는 지점에서는 목표 관절 명령을 계속 보내며 실제 TCP 피드백을 확인합니다.
위치 오차 **0.1mm 이하**, 방향 오차 **0.1° 이하**가 **0.25초** 유지되어야 다음 구간으로 진행합니다.
**5초** 내에 도달하지 못하면 실험을 중단하고 그때까지 기록한 CSV를 닫습니다.
거리 기록에는 실제 피드백 오차가 반영되며, 0mm나 저장한 시작 거리 값에 강제로 맞추지 않습니다.
2·4번은 Z 경로 전체 IK를 확인하고, 3번은 진행 중 각 경유점 IK 실패 시 중단합니다.

### 3번 X 범위

기본은 시작 X 기준 **+30cm → -30cm**, 전체 60cm, **1mm 간격(601개 위치)**입니다.
각 X 위치에서 Z 왕복을 모두 마친 다음 상단에 도착한 것을 확인하고 옆으로 이동합니다.
모든 열이 끝나면 마지막 X 위치의 시작 Z 높이에서 종료합니다.

```bash
# 시작 X부터 +X로 30cm, 10mm 열 간격
ros2 launch ur10_proxskin_test proxskin_soc_test.launch.py x_direction:=positive x_span_mm:=300 spacing_mm:=10
```

### 메뉴 없이 실행

```bash
# 현재 XYZ/J6를 유지하며 툴 Z를 정렬하고 200mm 기준 저장(양수 mm, 소수 허용)
ros2 launch ur10_proxskin_test proxskin_soc_test.launch.py mode:=capture start_distance_mm:=200
# 저장한 거리만큼 Z 왕복 2회
ros2 launch ur10_proxskin_test proxskin_soc_test.launch.py mode:=vertical repeat_count:=2
# 각 X 열에서 왕복 1회
ros2 launch ur10_proxskin_test proxskin_soc_test.launch.py mode:=columns repeat_count:=1
# 10mm마다 3초 정지하며 왕복 2회
ros2 launch ur10_proxskin_test proxskin_soc_test.launch.py mode:=stepped repeat_count:=2 dwell_seconds:=3
```

메뉴 없이 실행하면 수행 후 launch가 종료됩니다.
메뉴 없이 실행한 2~4번도 q 키로 취소할 수 있으며, 이 경우 메뉴를 열지 않고 launch가 종료됩니다.
2~4번은 기본으로 센서에 연결한 뒤 `data/`에 시간·거리·센서값을 100Hz로 저장합니다.
센서 없이 로봇만 실행하려면 `enable_ecan:=false`를 붙입니다.

## 1. 패키지 역할

이 패키지는 크게 네 가지 일을 수행합니다.

1. `urcb2_driver`를 통해 현재 UR10 관절 상태를 읽고 목표 관절 명령을 전송합니다.
2. 메뉴 실행 시 1번으로 저장한 시작 위치/자세로 복귀한 뒤, 그 TCP를 기준으로 Z 왕복 또는 X-Z 열 스캔 경로를 생성합니다.
3. Prox_sensor_React와 같은 방식으로 eCAN TCP 서버에 클라이언트로 연결하고, 지정한 CAN ID들의 센서 프레임을 계속 받아 최신 값을 유지합니다.
4. 메뉴 실험 중 시간, 실제 관절 피드백으로 계산한 거리(mm), 센서값을 CSV로 저장합니다.

즉, "로봇 움직임"과 "근접 센서 데이터 수집"을 한 노드 안에서 묶어 실험용 데이터셋을 만드는 패키지라고 보면 됩니다.

## 2. 동작 흐름

실행 순서는 아래와 같습니다.

1. `urcb2_driver`의 `single_arm` 노드가 먼저 실행되어 있어야 합니다.
2. `proxskin_test` 노드는 `/UR10_right/joint_states`와 `/UR10_right/targetJ` 같은 토픽에 연결합니다.
3. `enable_ecan`이 켜져 있으면 `can_server_ip:can_server_port`에 접속해 선택한 CAN ID의 최신 프레임을 기다립니다. 0/1번은 센서 연결 없이 동작합니다.
4. 메뉴의 2~4번은 1번으로 저장한 시작 자세로 이동합니다. 0번은 J1/J6를 유지하며 기본 자세로 이동하고, 1번은 현재 XYZ/J6를 유지하며 툴 Z를 정렬하고 거리 기준을 저장합니다.
5. 메뉴의 2~4번은 저장된 시작 TCP 위치와 방향을 실험 기준점(origin)으로 사용합니다.
6. 메뉴의 2번은 시작점에서 -Z로 내려갔다 돌아오고, 3번은 각 X 열에서 같은 왕복을 수행합니다. 각 경유점마다 수치 IK를 풀어 조인트 명령을 만듭니다.
7. 경로를 따라 이동하면서 실제 피드백에서 계산한 로봇 거리와 시간을 CSV에 기록합니다.
8. 스캔이 끝나면 마지막 자세에서 잠시 유지하고 CSV를 닫은 뒤 메뉴로 돌아갑니다.

메뉴의 스캔 기준점은 1번으로 저장한 시작점입니다. 센서 0cm 캘리 파일과는 별도로 저장됩니다. 기존 `proxskin_test.launch.py`로
직접 실행하는 래스터는 별도 센서 0cm 설정 없이 base 초기 자세를 기준으로 동작합니다.

## 3. 주요 기능

### 3.1 래스터 스캔 생성

메뉴 3번은 `include/ur10_proxskin_test/experiment_trajectory.hpp`에서 각 X 열의 Z 하강·복귀 경로를 만듭니다. 기존 직접 실행의 `src/proxskin_test.cpp` 래스터는 Y-Z 평면의 지그재그 경로를 유지합니다. `x_offset_mm`, `y_span_mm`, `z_span_mm`, `spacing_mm` 파라미터로 스캔 범위를 정의하고, X는 기준점에서 고정된 오프셋만큼만 이동합니다.

### 3.2 수치 IK 기반 자세 계산

각 스캔 포인트에 대해 해석해 기반 IK 대신 수치 미분 자코비안과 Levenberg-Marquardt 방식의 수치 IK를 사용합니다. 기존 포팅 과정에서 해석해 왕복 정확도 문제가 있어, 현재 패키지는 보다 안정적인 수치 IK 쪽에 맞춰져 있습니다.

### 3.3 eCAN TCP 수신

노드는 자체적으로 TCP 서버를 열고 eCAN 송신 장치가 접속하기를 기다립니다. 수신 프레임은 14바이트 형식으로 해석하며, 각 프레임에서 CAN ID, DLC, front 값, back 값을 추출합니다. `tracked_can_ids`에 등록된 ID만 최신 값으로 유지하고 로그에 반영합니다.

### 3.4 CSV 기록

메뉴 2~4번은 `csv_save_dir`(기본: 패키지의 `data/`)에 실험마다 새 CSV를 생성합니다.
파일명은 `experiment_vertical_날짜_시간_마이크로초.csv`, `experiment_columns_...csv`, `experiment_stepped_...csv`입니다.

```csv
time_s,robot_distance_mm,sensor_value
0.000000,300.00,8590941
0.010000,299.50,8590941
0.020000,299.00,8590969
```

위 숫자는 형식을 설명하기 위한 예시입니다. 3열에는 수신한 0x41 앞 raw 정수값을 변환 없이 기록합니다.
첫 열은 첫 기록 프레임 이후의 경과 시간(초, 소수점 6자리), 둘째 열은 거리(mm, 소수점 2자리), 셋째 열은 센서값입니다.
거리 = 저장한 시작 거리(mm) + (현재 TCP Z - 저장한 시작점 TCP Z) × 1000입니다.
30cm 실험에서 시작점은 300.00mm, 아래 끝점은 0.00mm입니다. X 방향 이동은 이 Z 거리 계산에 영향을 주지 않습니다.
실제 관절 피드백으로 계산하며 목표값으로 치환하거나 범위 밖의 측정 오차를 잘라내지 않습니다.
기록은 시작 자세 복귀와 경로 준비가 끝난 뒤 실제 실험 시작부터 마지막 유지 구간까지 진행합니다.
독립된 10ms 타이머(100Hz)가 최신 관절 피드백과 센서값을 같은 행에 기록합니다. 로봇 제어는 125Hz로 유지합니다.
시간은 실제 기록 시각을 기준으로 하므로 운영체제 스케줄링에 따른 작은 주기 오차가 반영됩니다.
센서의 실제 수신 속도가 100Hz보다 느리면 새 값이 들어오기 전까지 최근 유효 센서값을 반복 기록합니다.

수신 대상은 CAN `0x41`의 front 4바이트(big-endian uint32)입니다.
`Prox_sensor_React/src/EthernetCommunication.cpp`의 TCP 접속·14바이트 프레임·정수 해석 방식을 사용합니다.
기본 서버는 `192.168.253.225:4001`이며, 별도 센서 노드를 실행할 필요 없이 각 실험 노드가 직접 연결합니다.
연결이 끊기거나 `sensor_stale_timeout_sec`(기본 0.5초)보다 오래된 값은 빈칸으로 둡니다.
처음에는 선택한 센서의 최신 데이터가 들어와야 실험을 시작합니다. 재연결 시 이전 데이터와 미완성 프레임은 버립니다.

기존 `proxskin_test.launch.py` 직접 실행의 센서 CSV는 기존 `proxskin_YYYY-MM-DD_HH-MM-SS.csv`와 아래 컬럼을 유지합니다.

- `time`
- `x`
- `y`
- `z`
- `<CAN_ID>_front`
- `<CAN_ID>_back`

예를 들어 `tracked_can_ids: [0x41, 0x42, 0x43]`이면 `41_front`, `41_back`, `42_front`, `42_back`, `43_front`, `43_back` 컬럼이 추가됩니다.

## 4. 사용 방법

### 4.1 빌드

워크스페이스 루트에서 다음처럼 빌드합니다.

```bash
cd /home/ur/ros2_ws
colcon build --packages-select ur10_proxskin_test
source install/setup.bash
```

### 4.2 드라이버 실행

현재 `urcb2_driver`의 `single_arm` 실행 파일은 launch 파라미터로 IP를 받는 구조가 아니므로, 먼저 별도로 실행해야 합니다.

```bash
ros2 run urcb2_driver single_arm
```

### 4.3 proxskin 테스트 실행

기본 설정 파일을 사용할 때:

```bash
ros2 launch ur10_proxskin_test proxskin_test.launch.py
```

직접 실행할 때:

```bash
ros2 run ur10_proxskin_test proxskin_test
```

다른 파라미터 파일을 쓰고 싶을 때:

```bash
ros2 launch ur10_proxskin_test proxskin_test.launch.py \
  config_file:=/path/to/proxskin_test.yaml
```

## 5. 파라미터 설명

기본 파라미터 파일은 `config/proxskin_test.yaml`입니다.

- `robot_name`: 조인트 상태와 목표 조인트 토픽 prefix입니다. 기본값은 `UR10_right`입니다.
- `enable_ecan`: false이면 센서 연결 없이 실행합니다. 메뉴 실험은 이 경우에도 로봇 거리 CSV를 저장합니다.
- `can_server_ip`: 연결할 eCAN 서버 IP입니다(기본 192.168.253.225).
- `can_server_port`: eCAN 서버 포트입니다(기본 4001).
- `reconnect_interval_sec`, `socket_connect_timeout_sec`: 재연결 간격과 연결 대기 시간입니다(기본 각 1초).
- `sensor_stale_timeout_sec`: 센서값 유효 시간입니다(기본 0.5초).
- `initial_arm_joint_deg`: J2~J6 초기 각도 5개입니다. 메뉴 0번은 J1/J6를 유지하므로 J2~J5 값만 적용합니다.
- `base_hold_j6`: base 모드에서 J6의 현재 각도를 유지합니다(기본 true).
- `initial_move_velocity_rad_s`: 초기 자세 및 base 모드 관절 이동 속도 제한입니다(기본 0.1 rad/s).
- `base_mode`: 센서와 스캔 없이 공통 초기 자세로만 이동합니다(메뉴 launch에서 설정).
- `calibration_mode`, `calibration_file`: Z축 수동 조정과 센서 0cm 파일 저장입니다.
- `calibration_speed_mm_s`: 캘리 조정 속도이며 기본값은 10mm/s입니다.
- `start_setup_mode`: 1번의 y/n, Z 조정, a 정렬, e 복귀와 거리 기준 선택 흐름입니다.
- `capture_start_mode`, `start_distance_mm`, `start_file`: 자동 실행에서 XYZ/J6를 유지하며 툴 Z를 정렬하고 입력한 양수 mm 시작 기준을 저장합니다. 소수도 허용하며 자동 실행의 기본값은 300mm입니다.
- `alignment_world_from_base_rpy_deg`: 월드 기준 컨트롤러 base의 방향입니다. 현재 TF에 맞춘 기본값은 `[0, 0, 180]`도입니다.
- `alignment_angular_speed_deg_s`, `alignment_joint_speed_rad_s`: 1번 정렬 속도 제한입니다(기본 5°/s, 0.1rad/s).
- `experiment_start_enabled`, `experiment_start_pose`, `experiment_start_joint_rad`: 메뉴 launch가 시작점 파일에서 읽어 전달하는 실험 기준입니다.
- `sensor_zero_enabled`, `sensor_zero_pose`, `sensor_zero_joint_rad`: 기존 센서 0cm 기준을 직접 노드에 전달할 때 사용할 수 있습니다.
- `x_direction`: 메뉴 3번의 X 진행 방향입니다(`centered`, `positive`, `negative`).
- `tracked_can_ids`: 수집해서 CSV에 기록할 CAN ID 목록입니다.
- `csv_save_dir`: CSV 저장 디렉터리입니다.
- `csv_sensor_can_id`, `csv_sensor_channel`: 메뉴 CSV의 센서 열 선택입니다. 기본 ID는 0x41이고 channel은 front입니다. -1이면 센서 칸을 비웁니다.
- `control_rate`: 조인트 명령 publish 주기입니다.
- `x_offset_mm`: 기준 TCP에서 X 방향으로 떨어진 스캔 평면 오프셋입니다.
- `y_span_mm`: 스캔 폭입니다.
- `z_span_mm`: 스캔 높이입니다.
- `spacing_mm`: 래스터 줄 간격입니다.
- `scan_speed_mm_s`: 스캔 구간에서 목표 선속도입니다.
- `approach_vel_rad_s`: 기존 래스터 초기 접근 이동에 사용할 관절 속도 제한입니다.
- `max_joint_velocity_rad_s`: 스캔 중 관절 보간 시 사용할 최대 관절 속도 제한입니다.

주석 처리된 옵션도 있습니다.

- `use_fixed_orientation`, `roll_deg`, `pitch_deg`, `yaw_deg`: 스캔 중 TCP 자세를 고정하고 싶을 때 사용합니다.
- `fix_wrist_joints`, `wrist1_deg`, `wrist2_deg`, `wrist3_deg`: 손목 관절을 고정하고 어깨/팔꿈치 위주로 IK를 풀고 싶을 때 사용합니다.

## 6. 파일별 의미

- `src/proxskin_test.cpp`
  메인 노드입니다. 파라미터 선언, eCAN TCP 서버, 조인트 상태 구독, 래스터 경로 생성, 수치 IK, 스캔 실행, CSV 저장 로직이 모두 들어 있습니다.
- `src/arm_function.cpp`
  UR10 전진기구학, 회전 변환, 트래피조이드 관절 궤적 생성 같은 수학/보조 기능 구현 파일입니다.
- `include/ur10_proxskin_test/arm_function.h`
  위 보조 기능의 선언부입니다. UR10의 DH 파라미터와 궤적 생성 API가 정의되어 있습니다.
- `config/proxskin_test.yaml`
  실험에 필요한 기본 파라미터 파일입니다. 시작 자세, TCP 서버 주소, 스캔 범위, 저장 경로 등을 여기서 조정합니다.
- `launch/proxskin_test.launch.py`
  파라미터 파일을 로드해서 `proxskin_test` 노드를 실행하는 launch 파일입니다.
- `CMakeLists.txt`
  라이브러리/실행 파일 빌드 및 `launch`, `config`, `include` 설치 규칙이 들어 있습니다.
- `package.xml`
  패키지 메타데이터와 ROS 2 의존성 정보입니다.
- `how_to_use.txt`
  간단 실행 메모 파일입니다.
- `data/`
  실험 중 생성되는 CSV 로그 위치입니다. 저장소에는 예제 데이터 대신 빈 디렉터리만 유지합니다.

## 7. 입력/출력 인터페이스

### 구독 토픽

- `/<robot_name>/joint_states`

현재 로봇 관절 상태를 받아 실제 자세를 추적합니다.

### 발행 토픽

- `/<robot_name>/targetJ`

목표 관절 각도를 publish해서 `urcb2_driver`를 통해 로봇을 움직입니다.

### 네트워크 입력

- TCP client → `<can_server_ip>:<can_server_port>` (기본 `192.168.253.225:4001`)

실험 노드가 eCAN 서버에 접속합니다. Prox_sensor_React와 같은 14바이트 프레임을 수신합니다.

### 파일 출력

- `<csv_save_dir>/proxskin_*.csv`

스캔 중의 TCP 위치와 CAN 센서 값이 저장됩니다.

## 8. 실험 시 주의사항

- 실행 전 `robot_name`, IP/포트, 시작 자세가 실제 장비와 맞는지 먼저 확인해야 합니다.
- `spacing_mm`를 너무 작게 잡으면 경유점 수가 급격히 늘어 스캔 시간이 길어질 수 있습니다.
- `tracked_can_ids`에 맞는 프레임이 들어오지 않으면 스캔 시작 전 대기 시간이 길어질 수 있습니다.
- `csv_save_dir`는 쓰기 가능한 경로여야 합니다.
- 이 패키지는 실제 UR10을 움직이므로, 충분한 작업 공간과 비상 정지 수단을 확보한 상태에서 사용해야 합니다.

## 9. 저장소 정책

이 저장소에는 코드와 설정 파일만 포함하고, 실험 중 생성된 CSV 데이터는 포함하지 않습니다. 데이터 로그는 로컬 `data/` 디렉터리에 계속 저장되며 Git에서는 제외됩니다.
