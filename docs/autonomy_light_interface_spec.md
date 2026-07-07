# autonomy-light 전달 명세서

작성일: 2026-07-08  
대상 패키지: `autonomy_light`  
대상 실행 환경: Jetson + ROS 2 Humble + Livox MID360 + Point-LIO  
주요 수신자: 제어용 SBC 또는 같은 PC의 제어 프로세스

## 1. 목적

`autonomy-light`는 Jetson에서 LiDAR 기반 SLAM과 local map 처리를 수행하고, 제어용 SBC가 바로 사용할 수 있는 compact height map과 odometry를 외부 ROS 2 domain으로 내보내는 경량 perception runtime이다.

핵심 목표는 다음과 같다.

1. MID360 raw LiDAR, IMU, Point-LIO, local map은 내부 domain에 둔다.
2. 제어용 SBC에는 필요한 출력만 보낸다.
3. sim-to-real이 가능하도록 autonomy의 DDS `HeightMap.idl`과 같은 float-array 순서를 유지한다.
4. ROS 2 custom message에 resolution, x/y length metadata를 포함한다.

## 2. ROS 2 Domain 구조

기본 설정:

```yaml
internal_ros_domain_id: 42
external_ros_domain_id: 0
debug_local_map_ros_domain_id: 42
```

### Internal Domain

내부 domain은 LiDAR, Point-LIO, local map 처리를 위한 domain이다. 기본값은 `42`이다.

포함되는 주요 topic:

- `/livox/lidar`
- `/livox/imu`
- `/aft_mapped_to_init`
- `/point_lio/local_map`
- `/cloud_registered`
- `/autonomy_light/heartbeat`

### External Domain

외부 domain은 제어용 SBC가 보는 domain이다. 기본값은 `0`이다.

포함되는 주요 topic:

- `/autonomy_light/height_map_data`
- `/autonomy_light/height_map`
- `/autonomy_light/odom`
- `/path`
- `/tf`
- `/tf_static`

### Debug Local Map

`debug_local_map_ros_domain_id`가 `external_ros_domain_id`와 같으면 `/point_lio/local_map`이 제어망으로 흘러가서 제어용 SBC가 느려질 수 있다. 실기체 기본은 내부 domain과 같은 `42`를 권장한다.

## 3. 실행

Jetson에서 빌드:

```bash
cd ~/ros2_ws/src/cocelo-hd-autonomy-light
./build.sh
```

Livox SDK가 이미 설치되어 있으면:

```bash
./build.sh --skip-sdk
```

실행:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
./launch.sh --real
```

정상 로그 예:

```text
ROS_DOMAIN_ID internal=42
ROS domains: internal=42 external=0 debug_local_map=42 (not republished)
Autonomy light ready: lidar=lidar_link target=base_link ...
```

## 4. 외부 출력 명세

아래 topic은 `external_ros_domain_id`에서 발행된다.

| Topic | Type | QoS | 설명 |
|---|---|---|---|
| `/autonomy_light/height_map_data` | `autonomy_light/msg/HeightMap` | Reliable, depth 2 | 제어용 compact height map |
| `/autonomy_light/height_map` | `sensor_msgs/msg/PointCloud2` | Reliable, depth 2 | height grid를 point cloud로 변환한 debug 출력 |
| `/autonomy_light/odom` | `nav_msgs/msg/Odometry` | Reliable, depth 10 | Point-LIO odometry를 target frame 기준으로 republish |
| `/path` | `nav_msgs/msg/Path` | Reliable, depth 10 | Point-LIO path republish |
| `/tf` | `tf2_msgs/msg/TFMessage` | ROS 기본 TF QoS | `odom -> base_link`, `odom -> base_link_gravity` |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | ROS 기본 static TF QoS | `base_link -> lidar_link` |

### Custom HeightMap 메시지

파일: `msg/HeightMap.msg`

```ros
float32[] data
float32 resolution
float32 x_length
float32 y_length
```

수신 확인:

```bash
ROS_DOMAIN_ID=0 ros2 interface show autonomy_light/msg/HeightMap
ROS_DOMAIN_ID=0 ros2 topic echo /autonomy_light/height_map_data
```

## 5. HeightMap 데이터 계약

### Shape

수신자는 아래 방식으로 grid shape를 계산한다.

```text
width  = ceil(x_length / resolution)
height = ceil(y_length / resolution)
data.size() == width * height
```

현재 기본값:

```text
resolution = 0.05
x_length = 1.2
y_length = 1.2
width = 24
height = 24
data.size() = 576
```

### 순서

`data`는 autonomy DDS `HeightMap.idl`의 `sequence<float> data`와 같은 row-major 순서다.

```text
index = row * width + col
row: y_min -> y_max
col: x_min -> x_max
```

`autonomy-light`의 grid는 로봇 중심이다.

```text
x_min = -0.5 * x_length
x_max =  0.5 * x_length
y_min = -0.5 * y_length
y_max =  0.5 * y_length

x(col) = x_min + (col + 0.5) * resolution
y(row) = y_min + (row + 0.5) * resolution
```

### 값 의미

`autonomy-light` 내부 grid의 `grid_z`는 바닥 0, 장애물 양수인 height 값이다. 외부 `HeightMap.data`는 Isaac/autonomy height scanner style과 맞추기 위해 distance 형태로 변환된다.

```text
base_height = max(0, algorithm.clipping.max_z)
data[index] = clamp(base_height - grid_z, 0, base_height)
```

현재 기본값에서:

```text
base_height = 0.48
flat ground grid_z = 0.0  -> data = 0.48
obstacle grid_z = 0.20    -> data = 0.28
high obstacle grid_z >= 0.48 -> data = 0.0
```

즉, 제어/학습 쪽에서 기존 Isaac height scanner와 같이 “거리값”으로 해석할 수 있다.

## 6. 내부 입력 명세

아래 topic은 기본적으로 `internal_ros_domain_id`에서 사용된다.

| Topic | Type | Producer | Consumer | 설명 |
|---|---|---|---|---|
| `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | Livox driver | Point-LIO | MID360 raw point packet |
| `/livox/imu` | `sensor_msgs/msg/Imu` | Livox driver | Point-LIO | MID360 internal IMU |
| `/aft_mapped_to_init` | `nav_msgs/msg/Odometry` | Point-LIO | autonomy-light | SLAM odometry |
| `/point_lio/local_map` | `sensor_msgs/msg/PointCloud2` | Point-LIO | autonomy-light | height map source |
| `/path` | `nav_msgs/msg/Path` | Point-LIO | autonomy-light | path republish source |
| `/cloud_registered` | `sensor_msgs/msg/PointCloud2` | Point-LIO | autonomy-light optional | optional fill source |

## 7. 주요 파라미터 설명

### Frame

| Parameter | Default | 설명 |
|---|---|---|
| `target_frame` | `base_link` | 제어 기준 로봇 frame |
| `height_map_frame` | `base_link_gravity` | roll/pitch를 제거한 yaw-only height map frame |
| `lidar_frame` | `lidar_link` | LiDAR frame 이름 |
| `target_to_lidar_xyz` | `[0.038335, 0.00162081, 0.138986]` | target frame 기준 LiDAR 위치 |
| `target_to_lidar_rpy` | `[3.1416, 0.0, 0.0]` | target frame 기준 LiDAR 자세 |

### Grid

| Parameter | Default | 설명 |
|---|---|---|
| `elevation_resolution` | `0.05` | height map cell 크기 |
| `elevation_x_length` | `1.2` | x 방향 map 크기 |
| `elevation_y_length` | `1.2` | y 방향 map 크기 |
| `publish_rate_hz` | `50.0` | 외부 출력 publish rate |

### Height Origin

| Parameter | Default | 설명 |
|---|---|---|
| `height_origin.mode` | `local_floor` | z 기준 결정 방식. `local_floor`, `odom`, `fixed` 지원 |
| `height_origin.fixed_z` | `0.0` | fixed 모드 z 기준 |
| `height_origin.filter_alpha` | `0.25` | height origin low-pass 계수 |
| `height_origin.max_step` | `0.03` | 한 frame에서 기준 높이가 변할 수 있는 최대량 |
| `height_origin.floor_radius` | `0.6` | local floor 후보 탐색 반경 |
| `height_origin.floor_percentile` | `0.20` | floor 후보 percentile |
| `height_origin.floor_min_points` | `20` | floor 추정 최소 point 수 |

### Domain

| Parameter | Default | 설명 |
|---|---|---|
| `internal_ros_domain_id` | `42` | LiDAR/SLAM/internal map domain |
| `external_ros_domain_id` | `0` | 제어용 SBC 출력 domain |
| `debug_local_map_ros_domain_id` | `42` | local map debug republish domain |
| `debug_local_map_topic` | `/point_lio/local_map` | debug local map topic |

### Livox Network

| Parameter | Default | 설명 |
|---|---|---|
| `livox_interface` | `enP8p1s0` | MID360 연결 NIC |
| `livox_host_ip` | `192.168.1.50/24` | Jetson LiDAR NIC 고정 IP |
| `livox_lidar_ip` | `192.168.1.166` | MID360 LiDAR IP |

### Topic Names

| Parameter | Default | 설명 |
|---|---|---|
| `raw_lidar_topic` | `/livox/lidar` | Livox raw LiDAR topic |
| `raw_imu_topic` | `/livox/imu` | Livox IMU topic |
| `point_lio_odom_topic` | `/aft_mapped_to_init` | Point-LIO odom input |
| `point_lio_map_topic` | `/point_lio/local_map` | Point-LIO local map input |
| `point_lio_registered_topic` | `/cloud_registered` | optional registered scan input |
| `odom_output_topic` | `/autonomy_light/odom` | external odom output |
| `height_map_topic` | `/autonomy_light/height_map` | external PointCloud2 height map output |
| `height_map_msg_topic` | `/autonomy_light/height_map_data` | external custom HeightMap output |
| `path_output_topic` | `/path` | external path output |

### Backend 및 Filtering

| Parameter | Default | 설명 |
|---|---|---|
| `algorithm.elevation_backend` | `autonomy_min_z` | cell 대표값 선택 방식 |
| `algorithm.clipping.enabled` | `true` | height clamp 사용 여부 |
| `algorithm.clipping.min_z` | `0.0` | 최소 height |
| `algorithm.clipping.max_z` | `0.48` | 최대 height 및 custom HeightMap base height |
| `algorithm.min_z.min_points_per_cell` | `3` | cell 채택 최소 point 수 |
| `algorithm.min_z.supported_min_enabled` | `true` | 낮은 cluster support 기반 min-z 사용 |
| `algorithm.min_z.support_band` | `0.04` | floor cluster z 폭 |
| `algorithm.min_z.obstacle_override_enabled` | `true` | 높은 cluster가 있으면 장애물로 우선 선택 |
| `algorithm.min_z.obstacle_min_height` | `0.06` | 장애물로 볼 최소 상대 높이 |
| `algorithm.min_z.obstacle_min_points` | `2` | 장애물 cluster 최소 point 수 |
| `algorithm.min_z.obstacle_support_band` | `0.06` | 장애물 cluster z 폭 |

### Post Processing

| Parameter | Default | 설명 |
|---|---|---|
| `interpolation_max_passes` | `2` | 빈 cell interpolation 반복 횟수 |
| `interpolation_min_neighbors` | `3` | interpolation 최소 neighbor 수 |
| `fill_remaining_height` | `0.0` | 마지막 남은 빈 cell 기본값 |
| `algorithm.frame_aggregation.temporal_alpha` | `1.0` | 이전 grid와 현재 grid 혼합 비율 |
| `algorithm.isolated_filter.radius` | `1` | isolated noise 판단 반경 |
| `algorithm.isolated_filter.min_support_neighbors` | `3` | 정상 cell로 볼 최소 주변 support |
| `algorithm.hole_fill.radius` | `1` | hole fill 반경 |
| `algorithm.hole_fill.min_neighbors` | `4` | hole fill 최소 neighbor 수 |
| `algorithm.bilateral.passes` | `0` | bilateral smoothing 횟수. 0이면 off |

## 8. 수신 SBC 체크리스트

1. `autonomy_light` 패키지 또는 메시지 인터페이스를 수신 workspace에 포함한다.
2. 수신 SBC에서 `source ~/ros2_ws/install/setup.bash`를 실행한다.
3. `ROS_DOMAIN_ID=0`으로 외부 출력 domain을 맞춘다.
4. `/autonomy_light/height_map_data`를 subscribe한다.
5. `data.size()`가 `ceil(x_length/resolution) * ceil(y_length/resolution)`와 같은지 확인한다.
6. `data`는 row-major distance 배열로 해석한다.

## 9. 빠른 디버그 명령

```bash
ROS_DOMAIN_ID=0 ros2 topic list | grep autonomy_light
ROS_DOMAIN_ID=0 ros2 interface show autonomy_light/msg/HeightMap
ROS_DOMAIN_ID=0 ros2 topic hz /autonomy_light/height_map_data
ROS_DOMAIN_ID=0 ros2 topic echo /autonomy_light/height_map_data --once
```

내부 local map 확인:

```bash
ROS_DOMAIN_ID=42 rviz2
```

외부 domain에 local map이 새는지 확인:

```bash
ROS_DOMAIN_ID=0 ros2 topic list | grep point_lio/local_map
```

정상 실기체 운용에서는 아무것도 나오지 않아야 한다.

