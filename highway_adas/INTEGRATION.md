# Baseline Integration Contract

baseline 담당자는 ROS adapter에서 입력을 만들고 `HighwayDecisionModule::update()`를 한 번
호출하면 된다. Region과 Lane Change FSM을 adapter에 다시 구현하지 않는다.

## 입력

```cpp
HighwayDecisionInput input;
input.region = current_highway_region;
input.distance_to_guard_stop_m = distance_to_last_safe_merge_start;
input.distance_to_next_limit_m = distance_to_next_60_kph_point;

input.adas.ego = ego_at_now;
input.adas.perception_healthy = object_message.sensor_healthy;
input.adas.detection_stamp_sec = object_message_stamp;
input.adas.detection_ego = interpolateEgoPose(object_message_stamp);
input.adas.detections = convertObjectsToBaseLink(object_message);
input.adas.lanes = highway_lane_centerlines;

HighwayDecisionOutput output = highway.update(input);
```

필수 입력 계약:

- Ego와 차로 centerline은 map 좌표계다.
- 객체 중심과 크기는 `base_link`, m 단위다. x는 전방, y는 좌측이다.
- `detection_ego`는 객체 timestamp에 보간한 Ego pose다. 오차가 0.05초를 넘으면 거부한다.
- `distance_to_guard_stop_m`는 현재 진행방향을 따라 마지막 안전 변경 시작선까지 잰 거리다.
  이 선은 소멸점보다 최소 차로변경 길이 45 m 이상 앞에 둔다.
- 모든 모듈 경계 값은 SI 단위다. km/h는 표시와 제어 메시지 변환에서만 사용한다.

객체 ID, 속도, 차로 ID는 필수가 아니다. 코어가 위치와 크기로 내부 ID 및 속도를 추정한다.
향후 인지 ID가 시간에 걸쳐 안정적이라고 검증되면 tracker 입력 계약을 확장할 수 있다.

## 출력

```cpp
if (!output.valid) {
  reportContractError(output.reason);
  return;
}

auto local_path = toBaseLinkPath(output.adas.target_path_map, input.adas.ego);
double speed_mps = std::min(output.target_speed_mps,
                            baseline_curvature_speed_cap_mps);
speed_mps = std::min(speed_mps, baseline_next_limit_speed_cap_mps);
publishPathWithSpeed(local_path, speed_mps * 3.6);
```

중요 출력:

- `adas.target_path_map`: 현재 차로 또는 고정된 quintic 차로 변경 경로
- `target_speed_mps`: Region, ACC/AEB, Gap Shaping을 합친 속도
- `adas.lane_change_state`: `KEEP_LANE/CHECK_GAP/EXECUTE/SETTLE`
- `adas.safety_action`: 시작 차단, 조기복귀, 전방 감속, 후방 위협 속도유지 등의 이유
- `region.guard_active`: 합류 실패로 소멸점 정지 프로파일이 활성화됐는지 여부

## Region 판정

판단 코어는 실제 맵의 Region 경계를 알지 않고 enum만 받는다. 제공한 ROS adapter에서는
`highway_region_manager.py`가 `highway_lane1` 누적거리로 이 enum을 생성한다.

```text
HW_ENTRY -> HW_ENTRY_GUARD
         -> HW_MAIN
         -> HW_MAIN_GUARD_1
         -> HW_MAIN_GUARD_2
         -> HW_TOLL
```

Region 경계는 polygon 또는 기준경로의 누적거리 `s` 범위로 만들 수 있다. 판단 코어에는
geometry를 중복 저장하지 않는다. Guard 시작점은 남은 거리로 안정적으로 정지할 수 있을 만큼
앞에 두고, Guard 정지선은 정지 후에도 변경 경로를 만들 수 있게 차로 소멸점보다 45 m 이상
앞에 둔다. 경계와 정지선은 실제 주행 로그로 조정한다.

`HW_TOLL`은 `highway_lane1`의 마지막 링크 `A2256W000444`를 그대로 추종한다. 별도의
2차로 baseline 톨게이트 경로를 섞지 않는다.

현재 HD맵 기반 초기 경계는 `24.0 / 167.4 / 282.0 / 474.5 / 672.0 / 761.5 m`이며
`highway_adas_ros/config/highway_regions.yaml` 한 곳에서 관리한다. 실제 콘 배치와 톨게이트
제한 시작점은 MORAI 주행 로그로 보정해야 한다.

실차 입력을 연결한 전체 고주로 스택은 다음 launch로 실행한다.

```bash
roslaunch highway_adas_ros highway_stack.launch
```

## 기존 baseline과 중복 제거

새 ADAS가 앞차 추종을 담당하므로 baseline `SpeedProfiler`의 `use_following`은 `false`로 둔다.
두 ACC를 동시에 켜면 서로 다른 leader 판정이 중복으로 속도를 제한한다. 곡률 속도와 정적
장애물 정지는 baseline에서 유지해도 된다.

첨부 LQR은 경로점 `pose.position.z`의 km/h 속도를 읽는다. 현재 launch 값이 아래보다 낮으면
판단의 100 km/h 출력이 제어기에서 잘린다.

```text
target_velocity_kph      = 100
general_speed_limit_kph  = 100
max_valid_speed_mps      >= 30
```

LQR에 Region이나 ADAS 내부 상태 문자열을 보낼 필요는 없다. 제어 계약은 목표 경로와 점별
속도로 제한하고, 상태와 reason은 로그/모니터링용으로 별도 발행한다.

ROS adapter는 `/highway/selected_path_with_speed`와 `/highway/active`를 발행한다. baseline
경로와 고주로 경로 중 하나를 고르는 최종 selector만 공용 `/selected_path_with_speed`를
발행해야 하며, 두 planner가 같은 토픽에 동시에 쓰면 안 된다.
