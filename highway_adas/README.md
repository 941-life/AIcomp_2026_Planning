# Highway ADAS

새 baseline에 붙일 수 있도록 ROS, UDP, LQR에 의존하지 않는 C++14 판단 모듈이다.
고주로 Region 정책, 앞차 추종, 객체 추적, 차로 변경 FSM을 한 번에 호출할 수 있다.

## 전체 구조

```text
Region + Ego + 객체 위치/크기 + 차로 centerline
                         |
                         v
             HighwayDecisionModule
              /                 \
   HighwayRegionPolicy        AdasPlanner
                         /        |        \
              ObjectTracker  Following  LaneChange
                         \        |        /
                          target path + target speed
```

- `HighwayRegionPolicy`: 현재 Region의 필수 차로, 차로 요청, 목표속도와 상한을 결정한다.
- `ObjectTracker`: 외부 객체 ID가 없어도 Hungarian association으로 내부 track ID와 속도를 만든다.
- `FollowingController`: 현재 및 변경 중인 차로의 앞차에 대해 ACC/AEB 속도 상한을 만든다.
- `LaneChangePlanner`: Gap 선택, 속도 조절, quintic 경로, 실행 중 재평가를 담당한다.
- `HighwayDecisionModule`: 위 모듈을 합쳐 baseline이 한 번만 호출하게 하는 얇은 인터페이스다.

## Region 정책

| Region | 현재 차로가 잘못된 경우 | 속도 정책 |
|---|---|---|
| `HW_ENTRY` | 4차로이면 3차로 `MANDATORY` | 목표 70, 상한 80 km/h |
| `HW_ENTRY_GUARD` | 4차로이면 3차로 `MANDATORY`, 소멸점 전 정지 가능 | 목표 70, 상한 80 km/h |
| `HW_MAIN` | 3→2→1 순서로 `STRATEGIC` 요청 | 목표/상한 100 km/h |
| `HW_MAIN_GUARD_1` | 3차로이면 2차로 `MANDATORY` | 목표/상한 100 km/h와 정지 상한 중 작은 값 |
| `HW_MAIN_GUARD_2` | 2차로이면 1차로 `MANDATORY` | 목표/상한 100 km/h와 정지 상한 중 작은 값 |
| `HW_TOLL` | 1차로 고정, 새 변경 금지 | 톨게이트 60 km/h 제한에 맞춰 감속 |

`MANDATORY`는 위험한 끼어들기를 허용한다는 뜻이 아니다. Gap이 없으면 현재 차로를 유지하며
속도로 Gap을 만들고, Guard에서는 마지막 합류 가능 지점을 넘지 않도록 감속 또는 정지한다.
이미 필수 차로에 들어갔다면 같은 Region에서도 `KEEP_LANE`을 출력한다. `HW_MAIN`의
`STRATEGIC` 요청도 Gap 안전기준은 동일하며 Guard에 들어가기 전까지 기회를 일찍 찾는 역할만 한다.

Guard 속도 상한은 다음 제동식으로 계산한다.

```text
usable_distance = distance_to_guard_stop
                - 5 m margin
                - ego_speed * 0.5 s response_time

guard_speed_cap = sqrt(2 * 3.0 m/s^2 * usable_distance)
```

`distance_to_guard_stop`의 기준점은 차로의 물리적 끝이 아니다. 정지 후에도 최소 45 m의
차로변경 경로를 만들 수 있도록, 차로 끝보다 충분히 앞에 둔 마지막 안전 변경 시작선이다.
이 값은 Ego가 움직일 때마다 갱신해야 하며 ROS adapter는 0.3초보다 오래된 값을 거부한다.

ROS 검증에서는 `highway_region_manager.py`가 `highway_lane1`에 Ego를 투영해 Region과
남은 거리를 20 Hz odometry 주기로 발행한다. 초기 경계는 다음과 같으며 MORAI 로그로
최종 보정한다.

| Region | 기준경로 s 범위 |
|---|---:|
| `HW_ENTRY` | 0.0 ~ 24.0 m |
| `HW_ENTRY_GUARD` | 24.0 ~ 167.4 m |
| `HW_MAIN` | 167.4 ~ 282.0 m |
| `HW_MAIN_GUARD_1` | 282.0 ~ 474.5 m |
| `HW_MAIN_GUARD_2` | 474.5 ~ 672.0 m |
| `HW_TOLL` | 672.0 ~ 761.5 m |

Guard 정지선은 각각 122.0 m, 429.0 m, 605.0 m다. 소멸 또는 분기 시작점보다 약 45 m
앞에 있어 정지 후에도 최소 변경 경로를 확보한다.

## 차로 변경 FSM

```text
KEEP_LANE -> CHECK_GAP -> EXECUTE -> SETTLE -> KEEP_LANE
```

### KEEP_LANE

현재 차로 centerline을 출력한다. 요청은 존재하는 인접 차로 한 칸만 허용하며, 두 차로 점프나
알 수 없는 차로 ID는 명시적으로 거부한다.

### CHECK_GAP

현재 차로를 유지하면서 목표 차로의 Front/Rear 한 쌍을 `SelectedGap`으로 고른다. Gap과
예측값은 매 주기 갱신한다. 같은 차량 쌍은 최대한 유지하고, 속도로 만들 수 없는 상태가
1.5초 지속되거나 즉시 위험해지면 다른 Gap을 평가한다.

모든 내부 계산 단위는 m, m/s, s, m/s^2다. km/h는 설정 표시와 ROS adapter 경계에서만 쓴다.

```text
front_closing = max(ego_speed - front_speed, 0)
rear_closing  = max(rear_speed - ego_speed, 0)

required_front_gap = max(6, 0.6 * ego_speed,
                         5 + 1.2 * front_closing)
required_rear_gap  = max(6, 0.6 * rear_speed,
                         5 + 1.5 * rear_closing)
```

차로 변경 승인은 다음을 모두 만족한 상태가 0.5초 이어질 때만 발생한다.

```text
front/rear gap >= required gap
front/rear TTC >= 4.0 s
front/rear required deceleration <= 2.5 m/s^2
남은 변경시간 뒤 predicted gap >= 6 m
candidate path corridor clear
perception age <= 0.3 s
|lateral error| <= 0.35 m
|heading error| <= 4 deg
|yaw rate| <= 6 deg/s
```

### Gap Shaping

경로 길이와 실제 속도로 예상 변경시간을 구한다.

```text
L_lc = clamp(ego_speed * 3.0 s, 45 m, 85 m)
T_lc = L_lc / max(ego_speed, 1.0 m/s)

slot_speed_min = rear_speed
               + (required_rear_gap - current_rear_gap) / T_lc
slot_speed_max = front_speed
               + (current_front_gap - required_front_gap) / T_lc
```

도달 가능한 범위는 감속 2.0 m/s^2, 가속 1.0 m/s^2로 제한한다. Gap 확보만을 위해 목표
차로 흐름 속도의 70% 아래로 감속하지 않는다. 이 하한은 ACC, AEB, Guard 정지에는 적용하지
않는다. Front가 없으면 상한은 무한대, Rear가 없으면 하한은 0이다.

### EXECUTE

시작 순간 만든 quintic 경로를 고정해 LQR에 전달한다.

```text
w(u) = 10u^3 - 15u^4 + 6u^5
path = source + w(u) * (target - source)
```

경로는 고정하지만 다음 항목은 매 주기 다시 계산한다.

- 현재 경로 진행률 `u`
- 목표 차로 Front/Rear Gap, TTC, 필요감속량
- 남은 변경시간 뒤 예측 Gap
- 변경 경로 corridor
- 조기 복귀가 필요할 때 원래 차로 Front/Rear와 복귀 corridor

`u < 0.35`, 차체가 차선을 넘지 않음, 목표 차로가 위험, 원래 차로와 복귀 경로가 안전한
경우에만 원래 차로로 부드러운 복귀 경로를 새로 만든다. 하나라도 불만족하면 급복귀하지 않는다.

변경이 이미 진행된 뒤에는 위험 종류에 따라 다르게 처리한다.

- 목표 차로 Front 위험: 두 차로 ACC 중 더 낮은 속도를 사용하며 변경을 계속한다.
- 목표 차로 Rear 위험: 감속으로 충돌을 키우지 않고 현재 속도를 유지하며 변경을 계속한다.
- 인지 지연: 새 경로를 만들거나 급감속하지 않고 현재 경로와 현재 속도를 유지한다.

### SETTLE

경로 진행률 0.85 이상에서 목표 차로 횡오차 0.25 m, 방향오차 3도 이내에 들어오면 시작한다.
그 상태가 0.5초 유지되면 변경을 완료한다.

## 앞차 추종

현재 차로 centerline corridor의 가장 가까운 전방 track을 선행차로 선택한다. 변경 중에는
source와 target 차로를 모두 평가하고 더 낮은 속도 상한을 사용한다.

```text
desired_gap    = 5 m + 1.8 s * ego_speed
TTC            = gap / max(ego_speed - lead_speed, 0)
required_decel = closing_speed^2 / (2 * (gap - 5 m))
```

- `FOLLOW`: 목표간격 주변에서 앞차 속도에 맞춘다.
- `HARD_BRAKE`: TTC 3.0초 이하 또는 필요감속 4.0 m/s^2 이상이다.
- `EMERGENCY_BRAKE`: 간격 5 m 이하, TTC 1.5초 이하 또는 필요감속 6.0 m/s^2 이상이다.

일반 추종에는 3회 확인된 track만 쓰고, 미확정 객체는 즉시 충돌 위험일 때 AEB에만 쓴다.

## 최종 출력

```text
HighwayDecisionOutput.target_speed_mps
  = min(region_speed_cap,
        following/gap/stale-perception speed cap,
        curvature speed cap)
```

곡률 제한은 코어가 최종 목표 경로를 기준으로 계산하며, 다음 제한구간 감속은
Region policy가 합성한다. ROS adapter는 `target_speed_mps`를 다시 제한하지 않는다.

`target_path_map`은 LQR 입력 경로로, 최종 속도는 경로점의 속도 값으로 변환한다. 자세한 연결
계약은 `INTEGRATION.md`에 있다.

## 의도적으로 둔 안전 동작

1. 인지가 0.3초보다 오래되면 새 차로 변경을 시작하지 않는다.
2. 차로 변경 전 인지 지연은 최고 60 km/h로 제한한다.
3. 변경 중 인지 지연은 급감속 대신 현재 경로와 현재 속도를 유지한다.
4. Guard에서 Gap을 끝내 얻지 못하면 사라지는 차로 끝 전에 정지할 수 있다.

그 외 잘못된 차로, 경로, 요청은 임의 fallback으로 숨기지 않고 `valid=false` 또는
`request_rejected=true`로 알린다.

## 빌드와 테스트

```bash
cmake -S highway_adas -B highway_adas/build
cmake --build highway_adas/build
ctest --test-dir highway_adas/build --output-on-failure
```
