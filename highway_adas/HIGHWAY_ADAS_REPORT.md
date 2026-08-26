# 고주로 판단 구현 보고서

## 1. 구현 목표

고주로 판단은 다음 세 책임을 분리한다.

1. `Region`: 지금 위치에서 어느 차로가 필수이고 속도 상한이 얼마인지 결정한다.
2. `Lane Change FSM`: 실제로 변경을 시작해도 되는지 판단하고 LQR용 경로를 만든다.
3. `Longitudinal ADAS`: 앞차 추종, 위험 감속, Gap 확보용 속도를 계산한다.

이 모듈은 ROS 메시지나 LQR 구현을 직접 알지 않는다. baseline은 지도와 인지 값을 구조체로
변환하고, 결과 경로와 속도를 기존 메시지로 바꾸는 역할만 한다.

## 2. 고주로 Region

### 2.1 Region 구분

```text
HW_ENTRY
  4차로 -> 3차로 합류를 일찍 시도

HW_ENTRY_GUARD
  아직 4차로이면 합류를 계속 시도
  끝까지 실패하면 소멸점 전에 정지

HW_MAIN
  3차로 -> 2차로 -> 1차로를 순서대로 일찍 시도

HW_MAIN_GUARD_1
  아직 3차로이면 2차로 합류를 보장

HW_MAIN_GUARD_2
  아직 2차로이면 1차로 합류를 보장
  끝까지 실패하면 소멸점 전에 정지

HW_TOLL
  단일 1차로와 고정된 톨게이트 통로 유지
  다음 60 km/h 제한에 맞춰 감속
```

### 2.2 속도와 차로 요청

| Region | 필수 차로 요청 | 목표 | 상한 |
|---|---|---:|---:|
| `HW_ENTRY` | 4차로이면 3차로 `MANDATORY` | 70 km/h | 80 km/h |
| `HW_ENTRY_GUARD` | 4차로이면 3차로 `MANDATORY` | 70 km/h | 80 km/h와 정지 상한 중 작은 값 |
| `HW_MAIN` | 3→2→1 `STRATEGIC` | 100 km/h | 100 km/h |
| `HW_MAIN_GUARD_1` | 3차로이면 2차로 `MANDATORY` | 100 km/h | 100 km/h와 정지 상한 중 작은 값 |
| `HW_MAIN_GUARD_2` | 2차로이면 1차로 `MANDATORY` | 100 km/h | 100 km/h와 정지 상한 중 작은 값 |
| `HW_TOLL` | 현재 1차로 유지 | 100 km/h | 다음 60 km/h 감속 상한 |

`HW_TOLL` 진입 시 현재 차로가 1차로가 아니면 앞선 GUARD 실패이므로 임의로 계속
주행하지 않고 입력 계약 오류로 반환한다.

### 2.3 MANDATORY의 의미

`MANDATORY`는 해당 구간이 끝나기 전에 반드시 목표 차로에 있어야 한다는 상위 의도다.
안전 기준을 완화하는 명령은 아니다.

- Gap이 안전하면 차로 변경한다.
- Gap이 부족하면 현재 차로에서 속도를 조절하며 기다린다.
- GUARD에서도 Gap이 없으면 억지로 들어가지 않는다.
- 마지막에는 소멸점 앞에서 정지할 수 있게 속도를 낮춘다.

이 구조라서 Entry 초반에 4→3을 실패하거나 Main 초반에 3→2 또는 2→1을 실패해도
각 Guard에서 다시 대응할 수 있다.

Guard 정지선은 차로의 물리적 끝이 아니다. 정지한 뒤에도 최소 45 m의 차로변경 경로를
만들 수 있도록 소멸점보다 앞에 둔다.

ROS 실험용 Region manager의 초기 누적거리 경계는 다음과 같다.

```text
0.0   HW_ENTRY
24.0  HW_ENTRY_GUARD       stop s=122.0
167.4 HW_MAIN
282.0 HW_MAIN_GUARD_1      stop s=429.0
474.5 HW_MAIN_GUARD_2      stop s=605.0
672.0 HW_TOLL              60 km/h point s=761.0
761.5 NONE
```

이는 HD맵 centerline으로 만든 초기값이다. 실제 콘 배치와 판정 시점은 MORAI 로그를 보고
YAML 값만 조정한다.

## 3. 입력 데이터와 내부 추적

필수 인지 입력은 객체 중심 위치와 폭/길이다. 외부 객체 ID와 속도는 요구하지 않는다.

```text
ObjectDetection
  x, y            base_link, m
  width, length   m
```

객체 timestamp의 Ego pose로 map 좌표로 변환한 뒤 Hungarian assignment를 수행한다.

```text
association cost = predicted position distance
                 + 0.5 * size difference
```

거리 gate 밖의 조합은 매칭하지 않고 dummy row/column을 두어 새 객체와 사라진 객체를
명확히 처리한다. 연결된 위치의 시간차로 속도를 추정한다. 일반 판단에는 3회 확인된 track을
사용하고, 새 객체는 즉시 충돌 위험일 때 AEB에만 사용할 수 있다.

내부 `track_id`는 물리적 차량의 영구 식별자가 아니다. 짧은 시간 동안 같은 객체의 위치와
속도를 잇기 위한 판단 내부 키다. Selected Gap은 이 ID로 Front/Rear 차량 쌍을 기억한다.

## 4. 차로 변경 FSM

```text
KEEP_LANE -> CHECK_GAP -> EXECUTE -> SETTLE -> KEEP_LANE
```

### 4.1 KEEP_LANE

- 현재 차로의 전방 centerline을 출력한다.
- 요청된 목표 차로가 실제로 존재하고 인접한지 확인한다.
- 두 차로 점프와 잘못된 ID는 `request_rejected=true`로 거부한다.

### 4.2 CHECK_GAP

- 횡이동을 시작하지 않고 현재 차로를 유지한다.
- 목표 차로 차량을 종방향으로 정렬한다.
- Ego가 들어갈 Front/Rear 차량 쌍을 `SelectedGap`으로 선택한다.
- Gap, TTC, 필요감속, corridor와 목표속도를 매 주기 다시 계산한다.
- 상위 요청이 철회되면 바로 `KEEP_LANE`으로 돌아간다.
- 모든 시작 조건이 0.5초 연속 안전할 때 `EXECUTE`로 간다.

시작 자세 조건은 다음과 같다.

```text
현재 차로 중심 횡오차 <= 0.35 m
현재 차로 방향오차   <= 4 deg
Ego yaw-rate          <= 6 deg/s
```

### 4.3 EXECUTE

- 시작 순간의 차로 변경 경로를 고정한다.
- 상위 Region 요청이 바뀌어도 진행 중 경로를 바로 바꾸지 않는다.
- source/target 앞차 추종 결과 중 더 낮은 속도를 사용한다.
- 목표 차로 Gap과 고정 경로 corridor는 매 주기 다시 평가한다.
- 예측시간은 고정 3초가 아니라 남은 변경시간을 사용한다.

### 4.4 SETTLE

- 목표 차로 centerline을 출력한다.
- 변경 진행률 0.85 이상이어야 한다.
- 횡오차 0.25 m, 방향오차 3도 이내가 0.5초 유지되면 완료한다.

## 5. Gap 판단

### 5.1 상대속도 기반 필요거리

```text
front_closing = max(ego_speed - front_speed, 0)
rear_closing  = max(rear_speed - ego_speed, 0)

required_front_gap = max(
    6.0,
    0.6 * ego_speed,
    5.0 + 1.2 * front_closing)

required_rear_gap = max(
    6.0,
    0.6 * rear_speed,
    5.0 + 1.5 * rear_closing)
```

100 km/h 동일 속도 흐름에서는 전후방 각각 약 16.7 m가 필요하다. Ego 길이를 5 m로 보면
약 38.4 m Slot에서 변경할 수 있다. 과거 절대속도 기반 약 90 m 요구와 달리 정속 교통에서
불필요하게 영원히 기다리지 않는다.

### 5.2 안전 gate

```text
front/rear actual gap >= required gap
front/rear TTC >= 4.0 s
front/rear required deceleration <= 2.5 m/s^2
predicted front/rear gap >= 6.0 m
candidate path corridor clear
perception age <= 0.3 s
```

TTC와 필요감속 gate가 있으므로 거리는 멀지만 빠르게 닫히는 차량도 차단한다.

### 5.3 Path corridor

차로 변경 경로와 Ego 속도로 미래 Ego 위치를 만들고, tracker 속도로 미래 객체 위치를 만든다.
0.25초 간격으로 남은 변경시간까지 검사한다.

```text
predicted_ego_s = current_path_s + ego_speed * t
predicted_object = object_position + object_velocity * t
```

Ego와 객체의 폭에 좌우 0.5 m margin을 더한다. 예측 직사각형의 종방향과 횡방향 범위가
동시에 겹치면 corridor가 점유된 것으로 판단한다.

## 6. Selected Gap과 Gap Shaping

목표 차로에서 한 차량만 보는 것이 아니라 Ego 앞의 Front와 뒤의 Rear를 한 쌍으로 고른다.
작은 관측 변화로 목표가 흔들리지 않도록 선택한 쌍을 유지한다.

- 같은 쌍이 추적되고 있으면 우선 유지한다.
- 속도로 실현할 수 없는 상태가 1.5초 이어지면 다른 쌍을 평가한다.
- TTC 2.5초 미만, 필요감속 4.0 m/s^2 초과, corridor 충돌은 즉시 위험으로 처리한다.
- Slot 고정 중에도 위치, 속도, Gap과 예측값은 매 주기 갱신한다.

차로 변경 경로와 예상 시간은 다음과 같다.

```text
nominal duration = 3.0 s
L_lc = clamp(ego_speed * 3.0, 45 m, 85 m)
T_lc = L_lc / max(ego_speed, 1.0 m/s)
```

Gap에 들어가기 위한 속도 범위:

```text
slot_speed_min = rear_speed
               + (required_rear_gap - current_rear_gap) / T_lc

slot_speed_max = front_speed
               + (current_front_gap - required_front_gap) / T_lc
```

- Front가 없으면 `slot_speed_max=INF`다.
- Rear가 없으면 `slot_speed_min=0`이다.
- 감속 2.0 m/s^2, 가속 1.0 m/s^2 안에서 도달 가능한지 검사한다.
- 목표 차로 차량 속도의 중앙값을 flow speed로 사용한다.
- Gap 확보만을 위한 감속은 flow speed의 70%를 하한으로 한다.
- 하한 아래가 필요하면 해당 Gap을 포기하고 다른 Gap을 찾는다.

70% 하한은 Gap Shaping에만 적용한다. ACC, AEB, GUARD 정지는 항상 우선한다.

## 7. 차로 변경 경로

source와 target centerline의 같은 전방 진행 위치를 quintic smooth-step으로 보간한다.

```text
w(u) = 10u^3 - 15u^4 + 6u^5
path = source + w(u) * (target - source)

변경 길이 = clamp(ego_speed * 3.0, 45 m, 85 m)
출력 horizon = 120 m
경로 간격 = 1 m
```

시작과 끝에서 횡방향 변화율이 0으로 연결되어 LQR에 계단형 횡목표를 주지 않는다.

## 8. EXECUTE 중 상황 변화

### 8.1 매 주기 갱신 항목

`Predicted Gap unsafe`를 실제로 판단할 수 있도록 EXECUTE에서도 다음을 다시 계산한다.

```text
진행률 u
목표 차로 Front/Rear Gap
TTC와 필요감속
남은 변경시간 뒤 predicted gap
고정 경로 corridor
필요 시 source Front/Rear와 복귀 corridor
```

### 8.2 조기 취소

아래 조건을 모두 만족해야만 조기 복귀한다.

```text
u < 0.35
차체가 source 차선 경계를 넘지 않음
목표 차로가 unsafe
source Front/Rear가 safe
복귀 corridor clear
```

복귀는 source centerline으로 순간 전환하지 않는다. 현재 고정 경로에서 source centerline으로
새 quintic 복귀 경로를 만들고, 그 경로를 완료할 때까지 `EXECUTE`를 유지한다.

### 8.3 이미 진행된 이후

| 위험 | 종방향 | 횡방향 |
|---|---|---|
| 목표 차로 Front 위험 | source/target ACC 중 더 낮은 속도 | 고정 경로 계속 |
| 목표 차로 Rear 빠른 접근 | 현재 속도 유지, 추가 감속 금지 | 고정 경로 계속 |
| 인지 지연 | 현재 속도 이상 가속 금지, 급감속 금지 | 고정 경로 계속 |
| source 복귀가 unsafe | 복귀하지 않음 | 고정 경로 계속 |

전방 위험과 후방 위험을 같은 60 km/h fallback으로 처리하지 않는다. 후방차가 접근할 때
감속하면 후방 TTC를 더 줄일 수 있기 때문이다.

## 9. 앞차 추종

현재 경로 corridor의 가장 가까운 전방 confirmed track을 leader로 선택한다.

```text
desired_gap = 5.0 + 1.8 * ego_speed
TTC = gap / max(ego_speed - lead_speed, 0)
required_decel = closing_speed^2 / (2 * (gap - 5.0))
```

| 상태 | 조건 |
|---|---|
| `CRUISE` | leader 없음 또는 충분한 거리 |
| `FOLLOW` | 목표간격 주변 또는 느린 leader |
| `HARD_BRAKE` | TTC <= 3.0 s 또는 필요감속 >= 4.0 m/s^2 |
| `EMERGENCY_BRAKE` | gap <= 5 m, TTC <= 1.5 s 또는 필요감속 >= 6.0 m/s^2 |

차로 변경 중에는 source와 target 모두의 leader를 평가한다. 일반 차로 변경 요청과 추월 요청은
같은 안전 FSM을 사용하며, 추월 여부를 결정하는 상위 행동 정책은 별도 책임이다.

## 10. 최종 속도

모듈 내부:

```text
ADAS speed = min(ACC/AEB speed, Gap Shaping target)
highway target = min(Region cap, ADAS speed)
```

baseline adapter:

```text
final speed = min(highway target,
                  curvature cap,
                  baseline next-limit cap)
```

Region 설정값도 코드 내부에서는 m/s로 보관한다. UI와 로그에서만 km/h로 표시해 단위 혼동을
막는다.

## 11. 명시적인 안전 동작

과도한 fallback을 두지 않고 다음만 유지했다.

1. 인지 age가 0.3초를 넘으면 새 차로 변경을 시작하지 않는다.
2. 변경 전 인지 지연에서는 60 km/h 상한으로 새 고속 가속을 막는다.
3. 변경 중 인지 지연에서는 현재 고정 경로와 현재 속도를 유지한다.
4. 초기 목표 차로 위험은 source 복귀까지 안전할 때만 취소한다.
5. GUARD에서 합류가 끝내 불가능하면 차로 소멸점 앞에서 정지한다.

잘못된 차로 ID, 비인접 요청, 사라진 경로는 임의의 경로로 덮지 않고 오류로 반환한다.

## 12. 검증 항목

자동 테스트에는 다음 상황이 포함된다.

- 검출 순서가 바뀌어도 내부 track ID 유지
- 동일속도 100 km/h 흐름에서 현실적인 상대속도 Gap 승인
- 빠른 후방차에 대해 제한적 가속으로 Gap 확보
- CHECK_GAP 중 요청 철회
- source 후방이 안전할 때만 조기 취소
- 진행률 0.35 이후 급복귀 금지
- 변경 중 후방 위협에 감속 명령 금지
- Entry/Main ACQUIRE와 GUARD 차로 명령 및 정지 상한
- Single Lane의 다음 60 km/h 제한 감속
- 두 차로 점프 거부와 stale perception 처리
- `HighwayDecisionModule`에서 Region 요청과 Lane Change FSM 연결

실차 적용 전에는 맵 centerline, 차로 폭, Region 경계, GUARD 정지거리, LQR의 100 km/h 추종을
로그 기반으로 별도 튜닝해야 한다.
