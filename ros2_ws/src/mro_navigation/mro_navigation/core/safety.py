"""I-049: 제한된 탈출 회전과 속도 출력의 단일 선택자."""
import math


class SafetyController:
    """시각·센서를 호출자가 공급하는 안전 상태기계."""

    def __init__(self, stop=0.30, clear=0.38, yaw=0.25, rotate_s=7.0,
                 scan_valid_s=0.5, pause_s=2.0, sweep_clear_m=0.21, max_angle_deg=90.0, odom_valid_s=0.5, stopped_linear=0.01, stopped_angular=0.02, stopped_s=0.3):
        values = (stop, clear, yaw, rotate_s, scan_valid_s, pause_s, sweep_clear_m, max_angle_deg, odom_valid_s, stopped_linear, stopped_angular, stopped_s)
        if not all(math.isfinite(v) and v > 0 for v in values):
            raise ValueError('안전 파라미터는 유한한 양수여야 한다')
        # **각도 한도가 시간 한도보다 먼저 걸려야 한다** (PM 9/9). 반각 35° 부채꼴에서 정면 벽
        #   d 를 풀려면 d / cos(θ − 35°) > clear_dist 가 되는 θ 까지 돌아야 한다 — 0.30 m 면
        #   72.9°, 실주행 실측 0.29 m 면 75.3°. 종전 0.25 rad/s × 5 s = 71.6° 는 그 어느 것도
        #   못 풀고, 각도 상한 75° 는 5 s 가 항상 먼저 걸려 죽은 값이었다. 90° / 7 s 면
        #   0.25 × 7 = 100° > 90° 라 각도가 먼저 걸리고, 0.22 m 벽까지 덮는다
        if clear <= stop or yaw > 0.3 or rotate_s > 10.0 or max_angle_deg > 90:
            raise ValueError('clear > stop, 회전 ≤ 0.3 rad/s·10 s·90°가 필요하다')
        if yaw * rotate_s <= math.radians(max_angle_deg):
            raise ValueError('시간 한도가 각도 한도보다 먼저 걸린다 — 각도 상한이 죽은 값이 된다 (PM 9/9)')
        self.stop, self.clear_dist, self.yaw = stop, clear, yaw
        self.rotate_s, self.scan_valid_s, self.pause_s = rotate_s, scan_valid_s, pause_s
        self.sweep_clear_m = sweep_clear_m
        self.max_angle = math.radians(max_angle_deg)
        self.odom_valid_s = odom_valid_s
        self.odom_at = self.odom_yaw = None
        self.stopped_linear, self.stopped_angular = stopped_linear, stopped_angular
        self.stopped_s, self.still_since = stopped_s, None
        self.braking = False
        self.rotated = 0.0
        self.state = 'STOPPING'
        self.reason = '초기 스캔·주행 정지 확인 대기'
        self.since = None
        self.scan_at = None
        self.front = None
        self.sweep = None
        self.direction = 1.0
        self.hit = self.clear = 0
        self.epoch = 0
        self.obstacle = False
        # **도크에 물려 있는 동안은 전방 판정을 하지 않는다.**
        #   Create3 는 도크에 코를 박고 들어간다 — 도킹 상태에서 전방 ±35° 안에 있는 것은
        #   도크와 그 뒤 벽이고, stop_dist 0.30 m 안에 반드시 들어온다. 그것을 장애물로
        #   세면 **estop 이 항상 걸려 언독 자체가 불가능해진다** (9/10 실주행에서 막혔다).
        #   게다가 언독은 그 장애물에서 **멀어지는** 후진 동작이라 전방 판정으로 막을 이유가
        #   없다. 도크는 알려진 정적 구조물이지 위험물이 아니다.
        #   도크를 벗어나는 순간 is_docked 가 거짓이 되어 판정이 정상으로 돌아온다.
        self.docked = False

    @property
    def estop(self):
        return self.state != 'CLEAR'

    def transition(self, state, now, reason):
        self.state, self.since, self.reason = state, now, reason
        self.still_since = None
        if state == 'STOPPING':
            self.braking = False
        if state == 'CLEAR':
            self.epoch += 1

    def dock_state(self, docked):
        """도킹 여부. 참인 동안 전방 판정을 보류한다 (도크가 곧 전방 장애물이다)."""
        was, self.docked = self.docked, bool(docked)
        if was and not self.docked:
            self.hit = self.clear = 0   # 도크를 벗어났다 — 판정을 새로 쌓는다

    def scan(self, now, front, sweep, left, right):
        self.scan_at, self.front, self.sweep = now, front, sweep
        if self.docked:
            # 도크가 전방에 있는 것이 정상이다. 막혔다고 세지 않고, 해제도 세지 않는다
            self.hit = 0
            self.clear = 5
            return
        if front is None:
            self.hit = self.clear = 0
            return
        self.hit = self.hit + 1 if front < self.stop else 0
        self.clear = self.clear + 1 if front > self.clear_dist else 0
        if self.state == 'CLEAR' and self.hit >= 2:
            self.obstacle = True
            self.direction = 1.0 if left >= right else -1.0
            self.transition('STOPPING', now, '전방 장애물')

    def invalidate_odometry(self):
        self.odom_at = self.still_since = None

    def odometry(self, now, yaw, linear=None, angular=None):
        if not math.isfinite(yaw):
            self.invalidate_odometry()
            return
        # 시료가 끊기거나 속도가 없으면 연속 정지를 입증할 수 없다.
        continuous = self.odom_at is not None and 0 <= now - self.odom_at <= self.odom_valid_s
        stationary = (linear is not None and angular is not None
                      and math.isfinite(linear) and math.isfinite(angular)
                      and abs(linear) <= self.stopped_linear
                      and abs(angular) <= self.stopped_angular)
        if not stationary:
            self.still_since = None
        elif self.still_since is None or not continuous:
            self.still_since = now
        if self.state == 'SAFE_ROTATE' and self.odom_yaw is not None:
            # ±π 경계와 방향 반전을 포함한 실측 회전량. 왕복해도 상쇄하지 않는다.
            delta = math.atan2(math.sin(yaw - self.odom_yaw), math.cos(yaw - self.odom_yaw))
            self.rotated += abs(delta)
        self.odom_at, self.odom_yaw = now, yaw

    def stopped(self, now):
        return (self.odom_at is not None and 0 <= now - self.odom_at <= self.odom_valid_s
                and self.still_since is not None
                and now - self.still_since >= self.stopped_s)

    def tick(self, now, paused):
        if self.since is None:
            self.since = now
        fresh = self.scan_at is not None and 0 <= now - self.scan_at <= self.scan_valid_s
        if not fresh or (self.front is None and not self.docked):
            if self.state != 'STUCK':
                self.transition('STUCK', now, '유효한 LiDAR 스캔 없음')
            self.clear = 0
            return 0.0
        if self.state == 'CLEAR':
            return None
        if self.clear >= 5 and paused:
            if not self.braking:
                # 0 출력 이후부터 새 정지 시료를 쌓는다. ESTOP은 계속 유지한다.
                self.braking = True
                self.still_since = None
            if self.stopped(now):
                self.obstacle = False
                self.transition('CLEAR', now, '전방 해제 거리 확보·실제 정지 확인')
            return 0.0
        if self.braking:
            # 제동 중 다시 막히면 재회전하지 않는다.
            self.transition('STUCK', now, '해제 제동 중 장애물 또는 정지 ACK 소실')
            return 0.0
        if self.state == 'STUCK':
            return 0.0
        if not paused:
            if self.state == 'SAFE_ROTATE' or now - self.since >= self.pause_s:
                self.transition('STUCK', now, '주행 액션 정지 확인 실패')
            return 0.0
        if self.state == 'STOPPING':
            # 최초 기동에는 로봇을 돌리지 않는다. 실제 장애물 진입에만 한 번 탈출한다.
            if not self.obstacle:
                return 0.0
            if self.sweep is None or self.sweep < self.sweep_clear_m:
                self.transition('STUCK', now, '회전 영역 여유 부족')
                return 0.0
            if not self.stopped(now):
                if now - self.since >= self.pause_s:
                    self.transition('STUCK', now, '실제 정지 확인 시간 초과')
                return 0.0
            self.rotated = 0.0
            self.transition('SAFE_ROTATE', now, '워치독 탈출 회전')
        if self.state == 'SAFE_ROTATE':
            if self.odom_at is None or not 0 <= now - self.odom_at <= self.odom_valid_s:
                self.transition('STUCK', now, '유효한 odometry 없음 — 회전량 확인 불가')
                return 0.0
            if (now - self.since >= self.rotate_s or self.rotated >= self.max_angle
                    or self.sweep is None
                    or self.sweep < self.sweep_clear_m):
                self.transition('STUCK', now, '탈출 시간·각도 한도 도달 또는 회전 영역 여유 부족')
                return 0.0
            return self.direction * self.yaw
        return 0.0


# 구역을 판정하려면 빔의 이만큼은 유효해야 한다. 실측 유효율이 74~81 % 라 절반이면 여유가 있고,
#   그 아래는 센서가 가려졌거나 고장난 것이라 '모름' 으로 두는 것이 맞다
MIN_VALID_FRACTION = 0.5


def scan_sectors(scan):
    """전방·전체 회전 영역·좌우 거리. 결측 스캔으로 회전을 허용하지 않는다."""
    sectors = {'front': [], 'left': [], 'right': []}
    all_ranges = []
    for i, value in enumerate(scan.ranges):
        angle = math.atan2(math.sin(scan.angle_min + i * scan.angle_increment),
                           math.cos(scan.angle_min + i * scan.angle_increment))
        # +inf는 해당 방향에 반사체가 없는 유효 스캔이다. NaN/0은 결측이다.
        valid = value == math.inf or (math.isfinite(value)
                                     and scan.range_min < value <= scan.range_max)
        distance = value if valid else None
        all_ranges.append(distance)
        key = ('front' if abs(angle) <= math.radians(35)
               else 'left' if angle > 0 else 'right')
        sectors[key].append(distance)
    def minimum(items):
        # **결측 빔은 버리고 남은 빔으로 판정한다.** 종전에는 빔 하나라도 결측이면 구역 전체를
        #   None(모름)으로 쳤는데, RPLIDAR A1 은 반사가 약한 방향·너무 가까운 방향에 0 을 돌려
        #   보내므로 **정상 스캔에서도 전방 140 빔 중 26~36 개가 늘 결측이다** (9/10 실주행
        #   `full_0910_0838`, 유효 104~114/140). 그 규칙으로는 front 가 언제나 None 이라
        #   언독하는 순간 STUCK('유효한 LiDAR 스캔 없음')이 걸려 로봇이 영영 굳는다.
        #   구역을 '모름' 으로 치는 것은 유효 빔이 절반도 안 될 때뿐이다 — 그건 센서 이상이다.
        valid = [v for v in items if v is not None]
        if not items or len(valid) < max(1, math.ceil(len(items) * MIN_VALID_FRACTION)):
            return None
        return min(valid)
    front = minimum(sectors['front'])
    # 360도 범위를 보지 못하면 회전을 허용하지 않는다.
    coverage = abs(scan.angle_increment) * len(scan.ranges)
    sweep = minimum(all_ranges) if coverage >= math.radians(350) else None
    left = minimum(sectors['left'])
    right = minimum(sectors['right'])
    return front, sweep, left or 0.0, right or 0.0
