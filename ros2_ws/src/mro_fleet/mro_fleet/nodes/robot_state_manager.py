"""로봇별 RobotStatus 객체를 생성하고 관리한다."""
import time

from mro_fleet.nodes.robot_status import RobotStatus, STATUS_SOURCES

from mro_interfaces.msg import Mission


RETURN_AND_DOCK_STATES = {
    'RETURNING', 'DOCK_WAIT', 'DOCKING', 'DOCKED', 'FAILED'
}
APPROACH_MISSION_STATES = {
    Mission.PAIRED, Mission.FOLLOWING, Mission.STANDBY
}


def source_freshness(received_time, publishers, current_time):
    """소스 하나가 아직 믿을 만한지 판정한다 (I-055).

    기준은 나이가 아니라 **소스가 살아 있는지**다. `safety/estop` 은 바뀔 때만,
    `amcl_pose` 는 로봇이 움직일 때만 발행하므로, 나이로 재면 멀쩡한 로봇이
    계속 「확인 불가」가 된다. 발행자가 사라졌을 때만 마지막 값이 현재값이
    아니게 된다.

    ``publishers`` 가 ``None`` 이면 아직 세기 전이다 — 한 번도 못 받았을 때만
    모른다고 한다. 세는 타이머가 처음 돌기 전 잠깐이다.
    """
    age = None if received_time is None else round(
        max(0.0, current_time - received_time), 3
    )
    if received_time is None:
        stale = True
    elif publishers is None:
        stale = False
    else:
        stale = publishers == 0
    return {'age_s': age, 'publishers': publishers, 'stale': stale}


class RobotStateManager:
    """관제가 다룰 로봇 객체를 namespace별로 관리한다."""

    def __init__(self, namespaces, executor):
        """namespace마다 RobotStatus 객체를 하나씩 생성한다."""
        self.executor = executor
        # 관제가 두 벌 떠 있을 때 화면에 띄울 경고 (I-054). FleetManager 가 채운다.
        self.duplicate_monitor_warning = ''
        self.robots = {}
        self.last_received_status = {}
        for namespace in namespaces:
            self.add_robot(namespace)

    def add_robot(self, namespace):
        """로봇 객체를 생성하고 ROS executor에 추가한다."""
        if namespace in self.robots:
            return False
        robot = RobotStatus(namespace)
        self.executor.add_node(robot)
        self.robots[namespace] = robot
        self.last_received_status[namespace] = {
            'approach_state': 'UNKNOWN',
            'shuttle_state': 'UNKNOWN',
            'battery': None,
            'estop': None,
            'paired_worker': '',
        }
        return True

    def remove_robot(self, namespace):
        """로봇 객체를 ROS executor에서 빼고 종료한다."""
        robot = self.robots.get(namespace)
        if robot is None:
            return False
        self.executor.remove_node(robot)
        robot.destroy_node()
        del self.robots[namespace]
        del self.last_received_status[namespace]
        return True

    def get_robot(self, namespace):
        """namespace에 해당하는 로봇 객체를 반환한다."""
        return self.robots.get(namespace)

    def get_all_robot_status(self):
        """모든 로봇의 상태와 수신 지연 여부를 반환한다."""
        current_time = time.monotonic()
        all_status = {}
        for namespace, robot in self.robots.items():
            status = robot.get_status()
            received_time = status.pop('_received_monotonic', None)
            source_times = status.pop('_source_monotonic', {})
            source_publishers = status.pop('_source_publishers', {})
            if received_time is None:
                status['age_s'] = None
                status['stale'] = True
            else:
                age = max(0.0, current_time - received_time)
                status['age_s'] = round(age, 3)
                status['stale'] = age > robot.status_overtime_sec
            # 칸마다 따로 판정한다 — 배터리 하나가 살아 있다고 위치·E-STOP 까지
            #   신선하다고 말하면 화면이 「모른다」를 말할 수 없다 (I-055).
            status['sources'] = {
                source: source_freshness(
                    source_times.get(source),
                    source_publishers.get(source),
                    current_time,
                )
                for source in STATUS_SOURCES
            }
            all_status[namespace] = status
        return all_status

    def update_fleet_status(self, message):
        """FleetStatus를 로봇 객체에 반영하고 변경된 값을 반환한다."""
        robot = self.robots[message.robot_ns]
        previous = self.last_received_status[message.robot_ns]
        current = {
            'approach_state': message.approach_state,
            'shuttle_state': message.shuttle_state,
            'battery': message.battery,
            'estop': message.estop,
            'paired_worker': message.paired_worker,
        }
        changes = {}
        for key, value in current.items():
            if previous[key] != value:
                changes[key] = (previous[key], value)

        with robot.status_lock:
            robot.status['approach_state'] = message.approach_state
            robot.status['shuttle_state'] = message.shuttle_state
            robot.status['battery'] = message.battery
            robot.status['x'] = message.x
            robot.status['y'] = message.y
            robot.status['yaw'] = message.yaw
            robot.status['pair_state'] = message.pair_state
            robot.status['worker'] = message.paired_worker
            robot.status['estop'] = message.estop
            robot.status['status_received'] = True

        previous['battery'] = message.battery
        previous['estop'] = message.estop
        previous['paired_worker'] = message.paired_worker
        return changes

    def update_source_state(self, namespace, source, state):
        """접근 또는 셔틀 상태가 바뀌 경우에만 로봇에 반영한다."""
        key = f'{source}_state'
        if self.last_received_status[namespace][key] == state:
            return False
        self.last_received_status[namespace][key] = state
        robot = self.robots[namespace]
        with robot.status_lock:
            robot.status[key] = state
        return True

    def get_available_robots(self):
        """상태를 받았고 미션·E-stop이 없는 로봇을 반환한다."""
        available_robots = []
        for robot in self.robots.values():
            status = robot.get_status()
            if (
                status['status_received']
                and not status['mission']
                and not status['estop']
            ):
                available_robots.append(robot)
        return available_robots

    def assign_mission(self, namespace, mission_id, worker_id, mission_state):
        """로봇 객체에 배정된 미션과 작업자를 저장한다."""
        robot = self.robots[namespace]
        with robot.status_lock:
            robot.status['mission'] = mission_id
            robot.status['mission_state'] = mission_state
            robot.status['target_worker'] = worker_id

    def is_mission_transition_allowed(self, namespace, source, new_state):
        """로봇의 현재 상태를 기준으로 미션 전이를 허용한다."""
        status = self.robots[namespace].get_status()
        current_state = status['mission_state']
        if current_state == Mission.DONE:
            return False
        if source == 'approach':
            if status['shuttle_state'] in RETURN_AND_DOCK_STATES:
                return False
            return current_state in {
                Mission.WAITING,
                Mission.PAIRED,
                Mission.FOLLOWING,
                Mission.STANDBY,
            }
        if new_state in {
            Mission.SHUTTLE,
            Mission.AT_BAY,
            Mission.WAITING,
        }:
            return current_state not in APPROACH_MISSION_STATES
        return True

    def update_mission_state(self, namespace, mission_state):
        """로봇 객체의 미션 진행 상태를 변경한다."""
        robot = self.robots[namespace]
        with robot.status_lock:
            robot.status['mission_state'] = mission_state

    def clear_mission(self, namespace):
        """로봇 객체의 완료된 미션 정보를 비운다."""
        robot = self.robots[namespace]
        with robot.status_lock:
            robot.status['mission'] = ''
            robot.status['mission_state'] = None
            robot.status['target_worker'] = ''
