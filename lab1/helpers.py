from bosdyn.client.robot_state import RobotStateClient
from bosdyn.api.robot_state_pb2 import BatteryState
from bosdyn.client.robot_id import RobotIdClient, create_strict_version
from datetime import datetime
import math

try:
    from bosdyn.api.robot_state_pb2 import PowerState
except Exception:
    PowerState = None

try:
    from bosdyn.client.frame_helpers import get_odom_tform_body
except Exception:
    get_odom_tform_body = None


def get_robot_state(robot):
    """
    Function for creating a state client and accessing a robot's state attributes.
    - Returns robot state.
    """
    state_client = robot.ensure_client(RobotStateClient.default_service_name)
    return state_client.get_robot_state()


def get_robot_id(robot):
    """
    Function for creating an ID client and accessing a robot's ID attributes.
    - Returns robot ID.
    """
    id_client = robot.ensure_client(RobotIdClient.default_service_name)
    return id_client.get_id()


def get_battery_level(robot) -> float:
    """
    Function for getting a robot's current battery level.
    - Gets robot state, then accesses its battery state.
    - Returns robot battery percentage.
    """
    state = get_robot_state(robot)

    if not state.battery_states:
        return 0.0

    battery_state = state.battery_states[0]
    return round(float(battery_state.charge_percentage.value), 1)


def get_charging_status(robot) -> str:
    """
    Function for getting a robot's current charge status.
    - Gets robot state, then uses the API to access charging state.
    - Returns robot current charging status.
    """
    state = get_robot_state(robot)

    if not state.battery_states:
        return "Battery State Unknown"

    battery = state.battery_states[0]

    if battery.status == BatteryState.STATUS_UNKNOWN:
        return "Battery State Unknown"
    if battery.status == BatteryState.STATUS_MISSING:
        return "Battery is missing"
    if battery.status == BatteryState.STATUS_CHARGING:
        return "Battery is charging"
    if battery.status == BatteryState.STATUS_DISCHARGING:
        return "Battery is discharging"

    return "Battery is booting"


def get_robot_software_version(robot) -> str:
    """
    Function for getting a robot's current software version.
    - Gets robot ID, then accesses version number.
    - Returns robot current software version.
    """
    robot_id = get_robot_id(robot)
    return str(create_strict_version(robot_id))


def get_robot_power_state(robot) -> str:
    """
    Function for getting a robot's current power state.
    - Gets robot state, then accesses current power state.
    - Returns robot current power status.
    """
    state = get_robot_state(robot)
    power_status = state.power_state.robot_power_state

    if PowerState is not None:
        if power_status == PowerState.STATE_ON:
            return "ROBOT_POWER_ON"
        if power_status == PowerState.STATE_OFF:
            return "ROBOT_POWER_OFF"
        if power_status == PowerState.STATE_UNKNOWN:
            return "ROBOT_POWER_UNKNOWN"

    # Fallback for SDK enum value differences.
    if power_status == 2:
        return "ROBOT_POWER_ON"
    if power_status == 1:
        return "ROBOT_POWER_OFF"

    return "ROBOT_POWER_UNKNOWN"


def get_active_faults_or_warnings(robot) -> tuple[bool, object | None]:
    """
    Function for getting a robot's current active faults or warnings.
    - Gets robot state, then accesses faults.
    - Returns (True, faults) if there are any faults.
    - Returns (False, None) if not.
    """
    state = get_robot_state(robot)
    faults = state.system_fault_state.faults

    if faults:
        return True, faults

    return False, None


def get_odom_pose(robot) -> dict | None:
    """
    Gets the robot body pose in the odometry frame.
    Returns a dictionary with x, y, z, and yaw, or None if odometry is unavailable.
    """
    if get_odom_tform_body is None:
        return None

    try:
        state = get_robot_state(robot)
        odom_tform_body = get_odom_tform_body(state.kinematic_state.transforms_snapshot)

        if odom_tform_body is None:
            return None

        return {
            "x": float(odom_tform_body.position.x),
            "y": float(odom_tform_body.position.y),
            "z": float(odom_tform_body.position.z),
            "yaw": float(odom_tform_body.rot.to_yaw()),
        }

    except Exception:
        return None


def normalize_angle(angle_radians: float) -> float:
    """
    Normalizes an angle to the range [-pi, pi].
    """
    return math.atan2(math.sin(angle_radians), math.cos(angle_radians))


def heading_arrow(yaw_radians: float) -> str:
    """
    Returns a simple visual facing direction indicator based on yaw.
    """
    yaw = normalize_angle(yaw_radians)
    degrees = math.degrees(yaw)

    if -45 <= degrees < 45:
        return "EAST  ->"
    if 45 <= degrees < 135:
        return "NORTH ^"
    if -135 <= degrees < -45:
        return "SOUTH v"
    return "WEST  <-"


def pose_distance(start_pose: dict | None, end_pose: dict | None) -> float | None:
    """
    Calculates 2D distance between two odometry poses.
    """
    if start_pose is None or end_pose is None:
        return None

    dx = end_pose["x"] - start_pose["x"]
    dy = end_pose["y"] - start_pose["y"]
    return math.sqrt(dx * dx + dy * dy)


def pose_yaw_change(start_pose: dict | None, end_pose: dict | None) -> float | None:
    """
    Calculates normalized yaw change between two odometry poses.
    """
    if start_pose is None or end_pose is None:
        return None

    return normalize_angle(end_pose["yaw"] - start_pose["yaw"])


def connection_logger(path: str, message: str) -> None:
    """
    Function for writing connection attempts to a .txt file.
    """
    with open(path, "a") as file:
        timestamp = datetime.now().strftime("%m/%d/%Y %H:%M:%S")
        file.write(f"Connection attempt - {timestamp} -> {message}\n")