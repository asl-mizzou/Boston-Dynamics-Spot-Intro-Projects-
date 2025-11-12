from bosdyn.client.robot_state import RobotStateClient
from bosdyn.api.robot_state_pb2 import BatteryState
from bosdyn.client.robot_id import RobotIdClient, create_strict_version
from datetime import datetime

def get_robot_state(robot) -> str:
    """
    Function for creating a state client and accessing a robots state attributes.
    - Returns robots state.
    """

    # Create robot state client
    state_client = robot.ensure_client(RobotStateClient.default_service_name)

    # Request robot state attributes
    state = state_client.get_robot_state()

    return state

def get_robot_id(robot) -> str:
    """
    Function for creating a id client and accessing a robots id attributes.
    - Returns robots id.
    """

    # Create robot id client
    id_client = robot.ensure_client(RobotIdClient.default_service_name)

    # Request robot id
    id = id_client.get_id()

    return id

def get_battery_level(robot) -> int:
    """
    Function for getting a robots current battery level.
    - Gets robots state then accesses its battery state.
    - Returns robots battery level.
    """

    # Get robot state
    state = get_robot_state(robot)

    # Get bettery level
    battery_state = state.battery_states[0]
    battery_level = battery_state.charge_percentage.value

    return battery_level

def get_charging_status(robot) -> str:
    """
    Function for getting a robots current charge status.
    - Gets robots state then uses api the access charging state.
    - Returns robots current charging status.
    """

    # Get robot state
    state = get_robot_state(robot)

    # Get charge status
    charge_status = ""

    for battery in state.battery_states:

        if battery.status == BatteryState.STATUS_UNKNOWN:
            charge_status = "Battery State Unknown"

        elif battery.status == BatteryState.STATUS_MISSING:
            charge_status = "Battery is missing"

        elif battery.status == BatteryState.STATUS_CHARGING:
            charge_status = "Battery is charging"

        elif battery.status == BatteryState.STATUS_DISCHARGING:
            charge_status = "Battery is discharging"

        else: 
            charge_status = "Battery is boooting"

    return charge_status

def get_robot_software_version(robot) -> str:
    """
    Function for getting a robots current software version.
    - Gets robots id then accesses version number.
    - Returns robots current software version.
    """

    # Get robot id
    id = get_robot_id(robot)

    # Get robots software version
    software_version = create_strict_version(id)

    return software_version

def get_robot_power_state(robot) -> str:
    """
    Function for getting a robots current power state.
    - Gets robots state then accesses its current power state.
    - Returns robots current power status.
    """

    # Get robot state
    state = get_robot_state(robot)

    # Get robots power status
    power_status = state.power_state.robot_power_state

    if power_status == 1:
        power_status = "ROBOT_POWER_ON"

    elif power_status == 0:
        power_status = "ROBOT_POWER_OFF"

    else:
        power_status = "ROBOT_POWER_UNKOWN"

    return power_status

def get_active_faults_or_warnings(robot) -> tuple[bool, str | None]:
    """
    Function for getting a robots current active faults or warnings.
    - Gets robots state then accesses its faults.
    - Returns (True, faults) if there are any faults, Returns (False, None) if not.
    """

    # Get robot state
    state = get_robot_state(robot)

    # Get any warnings or faults
    faults = state.system_fault_state.faults

    if faults:

        return True, faults
    
    else:

        faults = None
        return False, faults
    
def connection_logger(path, message):
    """
    Function for wrting connection attempts to .txt file.
    """

    with open(path, "a") as file:
        timestamp = datetime.now().strftime("%m/%d/%Y %H:%M:%S")
        file.write(f"Connection attempt - {timestamp} -> {message}")