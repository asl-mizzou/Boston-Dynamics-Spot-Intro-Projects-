from bosdyn.client.sdk import create_standard_sdk
from dotenv import load_dotenv
from colorama import Fore, init
import helpers
import os
import sys
import time

load_dotenv() # for accessing environment variables
init(autoreset=True) # initialize colorama for terminal colors

# Get environment variables
BOSDYN_CLIENT_USERNAME = os.getenv("BOSDYN_CLIENT_USERNAME")
BOSDYN_CLIENT_PASSWORD = os.getenv("BOSDYN_CLIENT_PASSWORD")
SPOT_IP = os.getenv("SPOT_IP")

def connect_to_robot() -> object:
    """
    This function attempts to connect to Spot 3 times, each attempt with a 10s timeout.
    - Creates SDK
    - Creates Robot object
    - Authenticates
    """

    for attempt in range(3): # attempt to connect 3 times

        print(f"\nConnecting to Robot attempt: {attempt + 1}\n")
        time.sleep(2) # wait before next connection attempt

        try:
            # Create SDK
            sdk = create_standard_sdk('Robot_SDK')

            # Create robot object
            robot = sdk.create_robot(SPOT_IP)

            # Authenticate
            robot.authenticate(BOSDYN_CLIENT_USERNAME, BOSDYN_CLIENT_PASSWORD, timeout=10.0)

            # Sync
            robot.time_sync.wait_for_sync()

            # Get robots serial number through id client
            robot_id = helpers.get_robot_id(robot)
            serial_number = robot_id.serial_number

            # Connected
            connected_message = f"Connected to {serial_number}\n"
            print(Fore.GREEN + connected_message)
            helpers.connection_logger("connection_log.txt", connected_message)
            break

        except Exception as error:

            # Unable to connect
            unable_to_connect_message = "Unable to connect to robot\n"
            print(Fore.RED + unable_to_connect_message)
            helpers.connection_logger("connection_log.txt", unable_to_connect_message)
            print(f"[Error] -> {error}\n")

            if attempt == 2: # prevent traceback
                sys.exit()

    return robot

def check_robot_status(robot) -> None:
    """
    This function shows a robots current status.
    - Battery %
    - Charge stauts
    - Software version
    - Power state
    - Faults or warnings
    """

    # Battery percentage
    battery_level = helpers.get_battery_level(robot)

    if battery_level > 50:

        print("[Battery_Level] -> " + Fore.GREEN + f"{battery_level}\n")

    elif battery_level < 20:

        print("[Battery_Level] -> " + Fore.RED + f"{battery_level}\n")

    else:

        print("[Battery_Level] -> " + Fore.YELLOW + f"{battery_level}\n")
    
    # Charging status
    charge_status = helpers.get_charging_status(robot)
    print(f"[Charge_Status] -> {charge_status}\n")

    # Robot software version
    software_version = helpers.get_robot_software_version(robot)
    print(f"[Software_Version] -> {software_version}\n")

    # Current power state
    power_state = helpers.get_robot_power_state(robot)
    print(f"[Power_State] -> {power_state}\n")

    # Active faults or warnings
    _, faults = helpers.get_active_faults_or_warnings(robot)
    if faults != None:
        for fault in faults:
            print(Fore.RED + f"[Fault] -> {fault.name}: {fault.error_message}\n")
    else:
        print(Fore.GREEN + "No faults or warnings\n")

def is_robot_ready(robot) -> bool:
    """
    This function returns true if robot is ready to use, false otherwise
    - Checks battery level, charging status, and faults or warnings.
    """

    # Battery percentage
    battery_level = helpers.get_battery_level(robot)

    # Charging status
    charge_status = helpers.get_charging_status(robot)

    # Active faults or warnings
    any_faults, _ = helpers.get_active_faults_or_warnings(robot)

    # Robot is ready
    if (battery_level > 20) and (charge_status == "Battery is discharging") and (any_faults == False):

        print("[Status] -> " + Fore.GREEN + "Ready\n")

        return True
    
    # Robot is not ready
    else:

        print("[Status] -> " + Fore.RED + "Not ready\n")

        return False

# Run program
if __name__ == '__main__':
    robot = connect_to_robot()
    check_robot_status(robot)
    can_continue = is_robot_ready(robot)