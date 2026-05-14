from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder, blocking_stand
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from colorama import Fore, init
from datetime import datetime
import lab1_1_connection as lab1
import helpers
import math
import time
import sys

init(autoreset=True)

# Short command horizons prevent "command too long" errors.
# Commands are streamed repeatedly instead of sending one long command.
COMMAND_END_TIME_SEC = 0.35
COMMAND_REFRESH_SEC = 0.10
MIN_COMMAND_DURATION_SEC = 0.20

_ROBOT_STANDING = False
_TIME_SYNC_ENDPOINT = None
_TOTAL_PATH_LENGTH = 0.0


def movement_logger(path: str, message: str) -> None:
    """
    Function for writing movement events to a .txt file.
    """
    with open(path, "a") as file:
        timestamp = datetime.now().strftime("%m/%d/%Y %H:%M:%S")
        file.write(f"{timestamp} -> {message}\n")


def _send_robot_command(command_client, command, end_time_secs=None):
    """
    Sends a robot command while using the robot time sync endpoint when available.
    The TypeError fallback keeps the code compatible with older SDK versions.
    """
    try:
        if end_time_secs is not None and _TIME_SYNC_ENDPOINT is not None:
            return command_client.robot_command(
                command=command,
                end_time_secs=end_time_secs,
                timesync_endpoint=_TIME_SYNC_ENDPOINT,
            )

        if end_time_secs is not None:
            return command_client.robot_command(command=command, end_time_secs=end_time_secs)

        return command_client.robot_command(command=command)

    except TypeError:
        if end_time_secs is not None:
            return command_client.robot_command(command=command, end_time_secs=end_time_secs)
        return command_client.robot_command(command=command)


def _stop_robot(command_client) -> None:
    """
    Sends multiple zero-velocity commands so the robot cleanly stops.
    """
    for _ in range(3):
        stop_cmd = RobotCommandBuilder.synchro_velocity_command(
            v_x=0.0,
            v_y=0.0,
            v_rot=0.0,
        )
        _send_robot_command(command_client, stop_cmd, end_time_secs=time.time() + COMMAND_END_TIME_SEC)
        time.sleep(COMMAND_REFRESH_SEC)


def _check_can_move(robot=None) -> bool:
    """
    Basic safety gate before movement commands.
    """
    if not _ROBOT_STANDING:
        print(Fore.RED + "[Safety] Robot has not completed the stand command. Movement blocked.\n")
        return False

    if robot is not None:
        power_state = helpers.get_robot_power_state(robot)
        if power_state != "ROBOT_POWER_ON":
            print(Fore.RED + f"[Safety] Robot motors are not powered on ({power_state}). Movement blocked.\n")
            return False

    return True


def _format_pose(pose: dict | None) -> str:
    """
    Formats an odometry pose for printing/logging.
    """
    if pose is None:
        return "odometry unavailable"

    return (
        f"x={pose['x']:.2f}m, y={pose['y']:.2f}m, "
        f"yaw={math.degrees(pose['yaw']):.1f}deg, facing={helpers.heading_arrow(pose['yaw'])}"
    )


def power_on_robot(robot) -> tuple:
    """
    This function powers on the robot and commands it to stand.
    - Acquires a lease
    - Powers on motors
    - Waits for motor power to report ON
    - Commands robot to stand
    - Returns (command_client, lease_keep_alive)
    """
    global _ROBOT_STANDING, _TIME_SYNC_ENDPOINT

    print("Powering on robot...\n")
    _ROBOT_STANDING = False

    try:
        robot.time_sync.wait_for_sync(timeout_sec=10.0)
        _TIME_SYNC_ENDPOINT = robot.time_sync.endpoint
    except Exception as error:
        print(Fore.YELLOW + f"[Warning] Time sync check failed, continuing with SDK defaults -> {error}\n")
        _TIME_SYNC_ENDPOINT = None

    lease_client = robot.ensure_client(LeaseClient.default_service_name)
    lease_keep_alive = LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True)

    try:
        print("Sending motor power-on command...\n")
        robot.power_on(timeout_sec=20)

        power_wait_start = time.time()
        while time.time() - power_wait_start < 10:
            if helpers.get_robot_power_state(robot) == "ROBOT_POWER_ON":
                print(Fore.GREEN + "Motors powered on\n")
                break
            time.sleep(0.25)
        else:
            raise TimeoutError("Motors did not report ROBOT_POWER_ON within 10 seconds")

        command_client = robot.ensure_client(RobotCommandClient.default_service_name)

        print("Commanding robot to stand...\n")
        blocking_stand(command_client, timeout_sec=10)
        _ROBOT_STANDING = True
        print(Fore.GREEN + "Robot is standing\n")

        movement_logger("movement_log.txt", "Robot powered on and standing")
        return command_client, lease_keep_alive

    except Exception as error:
        print(Fore.RED + "Failed to power on robot or stand\n")
        print(f"[Error] -> {error}\n")
        try:
            lease_keep_alive.shutdown()
        except Exception:
            pass
        sys.exit(1)


def move_forward(command_client, distance_meters: float, speed: float = 0.3, robot=None) -> dict:
    """
    Moves robot forward using short continuous velocity command streaming.
    This avoids long command windows that can cause command-duration errors.
    """
    global _TOTAL_PATH_LENGTH

    result = {
        "type": "move_forward",
        "commanded_distance_m": distance_meters,
        "commanded_speed_mps": speed,
        "actual_distance_m": None,
        "start_pose": None,
        "end_pose": None,
    }

    if not _check_can_move(robot):
        return result

    if distance_meters <= 0:
        print(Fore.YELLOW + "[Skip] Forward distance must be greater than 0.\n")
        return result

    if speed <= 0:
        print(Fore.RED + "[Error] Speed must be greater than 0.\n")
        return result

    duration = max(distance_meters / speed, MIN_COMMAND_DURATION_SEC)
    start_pose = helpers.get_odom_pose(robot) if robot is not None else None
    result["start_pose"] = start_pose

    print(f"Moving forward {distance_meters:.2f}m at {speed:.2f}m/s for {duration:.2f}s...\n")
    print(f"  Start pose: {_format_pose(start_pose)}\n")

    start_time = time.time()

    try:
        while time.time() - start_time < duration:
            move_cmd = RobotCommandBuilder.synchro_velocity_command(
                v_x=speed,
                v_y=0.0,
                v_rot=0.0,
            )
            _send_robot_command(command_client, move_cmd, end_time_secs=time.time() + COMMAND_END_TIME_SEC)
            time.sleep(COMMAND_REFRESH_SEC)

    finally:
        _stop_robot(command_client)
        time.sleep(0.75)

    end_pose = helpers.get_odom_pose(robot) if robot is not None else None
    actual_distance = helpers.pose_distance(start_pose, end_pose)

    result["end_pose"] = end_pose
    result["actual_distance_m"] = actual_distance

    if actual_distance is not None:
        _TOTAL_PATH_LENGTH += actual_distance
        actual_text = f"actual_distance={actual_distance:.2f}m"
    else:
        actual_text = "actual_distance=odometry unavailable"

    print(f"  End pose:   {_format_pose(end_pose)}\n")
    print(Fore.GREEN + f"Movement complete | commanded={distance_meters:.2f}m | {actual_text}\n")

    movement_logger(
        "movement_log.txt",
        (
            f"move_forward | commanded_distance={distance_meters:.2f}m | "
            f"speed={speed:.2f}m/s | duration={duration:.2f}s | {actual_text} | "
            f"start=({_format_pose(start_pose)}) | end=({_format_pose(end_pose)})"
        ),
    )

    return result


def turn_in_place(command_client, angle_radians: float, angular_speed: float = 0.35, robot=None) -> dict:
    """
    Rotates the robot using short continuous velocity command streaming.
    Positive angles turn left. Negative angles turn right.
    """
    result = {
        "type": "turn_in_place",
        "commanded_angle_rad": angle_radians,
        "commanded_angle_deg": math.degrees(angle_radians),
        "commanded_angular_speed_rps": angular_speed,
        "actual_angle_rad": None,
        "actual_angle_deg": None,
        "start_pose": None,
        "end_pose": None,
    }

    if not _check_can_move(robot):
        return result

    angle_radians = helpers.normalize_angle(angle_radians)

    if abs(angle_radians) < 0.03:
        print(Fore.YELLOW + "[Skip] Turn angle is very small, skipping turn.\n")
        return result

    if angular_speed <= 0:
        print(Fore.RED + "[Error] Angular speed must be greater than 0.\n")
        return result

    duration = max(abs(angle_radians) / angular_speed, MIN_COMMAND_DURATION_SEC)
    direction = "right" if angle_radians < 0 else "left"
    degrees = math.degrees(abs(angle_radians))
    v_rot = angular_speed if angle_radians > 0 else -angular_speed

    start_pose = helpers.get_odom_pose(robot) if robot is not None else None
    result["start_pose"] = start_pose

    print(f"Turning {direction} {degrees:.1f} degrees at {angular_speed:.2f}rad/s for {duration:.2f}s...\n")
    print(f"  Start pose: {_format_pose(start_pose)}\n")

    start_time = time.time()

    try:
        while time.time() - start_time < duration:
            turn_cmd = RobotCommandBuilder.synchro_velocity_command(
                v_x=0.0,
                v_y=0.0,
                v_rot=v_rot,
            )
            _send_robot_command(command_client, turn_cmd, end_time_secs=time.time() + COMMAND_END_TIME_SEC)
            time.sleep(COMMAND_REFRESH_SEC)

    finally:
        _stop_robot(command_client)
        time.sleep(0.75)

    end_pose = helpers.get_odom_pose(robot) if robot is not None else None
    actual_angle = helpers.pose_yaw_change(start_pose, end_pose)

    result["end_pose"] = end_pose
    result["actual_angle_rad"] = actual_angle
    result["actual_angle_deg"] = math.degrees(actual_angle) if actual_angle is not None else None

    if actual_angle is not None:
        actual_text = f"actual_rotation={math.degrees(actual_angle):.1f}deg"
    else:
        actual_text = "actual_rotation=odometry unavailable"

    print(f"  End pose:   {_format_pose(end_pose)}\n")
    print(Fore.GREEN + f"Turn complete | commanded={math.degrees(angle_radians):.1f}deg | {actual_text}\n")

    movement_logger(
        "movement_log.txt",
        (
            f"turn_in_place | direction={direction} | commanded_degrees={math.degrees(angle_radians):.1f} | "
            f"angular_speed={angular_speed:.2f}rad/s | duration={duration:.2f}s | {actual_text} | "
            f"start=({_format_pose(start_pose)}) | end=({_format_pose(end_pose)})"
        ),
    )

    return result


def basic_inspection_movement(robot, command_client) -> list:
    """
    Executes a square movement pattern:
    - Move forward 2 meters
    - Turn right 90 degrees
    - Repeat 4 times
    """
    global _TOTAL_PATH_LENGTH

    print("Starting square pattern...\n")
    movement_logger("movement_log.txt", "Square pattern started")
    start_time = time.time()
    _TOTAL_PATH_LENGTH = 0.0
    movement_results = []

    for side in range(4):
        print(f"[Square Pattern] Side {side + 1} of 4\n")
        movement_logger("movement_log.txt", f"Side {side + 1} of 4 started")

        move_result = move_forward(command_client, distance_meters=2.0, speed=0.3, robot=robot)
        movement_results.append(move_result)
        time.sleep(2)

        turn_result = turn_in_place(command_client, angle_radians=-math.pi / 2, angular_speed=0.35, robot=robot)
        movement_results.append(turn_result)
        time.sleep(2)

    elapsed = time.time() - start_time
    final_pose = helpers.get_odom_pose(robot)

    print(Fore.GREEN + f"Square pattern complete in {elapsed:.1f}s\n")
    print(f"Total actual path length: {_TOTAL_PATH_LENGTH:.2f}m\n")
    print(f"Final pose: {_format_pose(final_pose)}\n")

    movement_logger(
        "movement_log.txt",
        f"Square pattern complete | elapsed={elapsed:.1f}s | total_actual_path_length={_TOTAL_PATH_LENGTH:.2f}m | final_pose=({_format_pose(final_pose)})",
    )

    return movement_results


def safe_shutdown(robot, lease_keep_alive) -> None:
    """
    Safely returns robot to sitting position, powers down motors, and releases the lease.
    """
    global _ROBOT_STANDING

    print("Initiating safe shutdown...\n")

    try:
        command_client = robot.ensure_client(RobotCommandClient.default_service_name)
        _stop_robot(command_client)

        print("Commanding robot to sit...\n")
        sit_cmd = RobotCommandBuilder.synchro_sit_command()
        _send_robot_command(command_client, sit_cmd)
        time.sleep(3)
        _ROBOT_STANDING = False
        print(Fore.GREEN + "Robot is sitting\n")

        print("Powering off motors...\n")
        robot.power_off(cut_immediately=False, timeout_sec=20)
        print(Fore.GREEN + "Motors powered off\n")

        movement_logger("movement_log.txt", "Robot safely sat down and powered off")

    except Exception as error:
        print(Fore.RED + f"[Error during shutdown] -> {error}\n")
        movement_logger("movement_log.txt", f"Shutdown error -> {error}")

    finally:
        try:
            lease_keep_alive.shutdown()
            print(Fore.GREEN + "Lease released\n")
            movement_logger("movement_log.txt", "Lease released")
        except Exception as error:
            print(Fore.RED + f"[Error releasing lease] -> {error}\n")


# Run program
if __name__ == '__main__':
    robot = lab1.connect_to_robot()

    if not lab1.is_robot_ready(robot):
        print(Fore.RED + "Robot is not ready. Exiting.\n")
        sys.exit(1)

    command_client, lease_keep_alive = power_on_robot(robot)

    try:
        basic_inspection_movement(robot, command_client)

    except KeyboardInterrupt:
        print(Fore.YELLOW + "Keyboard interrupt received. Shutting down safely.\n")

    except Exception as error:
        print(Fore.RED + f"[Error during movement] -> {error}\n")
        movement_logger("movement_log.txt", f"Movement error -> {error}")

    finally:
        safe_shutdown(robot, lease_keep_alive)