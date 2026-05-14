from bosdyn.client.image import ImageClient, build_image_request
from bosdyn.api import image_pb2
from colorama import Fore, init
from datetime import datetime
import lab1_1_connection as lab1
import lab1_2_movement as lab2
import helpers
import math
import os
import sys
import time
import json

try:
    import cv2
    import numpy as np
except Exception:
    cv2 = None
    np = None

try:
    from PIL import Image, ImageStat
except Exception:
    Image = None
    ImageStat = None

init(autoreset=True)

# All 5 Spot fisheye camera sources.
CAMERA_SOURCES = [
    "frontleft_fisheye_image",
    "frontright_fisheye_image",
    "left_fisheye_image",
    "right_fisheye_image",
    "back_fisheye_image",
]

# Inspection points: (name, x_meters, y_meters, final_yaw_radians)
# These points form a simple square route plus a return/start point.
# The robot starts at (0, 0) facing yaw 0.
INSPECTION_POINTS = [
    ("start", 0.0, 0.0, 0.0),
    ("front_wall", 2.0, 0.0, 0.0),
    ("right_corner", 2.0, -2.0, -math.pi / 2),
    ("back_wall", 0.0, -2.0, math.pi),
    ("return_start", 0.0, 0.0, 0.0),
]

MOVE_SPEED_MPS = 0.3
TURN_SPEED_RADPS = 0.35
STABILIZE_SECONDS = 2
IMAGE_QUALITY_PERCENT = 75


def setup_image_capture(robot) -> tuple:
    """
    Initializes the image client and creates a timestamped directory structure.
    - Creates inspections/YYYYMMDD_HHMMSS/ base directory
    - Creates subdirectory for each camera source
    - Returns (image_client, base_dir)
    """
    image_client = robot.ensure_client(ImageClient.default_service_name)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    base_dir = os.path.join("inspections", timestamp)

    for source in CAMERA_SOURCES:
        camera_dir = os.path.join(base_dir, source)
        os.makedirs(camera_dir, exist_ok=True)

    print(Fore.GREEN + f"Inspection directory created: {base_dir}\n")
    return image_client, base_dir


def _build_camera_request(source_name: str):
    """
    Builds a JPEG image request for one camera source.
    """
    return build_image_request(
        source_name,
        image_format=image_pb2.Image.FORMAT_JPEG,
        quality_percent=IMAGE_QUALITY_PERCENT,
    )


def _get_image_responses(image_client) -> list:
    """
    Requests all camera images at once.
    If the multi-camera request fails, retries each camera individually so one bad source does not stop the lab.
    """
    requests = [_build_camera_request(source) for source in CAMERA_SOURCES]

    try:
        return list(image_client.get_image(requests))

    except Exception as error:
        print(Fore.YELLOW + f"[Warning] Multi-camera request failed -> {error}\n")
        print(Fore.YELLOW + "Retrying cameras one at a time...\n")

        responses = []
        for source in CAMERA_SOURCES:
            try:
                single_response = image_client.get_image([_build_camera_request(source)])
                responses.extend(list(single_response))
            except Exception as single_error:
                print(Fore.RED + f"  [Failed] {source} -> {single_error}\n")

        return responses


def capture_inspection_images(image_client, base_dir: str, location_name: str) -> list:
    """
    Captures images from all 5 fisheye cameras.
    - Saves images as {location}_{camera}_{timestamp}.jpg
    - Skips unavailable cameras and logs failures
    - Returns a list of metadata dictionaries for saved images
    """
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    saved_records = []

    print(f"Capturing images at location: {location_name}\n")
    image_responses = _get_image_responses(image_client)

    if not image_responses:
        print(Fore.RED + "No image responses received.\n")
        return saved_records

    for response in image_responses:
        try:
            source_name = response.source.name

            if hasattr(response, "status") and response.status != image_pb2.ImageResponse.STATUS_OK:
                print(Fore.RED + f"  [Failed] {source_name} returned status {response.status}\n")
                continue

            image_bytes = response.shot.image.data
            if not image_bytes:
                print(Fore.RED + f"  [Failed] {source_name} returned empty image data\n")
                continue

            filename = f"{location_name}_{source_name}_{timestamp}.jpg"
            save_path = os.path.join(base_dir, source_name, filename)

            with open(save_path, "wb") as image_file:
                image_file.write(image_bytes)

            record = {
                "location": location_name,
                "camera": source_name,
                "timestamp": timestamp,
                "path": save_path,
                "filename": filename,
                "relative_path": os.path.relpath(save_path, base_dir),
            }
            saved_records.append(record)

            print(Fore.GREEN + f"  [Saved] {filename}\n")

        except Exception as error:
            source_name = getattr(getattr(response, "source", None), "name", "unknown_camera")
            print(Fore.RED + f"  [Failed] {source_name}: {error}\n")

    expected = len(CAMERA_SOURCES)
    actual = len(saved_records)
    print(f"Capture complete: {actual}/{expected} images saved for {location_name}\n")

    return saved_records


def _quality_from_cv2(path: str) -> tuple:
    """
    Uses OpenCV to get dimensions and average brightness.
    Returns (width, height, avg_brightness).
    """
    if cv2 is None or np is None:
        return None, None, None

    image = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if image is None:
        return None, None, None

    height, width = image.shape[:2]
    avg_brightness = float(np.mean(image))
    return width, height, avg_brightness


def _quality_from_pil(path: str) -> tuple:
    """
    Uses Pillow as a fallback to get dimensions and average brightness.
    Returns (width, height, avg_brightness).
    """
    if Image is None or ImageStat is None:
        return None, None, None

    try:
        with Image.open(path) as image:
            grayscale = image.convert("L")
            width, height = grayscale.size
            avg_brightness = float(ImageStat.Stat(grayscale).mean[0])
            return width, height, avg_brightness
    except Exception:
        return None, None, None


def verify_image_quality(image_records: list) -> list:
    """
    Checks image quality for each captured image.
    - Verifies file size > 10KB
    - Checks dimensions are reasonable
    - Checks image is not completely black
    - Logs average brightness per image
    - Returns list of quality report dictionaries
    """
    quality_reports = []

    print("Verifying image quality...\n")

    for record in image_records:
        path = record["path"]
        report = {
            "location": record["location"],
            "camera": record["camera"],
            "path": path,
            "filename": record["filename"],
            "passed": True,
            "warnings": [],
            "file_size_bytes": 0,
            "width": None,
            "height": None,
            "avg_brightness": None,
        }

        try:
            file_size = os.path.getsize(path)
            report["file_size_bytes"] = file_size

            if file_size < 10_000:
                report["warnings"].append(f"File too small: {file_size} bytes")
                report["passed"] = False

            width, height, avg_brightness = _quality_from_cv2(path)
            if width is None:
                width, height, avg_brightness = _quality_from_pil(path)

            report["width"] = width
            report["height"] = height
            report["avg_brightness"] = round(avg_brightness, 2) if avg_brightness is not None else None

            if width is None or height is None:
                report["warnings"].append("Could not read image dimensions")
                report["passed"] = False
            elif width < 100 or height < 100:
                report["warnings"].append(f"Image dimensions seem too small: {width}x{height}")
                report["passed"] = False

            if avg_brightness is None:
                report["warnings"].append("Could not calculate brightness")
                report["passed"] = False
            elif avg_brightness < 5:
                report["warnings"].append("Image may be completely black or obstructed")
                report["passed"] = False

        except Exception as error:
            report["warnings"].append(f"Quality check error: {error}")
            report["passed"] = False

        status = Fore.GREEN + "PASS" if report["passed"] else Fore.RED + "FAIL"
        dimensions = "unknown" if report["width"] is None else f"{report['width']}x{report['height']}"
        brightness = "N/A" if report["avg_brightness"] is None else report["avg_brightness"]

        print(
            f"  [{status}] {report['filename']} | "
            f"size={report['file_size_bytes']} bytes | dimensions={dimensions} | brightness={brightness}\n"
        )

        if report["warnings"]:
            for warning in report["warnings"]:
                print(Fore.YELLOW + f"       warning: {warning}\n")

        quality_reports.append(report)

    return quality_reports


def _turn_to_heading(command_client, robot, current_yaw: float, target_yaw: float) -> float:
    """
    Turns from current heading to target heading using the shortest angle.
    Returns the new assumed heading.
    """
    delta_yaw = helpers.normalize_angle(target_yaw - current_yaw)

    if abs(delta_yaw) > 0.05:
        lab2.turn_in_place(command_client, angle_radians=delta_yaw, angular_speed=TURN_SPEED_RADPS, robot=robot)

    return target_yaw


def complete_inspection_routine(robot) -> dict:
    """
    Runs the full inspection routine.
    - Powers on and stands up robot
    - Moves to each inspection point
    - Stabilizes 2 seconds before capturing
    - Captures images from all cameras
    - Safely returns to sit position
    - Returns inspection data dictionary
    """
    image_client, base_dir = setup_image_capture(robot)
    all_image_records = []
    all_quality_reports = []
    movement_records = []
    command_client = None
    lease_keep_alive = None

    battery_start = helpers.get_battery_level(robot)
    inspection_start = time.time()
    generated_at = datetime.now().strftime("%m/%d/%Y %H:%M:%S")

    print(f"[Battery at start] {battery_start}%\n")

    try:
        command_client, lease_keep_alive = lab2.power_on_robot(robot)
        total_points = len(INSPECTION_POINTS)
        current_yaw = INSPECTION_POINTS[0][3]

        for idx, (name, x, y, final_yaw) in enumerate(INSPECTION_POINTS):
            print(Fore.CYAN + f"[Inspection Point {idx + 1} of {total_points}] {name}\n")

            if idx > 0:
                previous_name, previous_x, previous_y, _ = INSPECTION_POINTS[idx - 1]
                dx = x - previous_x
                dy = y - previous_y
                distance = math.sqrt(dx * dx + dy * dy)

                if distance > 0.05:
                    travel_heading = math.atan2(dy, dx)
                    current_yaw = _turn_to_heading(command_client, robot, current_yaw, travel_heading)
                    move_result = lab2.move_forward(command_client, distance_meters=distance, speed=MOVE_SPEED_MPS, robot=robot)
                    move_result["from_point"] = previous_name
                    move_result["to_point"] = name
                    movement_records.append(move_result)

                current_yaw = _turn_to_heading(command_client, robot, current_yaw, final_yaw)

            print(f"Stabilizing for {STABILIZE_SECONDS} seconds before image capture...\n")
            time.sleep(STABILIZE_SECONDS)

            saved_records = capture_inspection_images(image_client, base_dir, name)
            all_image_records.extend(saved_records)

            quality_reports = verify_image_quality(saved_records)
            all_quality_reports.extend(quality_reports)

            time.sleep(1)

    except KeyboardInterrupt:
        print(Fore.YELLOW + "Keyboard interrupt received. Shutting down safely.\n")

    except Exception as error:
        print(Fore.RED + f"[Error during inspection] -> {error}\n")

    finally:
        if lease_keep_alive is not None:
            lab2.safe_shutdown(robot, lease_keep_alive)

    battery_end = helpers.get_battery_level(robot)
    elapsed_seconds = time.time() - inspection_start

    print(f"[Battery at end] {battery_end}%\n")
    print(Fore.GREEN + f"Inspection routine finished in {elapsed_seconds:.1f} seconds\n")

    return {
        "base_dir": base_dir,
        "generated_at": generated_at,
        "elapsed_seconds": elapsed_seconds,
        "battery_start": battery_start,
        "battery_end": battery_end,
        "inspection_points": INSPECTION_POINTS,
        "image_records": all_image_records,
        "quality_reports": all_quality_reports,
        "movement_records": movement_records,
    }


def _generate_svg_map(points: list) -> str:
    """
    Generates a simple SVG map of inspection point locations.
    """
    if not points:
        return ""

    xs = [point[1] for point in points]
    ys = [point[2] for point in points]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)

    width = 600
    height = 400
    pad = 60

    span_x = max(max_x - min_x, 1.0)
    span_y = max(max_y - min_y, 1.0)

    def map_x(x):
        return pad + ((x - min_x) / span_x) * (width - 2 * pad)

    def map_y(y):
        return height - pad - ((y - min_y) / span_y) * (height - 2 * pad)

    svg = []
    svg.append(f'<svg width="{width}" height="{height}" style="border:1px solid #999; background:#f8f8f8;">')
    svg.append('<text x="20" y="25" font-size="18" font-family="Arial">Inspection Point Map</text>')

    for idx in range(len(points) - 1):
        x1 = map_x(points[idx][1])
        y1 = map_y(points[idx][2])
        x2 = map_x(points[idx + 1][1])
        y2 = map_y(points[idx + 1][2])
        svg.append(f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" stroke="#333" stroke-width="2" />')

    for idx, (name, x, y, yaw) in enumerate(points, start=1):
        px = map_x(x)
        py = map_y(y)
        dx = 20 * math.cos(yaw)
        dy = -20 * math.sin(yaw)

        svg.append(f'<circle cx="{px:.1f}" cy="{py:.1f}" r="10" fill="#1f77b4" />')
        svg.append(f'<line x1="{px:.1f}" y1="{py:.1f}" x2="{px + dx:.1f}" y2="{py + dy:.1f}" stroke="#d62728" stroke-width="3" />')
        svg.append(f'<text x="{px + 14:.1f}" y="{py - 12:.1f}" font-size="12" font-family="Arial">{idx}. {name}</text>')
        svg.append(f'<text x="{px + 14:.1f}" y="{py + 4:.1f}" font-size="10" font-family="Arial">({x:.1f}, {y:.1f})</text>')

    svg.append('</svg>')
    return "\n".join(svg)


def generate_inspection_report(inspection_data: dict) -> None:
    """
    Generates a text report, HTML report, and JSON manifest.
    - Summarizes captured images, locations, quality flags, time, and battery use
    - Saves report files into the base inspection directory
    """
    base_dir = inspection_data["base_dir"]
    image_records = inspection_data["image_records"]
    quality_reports = inspection_data["quality_reports"]
    battery_start = inspection_data["battery_start"]
    battery_end = inspection_data["battery_end"]
    elapsed_seconds = inspection_data["elapsed_seconds"]
    generated_at = datetime.now().strftime("%m/%d/%Y %H:%M:%S")

    total_images = len(image_records)
    failed_images = sum(1 for report in quality_reports if not report["passed"])
    passed_images = total_images - failed_images

    manifest_path = os.path.join(base_dir, "inspection_manifest.json")
    with open(manifest_path, "w") as manifest_file:
        json.dump(inspection_data, manifest_file, indent=2, default=str)

    print(Fore.GREEN + f"Manifest saved: {manifest_path}\n")

    # Text report.
    text_report_path = os.path.join(base_dir, "inspection_report.txt")

    with open(text_report_path, "w") as report_file:
        report_file.write("Inspection Report\n")
        report_file.write(f"Generated: {generated_at}\n")
        report_file.write(f"{'=' * 60}\n\n")
        report_file.write(f"Total inspection time: {elapsed_seconds:.1f} seconds\n")
        report_file.write(f"Total images captured: {total_images}\n")
        report_file.write(f"Images passed quality check: {passed_images}\n")
        report_file.write(f"Images with quality issues: {failed_images}\n")
        report_file.write(f"Battery at start: {battery_start}%\n")
        report_file.write(f"Battery at end: {battery_end}%\n\n")

        report_file.write("Inspection Points\n")
        report_file.write(f"{'-' * 60}\n")
        for idx, (name, x, y, yaw) in enumerate(inspection_data["inspection_points"], start=1):
            report_file.write(f"{idx}. {name}: x={x:.2f}m, y={y:.2f}m, yaw={math.degrees(yaw):.1f}deg\n")

        report_file.write("\nCaptured Images\n")
        report_file.write(f"{'-' * 60}\n")
        for record in image_records:
            report_file.write(f"{record['location']} | {record['camera']} | {record['relative_path']}\n")

        report_file.write("\nQuality Warnings\n")
        report_file.write(f"{'-' * 60}\n")
        if failed_images == 0:
            report_file.write("No quality warnings detected.\n")
        else:
            for report in quality_reports:
                if not report["passed"]:
                    report_file.write(f"{report['filename']}: {', '.join(report['warnings'])}\n")

        report_file.write("\nMovement Records\n")
        report_file.write(f"{'-' * 60}\n")
        for movement in inspection_data["movement_records"]:
            report_file.write(json.dumps(movement, default=str) + "\n")

    print(Fore.GREEN + f"Text report saved: {text_report_path}\n")

    # HTML report.
    html_report_path = os.path.join(base_dir, "inspection_report.html")
    svg_map = _generate_svg_map(inspection_data["inspection_points"])

    records_by_location = {}
    for record in image_records:
        records_by_location.setdefault(record["location"], []).append(record)

    quality_by_path = {report["path"]: report for report in quality_reports}

    with open(html_report_path, "w") as report_file:
        report_file.write("<!DOCTYPE html>\n<html>\n<head>\n")
        report_file.write("<meta charset='UTF-8'>\n")
        report_file.write(f"<title>Inspection Report - {generated_at}</title>\n")
        report_file.write("<style>\n")
        report_file.write("body { font-family: Arial, sans-serif; margin: 24px; }\n")
        report_file.write(".summary { padding: 12px; background: #f0f0f0; border-radius: 8px; }\n")
        report_file.write(".grid { display: flex; flex-wrap: wrap; gap: 16px; }\n")
        report_file.write(".card { border: 1px solid #ccc; border-radius: 8px; padding: 8px; width: 420px; }\n")
        report_file.write("img { max-width: 400px; border-radius: 4px; }\n")
        report_file.write(".pass { color: green; font-weight: bold; }\n")
        report_file.write(".fail { color: red; font-weight: bold; }\n")
        report_file.write("</style>\n</head>\n<body>\n")

        report_file.write(f"<h1>Inspection Report - {generated_at}</h1>\n")
        report_file.write("<div class='summary'>\n")
        report_file.write(f"<p><strong>Total inspection time:</strong> {elapsed_seconds:.1f} seconds</p>\n")
        report_file.write(f"<p><strong>Total images:</strong> {total_images} | <strong>Passed:</strong> {passed_images} | <strong>Quality issues:</strong> {failed_images}</p>\n")
        report_file.write(f"<p><strong>Battery:</strong> {battery_start}% &rarr; {battery_end}%</p>\n")
        report_file.write("</div>\n")

        report_file.write("<h2>Inspection Map</h2>\n")
        report_file.write(svg_map)

        for point_name, _, _, _ in inspection_data["inspection_points"]:
            report_file.write(f"<h2>Point: {point_name}</h2>\n")
            report_file.write("<div class='grid'>\n")

            for record in records_by_location.get(point_name, []):
                quality = quality_by_path.get(record["path"], {})
                passed = quality.get("passed", False)
                status_class = "pass" if passed else "fail"
                status_text = "PASS" if passed else "FAIL"
                brightness = quality.get("avg_brightness", "N/A")
                dimensions = "N/A"
                if quality.get("width") is not None:
                    dimensions = f"{quality.get('width')}x{quality.get('height')}"

                report_file.write("<div class='card'>\n")
                report_file.write(f"<img src='{record['relative_path']}' width='400'><br>\n")
                report_file.write(f"<p><strong>{record['camera']}</strong></p>\n")
                report_file.write(f"<p>{record['filename']}</p>\n")
                report_file.write(f"<p>Status: <span class='{status_class}'>{status_text}</span></p>\n")
                report_file.write(f"<p>Dimensions: {dimensions} | Brightness: {brightness}</p>\n")

                warnings = quality.get("warnings", [])
                if warnings:
                    report_file.write("<ul>\n")
                    for warning in warnings:
                        report_file.write(f"<li>{warning}</li>\n")
                    report_file.write("</ul>\n")

                report_file.write("</div>\n")

            report_file.write("</div>\n")

        report_file.write("</body>\n</html>\n")

    print(Fore.GREEN + f"HTML report saved: {html_report_path}\n")


# Run program.
if __name__ == '__main__':
    robot = lab1.connect_to_robot()

    if not lab1.is_robot_ready(robot):
        print(Fore.RED + "Robot is not ready. Exiting.\n")
        sys.exit(1)

    inspection_data = complete_inspection_routine(robot)
    generate_inspection_report(inspection_data)