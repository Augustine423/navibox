import serial
import time
import logging
import os
import threading
import queue
import json
import pytz
from datetime import datetime
from dotenv import load_dotenv
from websocket import create_connection, WebSocketException
import pynmea2
from math import radians, sin, cos, atan2, sqrt

# ==============================================================================
# LOGGING CONFIGURATION
# ==============================================================================
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler(os.getenv("LOG_FILE", "system.log")),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

# GPS reading logger for raw NMEA data
gps_reading_logger = logging.getLogger('gps_reading')
gps_reading_handler = logging.FileHandler(os.getenv("GPS_READING_LOG", "gps_reading.log"))
gps_reading_handler.setFormatter(logging.Formatter('%(asctime)s - %(message)s'))
gps_reading_logger.addHandler(gps_reading_handler)
gps_reading_logger.setLevel(logging.INFO)

# WebSocket data logger for transmitted data
websocket_data_logger = logging.getLogger('websocket_data')
websocket_data_handler = logging.FileHandler(os.getenv("GPS_DATA_LOG", "gps_data.log"))
websocket_data_handler.setFormatter(logging.Formatter('%(message)s'))
websocket_data_logger.addHandler(websocket_data_handler)
websocket_data_logger.setLevel(logging.INFO)

# ==============================================================================
# ENVIRONMENT CONFIGURATION
# ==============================================================================
load_dotenv("/home/mdt/.env")

# ==============================================================================
# SYSTEM CONFIGURATION AND CONSTANTS
# ==============================================================================
WEBSOCKET_URL = os.getenv("WEBSOCKET_URL")
DEVICE_ID = os.getenv("DEVICE_ID")
SERIAL_PORT1 = os.getenv("SERIAL_PORT1")
SERIAL_PORT2 = os.getenv("SERIAL_PORT2")
BAUDRATE = int(os.getenv("BAUDRATE", 115200))
LOGFILE_PATH = os.getenv("LOG_FILE", "system.log")
GPS_READING_LOG = os.getenv("GPS_READING_LOG", "gps_reading.log")
GPS_DATA_LOG = os.getenv("GPS_DATA_LOG", "gps_data.log")
TIMEZONE = os.getenv("TIMEZONE", "Asia/Yangon")

# GPS device list
GPS_DEVICES = [SERIAL_PORT1, SERIAL_PORT2]

# Timing configuration
SEND_INTERVAL = 1  # seconds between data transmissions
DATA_TIMEOUT = 30  # seconds before considering device offline
BAUDRATE_FALLBACKS = [38400, 9600, 4800, 115200]  # fallback baudrates to try

# Enhanced Movement filtering configuration to prevent GPS drift and noise
MIN_MOVEMENT_THRESHOLD = 0.00009  # ~8.8 meters — increased to ignore more small movements
MIN_SPEED_THRESHOLD = 1.0         # km/h — reduced for better heading detection
POSITION_SMOOTHING_FACTOR = 0.8   # 0.0–1.0 — increased smoothing
MAX_POSITION_JUMP = 0.0005        # ~55 meters — reduced to catch smaller glitches
STATIONARY_THRESHOLD = 0.00002    # ~2.2 meters — threshold for completely stationary
UPDATE_COOLDOWN = 4               # seconds — minimum time between position updates

# Additional filtering parameters
MIN_SATELLITE_COUNT = 4           # minimum satellites required for position update
MIN_HDOP_THRESHOLD = 2.5         # maximum HDOP value for position update (lower is better)
SPEED_SMOOTHING_FACTOR = 0.6     # smoothing factor for speed calculations
MIN_ACCURACY_THRESHOLD = 10.0    # minimum accuracy in meters

# GPS fix quality mapping
FIX_QUALITY_MAP = {
    0: "No Fix",
    1: "GPS Fix (SPS)",
    2: "DGPS Fix",
    3: "PPS Fix",
    4: "Real Time Kinematic (RTK) Fixed",
    5: "RTK Float",
    6: "Dead Reckoning",
    7: "Manual Input",
    8: "Simulation"
}

# Log configuration summary
logger.info(f"WEBSOCKET_URL: {WEBSOCKET_URL}, DEVICE_ID: {DEVICE_ID}")
logger.info(f"SERIAL_PORT1: {SERIAL_PORT1}, SERIAL_PORT2: {SERIAL_PORT2}")
logger.info(f"BAUDRATE: {BAUDRATE}")
logger.info(f"LOG_FILE: {LOGFILE_PATH}, GPS_READING_LOG: {GPS_READING_LOG}, GPS_DATA_LOG: {GPS_DATA_LOG}")
logger.info(f"TIMEZONE: {TIMEZONE}")
logger.info(f"Movement Filtering: MIN_THRESHOLD={MIN_MOVEMENT_THRESHOLD}, STATIONARY={STATIONARY_THRESHOLD}, COOLDOWN={UPDATE_COOLDOWN}s")

# ==============================================================================
# UTILITY FUNCTIONS
# ==============================================================================
def calculate_distance(lat1, lon1, lat2, lon2):
    """Calculate distance between two GPS coordinates in meters"""
    if not all([lat1, lon1, lat2, lon2]):
        return float('inf')

    R = 6371000  # Earth's radius in meters
    dlat = radians(lat2 - lat1)
    dlon = radians(lon2 - lon1)
    a = sin(dlat/2)**2 + cos(radians(lat1)) * cos(radians(lat2)) * sin(dlon/2)**2
    c = 2 * atan2(sqrt(a), sqrt(1-a))
    return R * c

def should_update_position(old_lat, old_lon, new_lat, new_lon, speed, satellites, hdop, last_update_time):
    """
    Enhanced function to determine if position should be updated
    Returns True if position should be updated, False otherwise
    """
    current_time = time.time()

    # Check if we have valid coordinates
    if not all([old_lat, old_lon, new_lat, new_lon]):
        return True  # First position or invalid data

    # Check satellite count
    if satellites and satellites < MIN_SATELLITE_COUNT:
        logger.debug(f"Position update rejected: insufficient satellites ({satellites} < {MIN_SATELLITE_COUNT})")
        return False

    # Check HDOP (Horizontal Dilution of Precision)
    if hdop and hdop > MIN_HDOP_THRESHOLD:
        logger.debug(f"Position update rejected: poor HDOP ({hdop} > {MIN_HDOP_THRESHOLD})")
        return False

    # Check update cooldown
    if current_time - last_update_time < UPDATE_COOLDOWN:
        logger.debug(f"Position update rejected: cooldown active ({current_time - last_update_time:.1f}s < {UPDATE_COOLDOWN}s)")
        return False

    # Calculate distance moved
    distance = calculate_distance(old_lat, old_lon, new_lat, new_lon)

    # If completely stationary, don't update
    if distance < STATIONARY_THRESHOLD * 111000:  # convert to meters
        logger.debug(f"Position update rejected: stationary ({distance:.2f}m < {STATIONARY_THRESHOLD * 111000:.2f}m)")
        return False

    # If small movement but vehicle is moving fast enough, update
    if distance < MIN_MOVEMENT_THRESHOLD * 111000:
        if speed and speed > MIN_SPEED_THRESHOLD:
            logger.debug(f"Position update allowed: small movement but sufficient speed ({speed:.1f} km/h)")
            return True
        logger.debug(f"Position update rejected: small movement and low speed ({distance:.2f}m, speed: {speed})")
        return False

    # For normal movements, always update
    logger.debug(f"Position update allowed: normal movement ({distance:.2f}m)")
    return True

def enhanced_smooth_position(old_lat, old_lon, new_lat, new_lon, speed=None, factor=POSITION_SMOOTHING_FACTOR):
    """
    Enhanced position smoothing with speed-based adjustment
    """
    if not all([old_lat, old_lon, new_lat, new_lon]):
        return new_lat, new_lon

    distance = calculate_distance(old_lat, old_lon, new_lat, new_lon)

    # If position jump is too large, it's likely GPS error
    if distance > MAX_POSITION_JUMP * 111000:
        logger.warning(f"Large position jump detected: {distance:.2f}m, using previous position")
        return old_lat, old_lon

    # Adjust smoothing based on speed
    if speed:
        # Less smoothing when moving fast, more when slow
        speed_factor = min(speed / 15.0, 1.0)  # normalize to 0-1 based on 15 km/h
        adjusted_factor = factor * (1 - speed_factor * 0.4)  # reduce smoothing when moving
    else:
        adjusted_factor = factor

    # Apply smoothing for small movements
    if distance < MIN_MOVEMENT_THRESHOLD * 111000:
        smoothed_lat = old_lat * adjusted_factor + new_lat * (1 - adjusted_factor)
        smoothed_lon = old_lon * adjusted_factor + new_lon * (1 - adjusted_factor)
        logger.debug(f"Position smoothed: {distance:.2f}m -> {calculate_distance(old_lat, old_lon, smoothed_lat, smoothed_lon):.2f}m")
        return smoothed_lat, smoothed_lon

    return new_lat, new_lon

# ==============================================================================
# SERIAL PORT VALIDATION
# ==============================================================================
def check_serial_port(port, baudrate):
    """Check if GPS device is connected and responsive on given port"""
    baudrates = [baudrate] + BAUDRATE_FALLBACKS

    for br in baudrates:
        try:
            with serial.Serial(port, br, timeout=2) as ser:
                logger.info(f"Testing serial port {port} at baudrate {br}")
                nmea_count = 0
                valid_sentences = ['GGA', 'RMC', 'GSA', 'GSV']

                for attempt in range(20):  # increased attempts for better detection
                    try:
                        line = ser.readline().decode('ascii', errors='ignore').strip()
                        if line.startswith('$') and any(sentence in line for sentence in valid_sentences):
                            nmea_count += 1
                            logger.debug(f"Valid NMEA detected on {port} at {br}: {line[:50]}...")
                            if nmea_count >= 3:
                                logger.info(f"GPS confirmed on {port} at {br}")
                                return br
                    except Exception as e:
                        logger.debug(f"Error reading from {port}: {e}")
                        continue

                if nmea_count > 0:
                    logger.warning(f"Partial NMEA detection on {port} at {br} ({nmea_count} sentences)")
                    return br
                else:
                    logger.warning(f"No valid NMEA sentences detected on {port} at {br}")

        except serial.SerialException as e:
            logger.error(f"Serial port {port} failed at baudrate {br}: {e}")

    logger.error(f"Could not establish connection to {port} with any baudrate")
    return None

# ==============================================================================
# WEBSOCKET CONNECTIVITY CHECK
# ==============================================================================
def check_websocket(url):
    """Verify WebSocket server connectivity"""
    try:
        ws = create_connection(url, timeout=5)
        ws.close()
        logger.info(f"WebSocket server {url} is reachable")
        return True
    except WebSocketException as e:
        logger.error(f"WebSocket connection failed to {url}: {e}")
        return False
    except Exception as e:
        logger.error(f"Unexpected error connecting to WebSocket {url}: {e}")
        return False

# ==============================================================================
# GPS DATA FORMATTING
# ==============================================================================
def format_gps_data(device_data):
    """Format GPS data for WebSocket transmission"""
    timezone = pytz.timezone(TIMEZONE)
    timestamp = datetime.now(timezone).isoformat()

    # Use primary GPS heading
    primary_heading = device_data.get(SERIAL_PORT1, {}).get("heading")

    formatted_data = {
        "ship_id": DEVICE_ID,
        "device_id": DEVICE_ID,
        "timestamp": timestamp,
        "heading": round(primary_heading, 1) if primary_heading else None,  # reduced precision
        "gps_data": []
    }

    # Process both GPS devices
    for port, label in [(SERIAL_PORT1, "top_gps"), (SERIAL_PORT2, "bottom_gps")]:
        device_info = device_data.get(port, {})

        gps_entry = {
            "gps": label,
            "latitude": round(device_info.get("latitude"), 6) if device_info.get("latitude") else None,
            "longitude": round(device_info.get("longitude"), 6) if device_info.get("longitude") else None,
            "altitude": round(device_info.get("altitude"), 1) if device_info.get("altitude") else None,
            "speed": round(device_info.get("speed"), 1) if device_info.get("speed") else None,
            "satellites": device_info.get("satellites"),
            "satellite_prns": device_info.get("satellite_prns", [])[:15]  # limit to 15 PRNs
        }
        formatted_data["gps_data"].append(gps_entry)

    return formatted_data

# ==============================================================================
# NMEA DATA PARSING AND PROCESSING
# ==============================================================================
def read_gps_data(port, baudrate, data_queue, device_timing):
    """Read and parse GPS data from serial port with enhanced filtering"""
    try:
        with serial.Serial(port, baudrate, timeout=3) as ser:
            logger.info(f"GPS reading started: {port} at {baudrate} baud")

            # Position tracking for smoothing
            last_position = {'latitude': None, 'longitude': None}
            last_speed = None
            speed_buffer = []

            while True:
                try:
                    line = ser.readline().decode('ascii', errors='ignore').strip()

                    if line.startswith('$'):
                        gps_reading_logger.info(f"{port}: {line}")

                        try:
                            msg = pynmea2.parse(line)
                            data = {}

                            # Process GGA messages (position and fix quality)
                            if isinstance(msg, pynmea2.types.GGA):
                                fix_quality = int(msg.gps_qual) if msg.gps_qual else 0

                                if fix_quality > 0 and msg.latitude and msg.longitude:
                                    new_lat = float(msg.latitude)
                                    new_lon = float(msg.longitude)
                                    satellites = int(msg.num_sats) if msg.num_sats else 0
                                    hdop = float(msg.horizontal_dil) if msg.horizontal_dil else None

                                    # Check if position should be updated
                                    should_update = should_update_position(
                                        last_position['latitude'],
                                        last_position['longitude'],
                                        new_lat,
                                        new_lon,
                                        last_speed,
                                        satellites,
                                        hdop,
                                        device_timing[port]['last_position_update']
                                    )

                                    if should_update:
                                        # Apply enhanced position smoothing
                                        if last_position['latitude'] and last_position['longitude']:
                                            new_lat, new_lon = enhanced_smooth_position(
                                                last_position['latitude'],
                                                last_position['longitude'],
                                                new_lat,
                                                new_lon,
                                                last_speed
                                            )

                                        data = {
                                            'latitude': new_lat,
                                            'longitude': new_lon,
                                            'altitude': float(msg.altitude) if msg.altitude else None,
                                            'satellites': satellites,
                                            'fix_quality': fix_quality,
                                            'hdop': hdop
                                        }

                                        last_position = {'latitude': new_lat, 'longitude': new_lon}
                                        device_timing[port]['last_position_update'] = time.time()
                                        device_timing[port]['position_update_count'] += 1

                                        if device_timing[port]['position_update_count'] % 10 == 0:
                                            logger.info(f"{port}: Position updates: {device_timing[port]['position_update_count']}, "
                                                      f"rejected: {device_timing[port]['rejected_update_count']}")
                                    else:
                                        device_timing[port]['rejected_update_count'] += 1
                                        # Still update other data even if position is rejected
                                        data = {
                                            'satellites': satellites,
                                            'fix_quality': fix_quality,
                                            'hdop': hdop
                                        }
                                        if msg.altitude:
                                            data['altitude'] = float(msg.altitude)

                                    logger.debug(f"{port} GGA: Fix={FIX_QUALITY_MAP.get(fix_quality, 'Unknown')}, "
                                               f"Sats={satellites}, HDOP={hdop}, Updated={should_update}")

                            # Process RMC messages (speed and heading)
                            elif isinstance(msg, pynmea2.types.RMC):
                                if msg.status == 'A':  # Valid fix
                                    raw_speed = float(msg.spd_over_grnd) * 1.852 if msg.spd_over_grnd else 0.0  # convert knots to km/h

                                    # Apply speed smoothing
                                    if last_speed is not None:
                                        smoothed_speed = last_speed * SPEED_SMOOTHING_FACTOR + raw_speed * (1 - SPEED_SMOOTHING_FACTOR)
                                    else:
                                        smoothed_speed = raw_speed

                                    last_speed = smoothed_speed

                                    # Only use heading if speed is above threshold
                                    if smoothed_speed >= MIN_SPEED_THRESHOLD:
                                        data = {
                                            'speed': smoothed_speed,
                                            'heading': float(msg.true_course) if msg.true_course else None,
                                            'date': msg.datestamp,
                                            'time': msg.timestamp
                                        }
                                        logger.debug(f"{port} RMC: Speed={smoothed_speed:.1f}km/h, Heading={data.get('heading')}")
                                    else:
                                        data = {
                                            'speed': smoothed_speed,
                                            'heading': None,  # no heading when stationary
                                            'date': msg.datestamp,
                                            'time': msg.timestamp
                                        }
                                        logger.debug(f"{port} RMC: Speed={smoothed_speed:.1f}km/h (stationary)")

                            # Process GSA messages (satellite PRNs and DOP)
                            elif isinstance(msg, pynmea2.types.GSA):
                                prns = []
                                for prn in msg.sv_ids:
                                    if prn and prn.isdigit():
                                        prns.append(int(prn))

                                if prns:
                                    data = {
                                        'satellite_prns': prns,
                                        'pdop': float(msg.pdop) if msg.pdop else None,
                                        'hdop': float(msg.hdop) if msg.hdop else None,
                                        'vdop': float(msg.vdop) if msg.vdop else None
                                    }

                            # Process GSV messages (satellites in view)
                            elif isinstance(msg, pynmea2.types.GSV):
                                satellites_in_view = []
                                for i in range(4):
                                    try:
                                        sat_prn = getattr(msg, f'sv_prn_{i+1:02d}', None)
                                        if sat_prn and sat_prn.isdigit():
                                            satellites_in_view.append(int(sat_prn))
                                    except (AttributeError, ValueError):
                                        break

                                if satellites_in_view:
                                    data = {'satellite_prns': satellites_in_view}

                            # Queue data if any was parsed
                            if data:
                                data_queue.put((port, data))

                        except pynmea2.ParseError as e:
                            logger.debug(f"NMEA parse error on {port}: {e}")
                        except Exception as e:
                            logger.warning(f"Unexpected error parsing NMEA on {port}: {e}")

                except serial.SerialException as e:
                    logger.error(f"Serial communication error on {port}: {e}")
                    break
                except Exception as e:
                    logger.error(f"Unexpected error reading from {port}: {e}")
                    break

    except Exception as e:
        logger.error(f"Failed to initialize serial port {port}: {e}")
        return

# ==============================================================================
# MAIN DATA PROCESSING AND TRANSMISSION
# ==============================================================================
def send_gps_data():
    """Main function to collect and transmit GPS data"""
    logger.info("Starting GPS data collection system with enhanced movement filtering...")

    # Validate required environment variables
    required_vars = {
        "WEBSOCKET_URL": WEBSOCKET_URL,
        "DEVICE_ID": DEVICE_ID,
        "SERIAL_PORT1": SERIAL_PORT1,
        "SERIAL_PORT2": SERIAL_PORT2,
        "BAUDRATE": BAUDRATE,
        "LOG_FILE": LOGFILE_PATH,
        "GPS_READING_LOG": GPS_READING_LOG,
        "GPS_DATA_LOG": GPS_DATA_LOG,
        "TIMEZONE": TIMEZONE
    }

    for var_name, var_value in required_vars.items():
        if not var_value:
            logger.error(f"Environment variable {var_name} is not set")
            return False

    # Validate GPS hardware connectivity
    logger.info("Validating GPS hardware connectivity...")
    baudrate1 = check_serial_port(SERIAL_PORT1, BAUDRATE)
    if not baudrate1:
        logger.error(f"Cannot connect to primary GPS on {SERIAL_PORT1}")
        return False

    baudrate2 = check_serial_port(SERIAL_PORT2, BAUDRATE)
    if not baudrate2:
        logger.error(f"Cannot connect to secondary GPS on {SERIAL_PORT2}")
        return False

    # Validate WebSocket server connectivity
    logger.info("Validating WebSocket server connectivity...")
    if not check_websocket(WEBSOCKET_URL):
        logger.error("WebSocket server is not reachable")
        return False

    # Initialize data structures with enhanced timing
    logger.info("Initializing data structures with enhanced filtering...")
    device_data = {}
    device_timing = {}

    for dev in GPS_DEVICES:
        device_data[dev] = {
            "latitude": None,
            "longitude": None,
            "altitude": None,
            "speed": None,
            "heading": None,
            "satellites": None,
            "satellite_prns": [],
            "fix_quality": 0,
            "hdop": None,
            "pdop": None,
            "vdop": None
        }
        device_timing[dev] = {
            "last_position_update": 0,
            "last_speed_update": 0,
            "position_update_count": 0,
            "rejected_update_count": 0
        }

    # Timing variables
    last_data_time = {dev: time.time() for dev in GPS_DEVICES}
    last_send_time = time.time()

    # Connection retry configuration
    retry_delay = 2
    max_delay = 60

    # Start GPS data reading threads
    logger.info("Starting GPS data reading threads...")
    data_queue = queue.Queue(maxsize=1000)
    threads = []

    for port, baudrate in [(SERIAL_PORT1, baudrate1), (SERIAL_PORT2, baudrate2)]:
        thread = threading.Thread(
            target=read_gps_data,
            args=(port, baudrate, data_queue, device_timing),
            daemon=True,
            name=f"GPS_Reader_{port}"
        )
        thread.start()
        threads.append(thread)
        logger.info(f"GPS reader thread started for {port}")

    # Main data processing loop
    logger.info("Starting main data processing loop...")
    ws = None
    successful_transmissions = 0

    while True:
        try:
            # Establish WebSocket connection if needed
            if ws is None:
                logger.info("Establishing WebSocket connection...")
                try:
                    ws = create_connection(WEBSOCKET_URL, timeout=10)
                    logger.info("WebSocket connection established")
                    retry_delay = 2
                except WebSocketException as e:
                    logger.error(f"Failed to establish WebSocket connection: {e}")
                    ws = None
                    logger.info(f"Retrying in {retry_delay} seconds...")
                    time.sleep(retry_delay)
                    retry_delay = min(retry_delay * 2, max_delay)
                    continue

            # Process queued GPS data
            while not data_queue.empty():
                try:
                    port, data = data_queue.get_nowait()
                    current_time = time.time()
                    last_data_time[port] = current_time

                    # Handle fix quality changes
                    if 'fix_quality' in data:
                        old_quality = device_data[port].get('fix_quality', 0)
                        new_quality = data['fix_quality']

                        if old_quality != new_quality:
                            logger.info(f"{port} fix quality changed: {FIX_QUALITY_MAP.get(old_quality, 'Unknown')} -> {FIX_QUALITY_MAP.get(new_quality, 'Unknown')}")

                        # Reset position data if no fix
                        if new_quality == 0:
                            device_data[port].update({
                                'latitude': None,
                                'longitude': None,
                                'altitude': None,
                                'speed': None,
                                'heading': None
                            })

                    # Merge satellite PRNs
                    if 'satellite_prns' in data:
                        current_prns = set(device_data[port].get('satellite_prns', []))
                        new_prns = set(data['satellite_prns'])
                        merged_prns = list(current_prns.union(new_prns))

                        # Limit PRN list size
                        if len(merged_prns) > 30:
                            merged_prns = merged_prns[-20:]

                        data['satellite_prns'] = merged_prns

                    # Update device data
                    device_data[port].update(data)

                except queue.Empty:
                    break
                except Exception as e:
                    logger.warning(f"Error processing GPS data: {e}")

            # Check for data timeout
            current_time = time.time()
            for dev in GPS_DEVICES:
                if current_time - last_data_time[dev] > DATA_TIMEOUT:
                    logger.warning(f"No data from {dev} for {DATA_TIMEOUT} seconds - resetting")
                    device_data[dev].update({
                        'latitude': None,
                        'longitude': None,
                        'altitude': None,
                        'speed': None,
                        'heading': None,
                        'satellites': None,
                        'satellite_prns': [],
                        'fix_quality': 0,
                        'hdop': None,
                        'pdop': None,
                        'vdop': None
                    })

            # Send data at regular intervals
            if current_time - last_send_time >= SEND_INTERVAL:
                gps_data = format_gps_data(device_data)

                # Log transmission summary with filtering stats
                output_lines = [
                    f"GPS Data Transmission: {gps_data['timestamp']}",
                    f"Ship ID: {gps_data['ship_id']}",
                    f"Heading: {gps_data['heading']}" if gps_data['heading'] else "Heading: N/A",
                    "=" * 60
                ]

                for gps_entry in gps_data['gps_data']:
                    status = "ACTIVE" if gps_entry['latitude'] and gps_entry['longitude'] else "NO FIX"
                    port = SERIAL_PORT1 if gps_entry['gps'] == 'top_gps' else SERIAL_PORT2
                    timing_info = device_timing[port]

                    output_lines.extend([
                        f"{gps_entry['gps'].upper()}: {status}",
                        f"  Position: {gps_entry['latitude']}, {gps_entry['longitude']}" if gps_entry['latitude'] else "  Position: No Fix",
                        f"  Altitude: {gps_entry['altitude']}m" if gps_entry['altitude'] else "  Altitude: N/A",
                        f"  Speed: {gps_entry['speed']} km/h" if gps_entry['speed'] else "  Speed: N/A",
                        f"  Satellites: {gps_entry['satellites']}" if gps_entry['satellites'] else "  Satellites: N/A",
                        f"  PRNs: {', '.join(map(str, gps_entry['satellite_prns'][:10]))}{'...' if len(gps_entry['satellite_prns']) > 10 else ''}",
                        f"  Updates: {timing_info['position_update_count']} accepted, {timing_info['rejected_update_count']} rejected",
                        ""
                    ])

                output_str = "\n".join(output_lines)
                print(output_str)

                # Transmit data
                logger.info(f"Transmitting GPS data (#{successful_transmissions + 1})")
                try:
                    json_data = json.dumps(gps_data)
                    logger.debug(f"Data to send: {json_data}")
                    ws.send(json_data)
                    websocket_data_logger.info(json_data)
                    successful_transmissions += 1

                    if successful_transmissions % 100 == 0:
                        logger.info(f"Milestone: {successful_transmissions} successful transmissions")

                except WebSocketException as e:
                    logger.error(f"WebSocket transmission failed: {e}")
                    ws = None
                    retry_delay = 2
                except Exception as e:
                    logger.error(f"Unexpected error during transmission: {e}")
                    ws = None
                    retry_delay = 2

                last_send_time = current_time

            # Brief pause to prevent CPU overload
            time.sleep(0.1)

        except KeyboardInterrupt:
            logger.info("Received keyboard interrupt - shutting down gracefully...")
            break
        except Exception as e:
            logger.error(f"Unexpected error in main loop: {e}")
            time.sleep(1)

    # Cleanup
    logger.info("Cleaning up resources...")
    if ws:
        try:
            ws.close()
            logger.info("WebSocket connection closed")
        except:
            pass

    logger.info("GPS data collection system stopped")
    return True

# ==============================================================================
# SYSTEM STARTUP AND ENTRY POINT
# ==============================================================================
def main():
    """Main entry point with system initialization"""
    logger.info("=" * 80)
    logger.info("GPS TRACKING SYSTEM - STARTUP")
    logger.info("=" * 80)
    logger.info(f"System started at: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    logger.info(f"Device ID: {DEVICE_ID}")
    logger.info(f"WebSocket URL: {WEBSOCKET_URL}")
    logger.info(f"GPS Devices: {GPS_DEVICES}")
    logger.info(f"Timezone: {TIMEZONE}")
    logger.info("=" * 80)

    try:
        # Start the GPS data collection system
        success = send_gps_data()

        if success:
            logger.info("GPS tracking system completed successfully")
        else:
            logger.error("GPS tracking system failed to initialize")
            return 1

    except Exception as e:
        logger.error(f"Critical system error: {e}")
        return 1

    return 0

if __name__ == "__main__":
    import sys
    sys.exit(main())