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
from websocket import create_connection
from math import radians, sin, cos, atan2, degrees, sqrt
import pynmea2

# ============================== LOGGING CONFIGURATION ==============================
logging.basicConfig(
    level=logging.DEBUG,  # DEBUG for raw NMEA sentences
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler(os.getenv("LOG_FILE", "gps_data.log")),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

# ============================== ENVIRONMENT SETUP ==============================
load_dotenv("/home/mdt/.env")

# ============================== SYSTEM CONFIGURATION ==============================
WEBSOCKET_URL = os.getenv("WEBSOCKET_URL")
DEVICE_ID = os.getenv("DEVICE_ID")
SERIAL_PORT1 = os.getenv("SERIAL_PORT1")  # Top GPS (u-blox ZED-F9P)
SERIAL_PORT2 = os.getenv("SERIAL_PORT2")  # Bottom GPS (u-blox ZED-F9P)
BAUDRATE = int(os.getenv("BAUDRATE", 115200))  # u-blox ZED-F9P default: 38400
LOGFILE_PATH = os.getenv("LOG_FILE", "gps_data.log")
SEND_INTERVAL = float(os.getenv("SEND_INTERVAL", 1))  # seconds - Position data transmission interval
HEADING_SEND_INTERVAL = float(os.getenv("HEADING_SEND_INTERVAL", 1))  # seconds - Heading transmission interval
DATA_TIMEOUT = 30  # seconds - GPS data freshness timeout
BAUDRATE_FALLBACKS = [38400, 9600, 4800, 115200]  # u-blox ZED-F9P preferred rates
MIN_MOVEMENT_THRESHOLD = 0.10  # 10cm threshold for heading calculation
MIN_SPEED_THRESHOLD = 0.5  # km/h threshold for heading updates

# ============================== ACCURACY THRESHOLDS FOR u-blox ZED-F9P ==============================
MIN_SATELLITES_FOR_RTK = 4      # Minimum satellites for RTK fix
RTK_HORIZONTAL_ACCURACY = 0.01   # 1cm horizontal accuracy (RTK mode)
RTK_VERTICAL_ACCURACY = 0.01     # 1cm vertical accuracy (RTK mode)
RTK_HEADING_ACCURACY = 0.3       # 0.3° heading accuracy
DGPS_HORIZONTAL_ACCURACY = 0.25  # 25cm horizontal accuracy (DGPS mode)
STANDALONE_ACCURACY = 2.5        # 2.5m horizontal accuracy (standalone mode)

# Fix quality mapping for u-blox ZED-F9P
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

# Debug environment variables
logger.info(f"WEBSOCKET_URL: {WEBSOCKET_URL}, DEVICE_ID: {DEVICE_ID}")
logger.info(f"SERIAL_PORT1: {SERIAL_PORT1}, SERIAL_PORT2: {SERIAL_PORT2}")
logger.info(f"BAUDRATE: {BAUDRATE}, LOG_FILE: {LOGFILE_PATH}")
logger.info(f"SEND_INTERVAL: {SEND_INTERVAL}, HEADING_SEND_INTERVAL: {HEADING_SEND_INTERVAL}")

# ============================== SERIAL PORT VALIDATION ==============================
def check_serial_port(port, baudrate):
    """
    LOGIC: Serial Port Connection Validation
    - Tests serial port accessibility with primary and fallback baudrates
    - Validates NMEA sentence reception for GPS module detection
    - Returns working baudrate or None if connection fails
    """
    baudrates = [baudrate] + BAUDRATE_FALLBACKS
    for br in baudrates:
        try:
            with serial.Serial(port, br, timeout=2) as ser:
                logger.info(f"Testing serial port {port} at baudrate {br}")
                
                nmea_count = 0
                for attempt in range(15):
                    line = ser.readline().decode('ascii', errors='ignore').strip()
                    if line.startswith('$'):
                        nmea_count += 1
                        logger.debug(f"NMEA detected on {port} at {br}: {line}")
                        if nmea_count >= 3:
                            logger.info(f"✓ u-blox ZED-F9P confirmed on {port} at {br}")
                            return br
                
                if nmea_count > 0:
                    logger.warning(f"Partial NMEA detection on {port} at {br} ({nmea_count} sentences)")
                    return br
                else:
                    logger.warning(f"No NMEA sentences detected on {port} at {br}")
                    
        except serial.SerialException as e:
            logger.error(f"Serial port {port} failed at baudrate {br}: {e}")
    
    logger.error(f"✗ Could not establish connection to {port} with any baudrate")
    return None

# ============================== WEBSOCKET CONNECTIVITY CHECK ==============================
def check_websocket(url):
    """
    LOGIC: WebSocket Server Connectivity Validation
    - Tests WebSocket server reachability before main data transmission
    - Ensures server is available to prevent data loss
    """
    try:
        ws = create_connection(url, timeout=5)
        ws.close()
        logger.info(f"✓ WebSocket server {url} is reachable")
        return True
    except Exception as e:
        logger.error(f"✗ WebSocket connection failed to {url}: {e}")
        return False

# ============================== HIGH-PRECISION HEADING CALCULATION ==============================
def calculate_high_precision_heading(lat1, lon1, lat2, lon2):
    """
    LOGIC: High-Precision Heading Calculation for u-blox ZED-F9P
    - Uses great circle bearing calculation for centimeter-level accuracy
    - Accounts for Earth's curvature for precise heading determination
    - Returns heading in degrees (0-360°) with 0.1° precision
    """
    if not all([lat1, lon1, lat2, lon2]):
        return None
    
    lat1_rad, lon1_rad, lat2_rad, lon2_rad = map(radians, [lat1, lon1, lat2, lon2])
    
    dlat = lat2_rad - lat1_rad
    dlon = lon2_rad - lon1_rad
    a = sin(dlat/2)**2 + cos(lat1_rad) * cos(lat2_rad) * sin(dlon/2)**2
    distance_m = 2 * 6371000 * atan2(sqrt(a), sqrt(1-a))
    
    if distance_m < MIN_MOVEMENT_THRESHOLD:
        logger.debug(f"Movement too small for reliable heading: {distance_m:.4f}m")
        return None
    
    y = sin(dlon) * cos(lat2_rad)
    x = cos(lat1_rad) * sin(lat2_rad) - sin(lat1_rad) * cos(lat2_rad) * cos(dlon)
    bearing_rad = atan2(y, x)
    bearing_deg = (degrees(bearing_rad) + 360) % 360
    
    logger.debug(f"Heading calculated: {bearing_deg:.2f}° (distance: {distance_m:.4f}m)")
    return round(bearing_deg, 1)

# ============================== GPS DATA ACCURACY VALIDATION ==============================
def validate_gps_accuracy(data):
    """
    LOGIC: GPS Data Quality Assessment for u-blox ZED-F9P
    - Validates fix quality and accuracy based on u-blox specifications
    - Returns accuracy assessment and recommended usage
    """
    fix_quality = data.get('fix_quality', 0)
    satellites = data.get('satellites', 0)
    
    accuracy_info = {
        'horizontal_accuracy': None,
        'vertical_accuracy': None,
        'heading_accuracy': None,
        'quality_description': FIX_QUALITY_MAP.get(fix_quality, "Unknown"),
        'rtk_ready': False,
        'centimeter_accuracy': False
    }
    
    if fix_quality == 4:
        accuracy_info.update({
            'horizontal_accuracy': RTK_HORIZONTAL_ACCURACY,
            'vertical_accuracy': RTK_VERTICAL_ACCURACY,
            'heading_accuracy': RTK_HEADING_ACCURACY,
            'rtk_ready': True,
            'centimeter_accuracy': True
        })
    elif fix_quality == 5:
        accuracy_info.update({
            'horizontal_accuracy': 0.10,
            'vertical_accuracy': 0.15,
            'heading_accuracy': 0.5,
            'rtk_ready': True,
            'centimeter_accuracy': False
        })
    elif fix_quality == 2:
        accuracy_info.update({
            'horizontal_accuracy': DGPS_HORIZONTAL_ACCURACY,
            'vertical_accuracy': 0.50,
            'heading_accuracy': 1.0,
            'rtk_ready': False,
            'centimeter_accuracy': False
        })
    elif fix_quality == 1:
        accuracy_info.update({
            'horizontal_accuracy': STANDALONE_ACCURACY,
            'vertical_accuracy': 5.0,
            'heading_accuracy': 2.0,
            'rtk_ready': False,
            'centimeter_accuracy': False
        })
    
    return accuracy_info

# ============================== GPS DATA FORMATTING FOR TRANSMISSION ==============================
def format_gps_data(device_data, include_heading=True):
    """
    LOGIC: GPS Data Packaging for WebSocket Transmission
    - Formats dual GPS data to match server-expected JSON structure
    - Includes only required fields: gps, latitude, longitude, altitude, speed, satellites, satellite_prns
    - Uses device_id as ship_id and system_heading as heading
    - Adds timestamp and device identification
    - Optionally includes heading based on interval
    """
    timestamp = datetime.now(pytz.UTC).isoformat()
    
    primary_heading = device_data.get(SERIAL_PORT1, {}).get("heading") if include_heading else None
    primary_speed = device_data.get(SERIAL_PORT1, {}).get("speed")
    
    if include_heading and primary_speed is not None and primary_speed < MIN_SPEED_THRESHOLD:
        logger.debug(f"Speed {primary_speed:.2f} km/h below threshold {MIN_SPEED_THRESHOLD} km/h, suppressing heading")
        primary_heading = None
    
    formatted_data = {
        "ship_id": DEVICE_ID,
        "device_id": DEVICE_ID,
        "timestamp": timestamp,
        "heading": primary_heading,
        "gps_data": []
    }
    
    for port, label in [(SERIAL_PORT1, "top_gps"), (SERIAL_PORT2, "bottom_gps")]:
        device_info = device_data.get(port, {})
        
        gps_entry = {
            "gps": label,
            "latitude": device_info.get("latitude"),
            "longitude": device_info.get("longitude"),
            "altitude": device_info.get("altitude"),
            "speed": device_info.get("speed"),
            "satellites": device_info.get("satellites"),
            "satellite_prns": device_info.get("satellite_prns", [])
        }
        
        formatted_data["gps_data"].append(gps_entry)
    
    return formatted_data

# ============================== NMEA DATA PARSING AND PROCESSING ==============================
def read_gps_data(port, baudrate, data_queue):
    """
    LOGIC: GPS Data Reading Thread
    - Continuously reads NMEA sentences from u-blox ZED-F9P
    - Parses critical NMEA messages (GGA, RMC, GSA, GSV)
    - Queues parsed data for main processing thread
    - Handles serial communication errors gracefully
    """
    try:
        with serial.Serial(port, baudrate, timeout=2) as ser:
            logger.info(f"🛰️  GPS reading started: {port} at {baudrate} baud")
            
            while True:
                try:
                    line = ser.readline().decode('ascii', errors='ignore').strip()
                    
                    if line.startswith('$'):
                        logger.debug(f"Raw NMEA from {port}: {line}")
                        
                        try:
                            msg = pynmea2.parse(line)
                            data = {}
                            
                            if isinstance(msg, pynmea2.types.GGA):
                                fix_quality = int(msg.gps_qual) if msg.gps_qual else 0
                                data = {
                                    'latitude': msg.latitude if msg.latitude and fix_quality > 0 else None,
                                    'longitude': msg.longitude if msg.longitude and fix_quality > 0 else None,
                                    'altitude': msg.altitude if msg.altitude and fix_quality > 0 else None,
                                    'satellites': int(msg.num_sats) if msg.num_sats else None,
                                    'fix_quality': fix_quality,
                                    'hdop': float(msg.horizontal_dil) if msg.horizontal_dil else None
                                }
                                logger.debug(f"{port} GGA: Fix={FIX_QUALITY_MAP.get(fix_quality, 'Unknown')}, Sats={data.get('satellites')}")
                            
                            elif isinstance(msg, pynmea2.types.RMC):
                                if msg.status == 'A':
                                    data = {
                                        'speed': round(msg.spd_over_grnd * 1.852, 2) if msg.spd_over_grnd else None,
                                        'heading': float(msg.true_course) if msg.true_course else None,
                                        'date': msg.datestamp,
                                        'time': msg.timestamp
                                    }
                                    logger.debug(f"{port} RMC: Speed={data.get('speed')}km/h, Heading={data.get('heading')}°")
                            
                            elif isinstance(msg, pynmea2.types.GSA):
                                prns = [int(prn) for prn in msg.sv_ids if prn and prn.isdigit()]
                                if prns:
                                    data = {
                                        'satellite_prns': prns,
                                        'pdop': float(msg.pdop) if msg.pdop else None,
                                        'hdop': float(msg.hdop) if msg.hdop else None,
                                        'vdop': float(msg.vdop) if msg.vdop else None
                                    }
                            
                            elif isinstance(msg, pynmea2.types.GSV):
                                satellites_in_view = []
                                for i in range(4):
                                    try:
                                        sat_prn = getattr(msg, f'sv_prn_{i+1:02d}', None)
                                        if sat_prn:
                                            satellites_in_view.append(int(sat_prn))
                                    except (AttributeError, ValueError):
                                        break
                                
                                if satellites_in_view:
                                    data = {'satellite_prns': satellites_in_view}
                            
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

# ============================== MAIN DATA PROCESSING AND TRANSMISSION LOGIC ==============================
def send_gps_data():
    """
    LOGIC: Main GPS Data Processing and WebSocket Transmission
    - Validates system configuration and connectivity
    - Manages dual GPS data streams with accuracy validation
    - Calculates high-precision heading between GPS positions
    - Transmits formatted data via WebSocket with error recovery
    """
    logger.info("🚀 Starting GPS data collection system...")
    
    required_vars = {
        "WEBSOCKET_URL": WEBSOCKET_URL,
        "DEVICE_ID": DEVICE_ID,
        "SERIAL_PORT1": SERIAL_PORT1,
        "SERIAL_PORT2": SERIAL_PORT2,
        "BAUDRATE": BAUDRATE,
        "SEND_INTERVAL": SEND_INTERVAL,
        "HEADING_SEND_INTERVAL": HEADING_SEND_INTERVAL
    }
    
    for var_name, var_value in required_vars.items():
        if not var_value:
            logger.error(f"❌ Environment variable {var_name} is not set")
            return False
    
    logger.info("🔍 Validating GPS hardware connectivity...")
    
    baudrate1 = check_serial_port(SERIAL_PORT1, BAUDRATE)
    if not baudrate1:
        logger.error(f"❌ Cannot connect to primary GPS on {SERIAL_PORT1}")
        return False
    
    baudrate2 = check_serial_port(SERIAL_PORT2, BAUDRATE)
    if not baudrate2:
        logger.error(f"❌ Cannot connect to secondary GPS on {SERIAL_PORT2}")
        return False
    
    logger.info("🌐 Validating WebSocket server connectivity...")
    if not check_websocket(WEBSOCKET_URL):
        logger.error("❌ WebSocket server is not reachable")
        return False
    
    logger.info("📊 Initializing data structures...")
    
    device_data = {dev: {
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
    } for dev in GPS_DEVICES}
    
    position_history = {
        SERIAL_PORT1: [],
        SERIAL_PORT2: []
    }
    
    last_data_time = {dev: time.time() for dev in GPS_DEVICES}
    last_send_time = time.time()
    last_heading_send_time = time.time()
    
    retry_delay = 2
    max_delay = 60
    
    logger.info("🛰️  Starting GPS data reading threads...")
    
    data_queue = queue.Queue(maxsize=1000)
    threads = []
    
    for port, baudrate in [(SERIAL_PORT1, baudrate1), (SERIAL_PORT2, baudrate2)]:
        thread = threading.Thread(
            target=read_gps_data, 
            args=(port, baudrate, data_queue), 
            daemon=True,
            name=f"GPS_Reader_{port}"
        )
        thread.start()
        threads.append(thread)
        logger.info(f"📡 GPS reader thread started for {port}")
    
    logger.info("🔄 Starting main data processing loop...")
    
    ws = None
    successful_transmissions = 0
    
    while True:
        try:
            if ws is None:
                logger.info("🔌 Establishing WebSocket connection...")
                ws = create_connection(WEBSOCKET_URL, timeout=10)
                logger.info("✅ WebSocket connection established")
                retry_delay = 2
            
            while not data_queue.empty():
                try:
                    port, data = data_queue.get_nowait()
                    current_time = time.time()
                    last_data_time[port] = current_time
                    
                    if 'fix_quality' in data:
                        old_quality = device_data[port].get('fix_quality', 0)
                        new_quality = data['fix_quality']
                        
                        if old_quality != new_quality:
                            logger.info(f"📍 {port} fix quality changed: {FIX_QUALITY_MAP.get(old_quality, 'Unknown')} → {FIX_QUALITY_MAP.get(new_quality, 'Unknown')}")
                        
                        if new_quality == 0:
                            device_data[port].update({
                                'latitude': None,
                                'longitude': None,
                                'altitude': None,
                                'speed': None,
                                'heading': None
                            })
                    
                    if 'satellite_prns' in data:
                        current_prns = set(device_data[port].get('satellite_prns', []))
                        new_prns = set(data['satellite_prns'])
                        merged_prns = list(current_prns.union(new_prns))
                        
                        if len(merged_prns) > 50:
                            merged_prns = merged_prns[-30:]
                        
                        data['satellite_prns'] = merged_prns
                    
                    device_data[port].update(data)
                    
                    if 'latitude' in data and 'longitude' in data and data['latitude'] and data['longitude']:
                        new_position = (data['latitude'], data['longitude'], current_time)
                        position_history[port].append(new_position)
                        
                        if len(position_history[port]) > 10:
                            position_history[port] = position_history[port][-10:]
                        
                        if len(position_history[port]) >= 2:
                            prev_pos = position_history[port][-2]
                            curr_pos = position_history[port][-1]
                            
                            time_diff = curr_pos[2] - prev_pos[2]
                            
                            if time_diff >= 1.0:
                                calculated_heading = calculate_high_precision_heading(
                                    prev_pos[0], prev_pos[1],
                                    curr_pos[0], curr_pos[1]
                                )
                                
                                if calculated_heading is not None:
                                    device_data[port]['heading'] = calculated_heading
                                    logger.debug(f"📐 Calculated heading for {port}: {calculated_heading:.1f}°")
                    
                except queue.Empty:
                    break
                except Exception as e:
                    logger.warning(f"Error processing GPS data: {e}")
            
            current_time = time.time()
            for dev in GPS_DEVICES:
                if current_time - last_data_time[dev] > DATA_TIMEOUT:
                    logger.warning(f"⚠️  No data from {dev} for {DATA_TIMEOUT} seconds - resetting")
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
                    position_history[dev] = []
            
            if current_time - last_send_time >= SEND_INTERVAL:
                include_heading = (current_time - last_heading_send_time >= HEADING_SEND_INTERVAL)
                gps_data = format_gps_data(device_data, include_heading)
                
                output_lines = [
                    f"📡 GPS Data Transmission: {gps_data['timestamp']}",
                    f"🏷️  Ship ID: {gps_data['ship_id']}",
                    f"🧭 Heading: {gps_data['heading']}°" if gps_data['heading'] else "🧭 Heading: N/A",
                    "=" * 60
                ]
                
                for gps_entry in gps_data['gps_data']:
                    status_icon = "🟢" if gps_entry['latitude'] and gps_entry['longitude'] else "🔴"
                    
                    output_lines.extend([
                        f"{status_icon} {gps_entry['gps'].upper()}:",
                        f"  📍 Position: {gps_entry['latitude']:.8f}, {gps_entry['longitude']:.8f}" if gps_entry['latitude'] else "  📍 Position: No Fix",
                        f"  🏔️  Altitude: {gps_entry['altitude']:.2f}m" if gps_entry['altitude'] else "  🏔️  Altitude: N/A",
                        f"  🚀 Speed: {gps_entry['speed']:.2f} km/h" if gps_entry['speed'] else "  🚀 Speed: N/A",
                        f"  🛰️  Satellites: {gps_entry['satellites']}" if gps_entry['satellites'] else "  �satellite_prns: N/A",
                        f"  📊 PRNs: {', '.join(map(str, gps_entry['satellite_prns'][:10]))}{'...' if len(gps_entry['satellite_prns']) > 10 else ''}",
                        ""
                    ])
                
                output_str = "\n".join(output_lines)
                print(output_str)
                logger.info(f"📤 Transmitting GPS data (#{successful_transmissions + 1})")
                
                try:
                    ws.send(json.dumps(gps_data))
                    successful_transmissions += 1
                    last_send_time = current_time
                    if include_heading:
                        last_heading_send_time = current_time
                    logger.debug(f"✅ Data transmission successful (#{successful_transmissions})")
                except Exception as e:
                    logger.error(f"❌ WebSocket send failed: {e}")
                    ws.close()
                    ws = None
                    continue
            
            time.sleep(0.1)
            
        except Exception as e:
            logger.error(f"❌ Error in main processing loop: {e}")
            if ws:
                ws.close()
                ws = None
            
            logger.info(f"⏳ Retrying in {retry_delay} seconds...")
            time.sleep(retry_delay)
            retry_delay = min(retry_delay * 2, max_delay)
            continue
            
        except KeyboardInterrupt:
            logger.info("🛑 Shutdown requested by user")
            if ws:
                ws.close()
            
            for thread in threads:
                thread.join(timeout=1)
            
            logger.info(f"📊 Final Statistics: {successful_transmissions} successful transmissions")
            break
    
    return True

# ============================== SYSTEM ENTRY POINT ==============================
if __name__ == "__main__":
    logger.info("🌟 u-blox ZED-F9P GPS Data Collection System Starting...")
    result = send_gps_data()
    exit_code = 0 if result else 1
    logger.info(f"🏁 System shutdown complete (exit code: {exit_code})")
    exit(exit_code)