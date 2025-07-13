<img width="640" height="640" alt="image" src="https://github.com/user-attachments/assets/b82e0f4d-65ec-4d40-a0c3-27d04e0312f8" />

Navibox GPS Data Collection System
Overview
The Navibox GPS Data Collection System is a Python-based solution for collecting, processing, and transmitting high-precision GPS data from the u-blox ZED-F9P-04B-01 GNSS module. This system is designed for applications requiring centimeter-level accuracy, such as surveying, precision agriculture, drones, robotics, and autonomous navigation. It supports dual-band GNSS, real-time kinematic (RTK) positioning, and data transmission via WebSocket.
The system reads NMEA sentences from two GPS modules (primary and secondary), validates their accuracy, calculates high-precision headings, and transmits formatted data to a WebSocket server. A bash setup script is included to configure the system environment and optionally set up a systemd service for automatic execution.
Features

High-Precision Positioning: Leverages the u-blox ZED-F9P module for centimeter-level accuracy using RTK.
Dual GPS Support: Processes data from two GPS modules (e.g., top and bottom) for enhanced reliability and heading calculation.
Real-Time Data Transmission: Sends GPS data (latitude, longitude, altitude, speed, heading, etc.) to a WebSocket server.
Robust Logging: Logs system events, raw GPS data, and transmitted data to separate files for debugging and analysis.
Accuracy Validation: Validates GPS fix quality (RTK Fixed, RTK Float, DGPS, etc.) and enforces accuracy thresholds.
Automated Setup: Includes a bash script to install dependencies, set up a virtual environment, and configure a systemd service.
Error Handling: Implements robust error handling for serial communication, WebSocket connectivity, and data parsing.

Requirements

Hardware:
u-blox ZED-F9P-04B-01 GNSS module (x2 for dual GPS setup)
Active dual-band GNSS antennas
Linux-based system (e.g., Raspberry Pi, Ubuntu) with serial port access


Software:
Python 3.6+
Bash shell
systemd (optional, for automatic service setup)


Environment Variables:
WEBSOCKET_URL: URL of the WebSocket server
DEVICE_ID: Unique identifier for the device
SERIAL_PORT1: Path to the primary GPS serial port (e.g., /dev/ttyACM0)
SERIAL_PORT2: Path to the secondary GPS serial port (e.g., /dev/ttyACM1)
BAUDRATE: Serial port baudrate (default: 115200)
LOG_FILE: Path to the system log file (default: system.log)
GPS_READING_LOG: Path to the GPS reading log file (default: gps_reading.log)
GPS_DATA_LOG: Path to the WebSocket data log file (default: gps_data.log)



Installation

Clone the Repository:
```
git clone https://github.com/Augustine423/navibox
```
cd navibox


Set Up Environment Variables:Create a .env file in /home/mdt/ with the required variables:
WEBSOCKET_URL=ws://your-websocket-server:port
DEVICE_ID=your-device-id
SERIAL_PORT1=/dev/ttyACM0
SERIAL_PORT2=/dev/ttyACM1
BAUDRATE=115200
LOG_FILE=/var/log/navibox/system.log
GPS_READING_LOG=/var/log/navibox/gps_reading.log
GPS_DATA_LOG=/var/log/navibox/gps_data.log


Run the Setup Script:Execute the setup script as root to install dependencies and configure the system:
sudo ./setup.sh


The script installs system packages, creates a virtual environment, installs Python dependencies, and sets up log directories.
Optionally, it can configure a systemd service to run the GPS system automatically.


Manual Execution (if systemd is not used):Activate the virtual environment and run the Python script:
source /home/mdt/navibox/venv/bin/activate
python3 /home/mdt/navibox/main.py



Usage

Automatic Startup (with systemd):If the systemd service was set up, the system starts automatically on boot and runs as the mdt user. Check the service status:
sudo systemctl status navibox


Manual Startup:Run the Python script directly:
source /home/mdt/navibox/venv/bin/activate
python3 /home/mdt/navibox/main.py


Monitoring Logs:

System logs: /var/log/navibox/system.log
GPS reading logs: /var/log/navibox/gps_reading.log
WebSocket data logs: /var/log/navibox/gps_data.log



Output
The system outputs formatted GPS data to the console and WebSocket server every 5 seconds (configurable via SEND_INTERVAL). Example console output:
📡 GPS Data Transmission: 2025-07-13T14:09:23.123456+00:00
🏷️  Ship ID: your-device-id
🧭 Heading: 123.4°
============================================================
🟢 TOP_GPS:
  📍 Position: 37.12345678, -122.12345678
  🏔️  Altitude: 100.50m
  🚀 Speed: 10.25 km/h
  🛰️  Satellites: 12
  📊 PRNs: 1, 2, 3, 4, 5, 6, 7, 8, 9, 10
...

Configuration

u-blox ZED-F9P Setup:

Use u-blox u-center software to configure the module (e.g., enable RTK, set output protocols to NMEA/UBX/RTCM3).
Ensure active dual-band antennas are connected and placed away from noise sources.
Provide backup power (V_BCKP) to maintain RTC and satellite data.


Environment Variables:Modify the .env file to adjust serial ports, baudrate, or WebSocket URL as needed.


Troubleshooting

No GPS Data:
Verify serial port connections and permissions (ls -l /dev/ttyACM*).
Check antenna placement and ensure clear sky visibility.
Use u-center to confirm NMEA output from the ZED-F9P module.


WebSocket Connection Issues:
Ensure the WebSocket server is running and accessible.
Verify the WEBSOCKET_URL in the .env file.


Log Analysis:
Check system.log for system errors.
Check gps_reading.log for raw NMEA sentences.
Check gps_data.log for transmitted JSON data.



Contributing
Contributions are welcome! Please follow these steps:

Fork the repository.
Create a feature branch (git checkout -b feature/your-feature).
Commit your changes (git commit -m "Add your feature").
Push to the branch (git push origin feature/your-feature).
Open a pull request.



Built for the u-blox ZED-F9P-04B-01 GNSS module.
Uses the pynmea2 library for NMEA parsing and websocket-client for WebSocket communication.
Thanks to the open-source community for their contributions to the libraries used.

Contact
For support, contact the project maintainers via GitHub Issues or visit the u-blox Support Portal.
