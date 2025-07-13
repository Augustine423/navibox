<img width="640" height="640" alt="image" src="https://github.com/user-attachments/assets/b82e0f4d-65ec-4d40-a0c3-27d04e0312f8" />


Installation

Clone the Repository:
```
git clone https://github.com/Augustine423/navibox.git
```
cd navibox

u-blox ZED-F9P-04B-01 GNSS Module Documentation
________________________________________
Overview
The u-blox ZED-F9P-04B-01 is a high-precision, dual-band GNSS module designed for professional-grade positioning applications. It supports real-time kinematic (RTK) positioning and delivers centimeter-level accuracy using multi-constellation and multi-frequency tracking.
________________________________________
Key Features
•	Centimeter-Level Accuracy using RTK and GNSS corrections
•	Multi-Constellation Support: GPS, GLONASS, Galileo, BeiDou
•	Dual-Band Frequency Support: L1/L2
•	RTK Moving Base Capability: Supports both base and rover mobility
•	Concurrent Interface Availability:
o	UART (2x)
o	USB
o	SPI
o	I2C
•	Update Rate:
o	RTK Position: up to 18 Hz
o	Raw Data: up to 25 Hz
•	Power Supply: 2.7V to 3.6V
•	Power Consumption: ~75–130 mA
•	Form Factor: 17 x 22 x 2.4 mm LGA module
•	Operating Temperature: -40°C to +85°C
________________________________________
Technical Specifications
Parameter	Value
Convergence Time (RTK)	< 10 seconds
Horizontal Accuracy	0.01 m + 1 ppm
Vertical Accuracy	0.01 m + 1 ppm
Heading Accuracy	~0.3 – 0.4°
Sensitivity	-167 dBm (tracking)
Max Altitude	80 km
Max Velocity	500 m/s
Max Acceleration	4 g
Timepulse Accuracy	30 ns RMS
________________________________________
Interface Pinout Summary
Interface	Purpose
UART1	Main communication
UART2	RTCM corrections input
USB	Host interface
SPI	Alternate interface
I2C	Low-speed config
________________________________________
Software & Firmware
•	u-blox u-center: Windows software for configuration, firmware update, and monitoring
•	Firmware: Updatable via USB or UART using u-center
•	Protocols: Supports NMEA, UBX, RTCM3, SPARTN
________________________________________
Integration Guidelines
•	Use active dual-band GNSS antennas for optimal performance
•	Place module away from noise sources (e.g., switching regulators)
•	Provide backup power (V_BCKP) to maintain RTC and satellite data
________________________________________
Applications
•	Surveying and Mapping
•	Drones and Robotics
•	Precision Agriculture
•	Autonomous Navigation
•	Attitude Sensing
________________________________________
Resources
•	ZED-F9P-04B Datasheet (UBX-21044850)
•	Integration Manual (UBX-18010802)
•	u-center Software
•	Product Page
________________________________________
Contact
For more details, visit the official u-blox Support Portal.

