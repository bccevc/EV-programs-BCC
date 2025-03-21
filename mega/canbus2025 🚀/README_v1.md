🚗 Electric Vehicle CAN Bus Data Logger

This project is a professional-grade CAN bus and GPS-based data logger designed for real-time electric vehicle (EV) diagnostics, efficiency tracking, and system monitoring. It is built using Arduino, SparkFun CAN Bus Shield, and GPS modules, and logs vehicle telemetry to both .txt and .csv formats on an SD card.

📌 Project Purpose

This logger is developed to replace and enhance a previously bulky and repetitive Arduino-based data logging system by:

Increasing logging efficiency.

Ensuring accurate and structured CSV data for analysis.

Displaying real-time stats on an LCD screen.

Creating unique log files for every drive session.

Supporting full decoding of CAN messages using real, documented CAN IDs from the Battery Management System (BMS).

🧠 Key Improvements Over Previous Version

✅ Code Structure & Efficiency

Modularized all logic: Extracted reusable functions (e.g., readCANValue(), readCAN16BitValue(), calcDist()).

Reduced repetition: Eliminated repeated blocks for file I/O and CAN parsing.

Readable format: Added detailed inline comments and standardized naming.

✅ Expanded Parameter Logging

We implemented all 3.4 required parameters and more, including:

GPS Date & Time

Arduino Timestamp (ms)

Discharge/Charger Enable Status

GPS Speed, Latitude, Longitude

Motor Controller Temp, Pack Voltage & Current

State of Charge (%), Cell Voltages & IDs

Thermistor Temp & ID, Balancing, Error Flags

Incremental & Cumulative Distance, Energy

kWh/Mile & MPGe (EPA standard, 33.7 kWh/gal)

Each parameter is scaled and formatted to the correct unit and precision, e.g.:

Voltage: xxx.x V

Current: xx.x A

Distance: 0.0000 miles

Energy: 0.000000 kWh

MPGe: 000.00

🗃️ CSV & TXT Logging

📝 Raw Text Logging

Saved as log_MMDDYYYY_HHMM.txt

Human-readable logs including GPS, voltage, current, SoC, MPGe

Helpful for debugging or quick reference

📈 CSV Logging

Saved as log_MMDDYYYY_HHMM.csv

Structured table for importing into Excel, MATLAB, Python

CSV header includes all parameter labels (in correct order)

A new file is generated for every drive session using GPS date/time as filename

🔁 How It Works

GPS Module: Used to extract real-world location, speed, and time.

CAN Bus Shield: Communicates with the vehicle’s Battery Management System (BMS).

CAN IDs: Decoded with precise scaling to extract real values (e.g.:

0x03B for voltage and current,

0x1A0 for SoC,

0x1B0 and 0x1B1 for min/max cell voltages).

LCD Display: Shows values in real-time (e.g., Speed, SoC, Power, Distance, MPGe).

File Naming: Uses gps.crack_datetime() to extract the real timestamp and create unique filenames.

Loop Execution: Runs every 1 second to capture accurate incremental values (distance, energy, etc.).

🛠️ Key Functions Explained

readCANValue(canID, byteIndex)

Reads a single byte from a CAN message.

Used for flags or percent values (e.g. SoC, Enable flags).

readCAN16BitValue(canID, startIndex, scale)

Reads two bytes, combines them to a 16-bit integer, and scales it.

Used for precise parameters (voltage, current, temperature).

calcDist(lat1, lon1, lat2, lon2)

Implements the Haversine formula to calculate distance in miles between two GPS points.

Used for both incremental and cumulative distance.

🔒 Data Accuracy & Sync

All parameters are logged with the same timestamp.

CAN data is fetched only when messages are available.

File writes are flushed on every loop to prevent data loss.

GPS must be locked before starting (ensures time/date accuracy).

📝 Sample Output: log_03252025_1410.txt

Human-readable log (every 1 second)


[Time: 14:10:03] [Date: 3/25/2025] Speed: 34.28 mph, Lat: 40.900123, Lon: -74.213400, V: 322.5 V, I: 45.2 A, SoC: 78.3% , MPGe: 179.42
[Time: 14:10:04] [Date: 3/25/2025] Speed: 34.89 mph, Lat: 40.900145, Lon: -74.213410, V: 322.8 V, I: 44.8 A, SoC: 78.0% , MPGe: 178.90
[Time: 14:10:05] [Date: 3/25/2025] Speed: 35.25 mph, Lat: 40.900168, Lon: -74.213425, V: 323.1 V, I: 45.0 A, SoC: 77.7% , MPGe: 178.43
📈 Sample Output: log_03252025_1410.csv

Comma-separated values for Excel, Python, MATLAB, etc.


GPS Date,GPS Time,Timestamp(ms),Discharge Enable,Charger Safe,GPS Speed (mph),Latitude,Longitude,MC Temp (°C),Pack Voltage (V),Pack Current (A),State of Charge (%),Lowest Cell V (V),Lowest Cell ID,Highest Cell V (V),Highest Cell ID,Balancing Active,Highest Therm Temp (°C),Thermistor ID,Error Flags,Time Increment (s),Distance Increment (mi),Energy Increment (kWh),Cumulative Distance (mi),Cumulative Energy (kWh),kWh per Mile,MPGe
3/25/2025,14:10:03,120000,1,1,34.28,40.900123,-74.213400,38.1,322.5,45.2,78.3,3.621,6,4.195,12,0,41.2,3,0,1,0.0023,0.00015,1.2310,0.2312,0.1878,179.42
3/25/2025,14:10:04,121000,1,1,34.89,40.900145,-74.213410,38.3,322.8,44.8,78.0,3.622,6,4.194,12,0,41.1,3,0,1,0.0024,0.00014,1.2334,0.2313,0.1875,179.28
3/25/2025,14:10:05,122000,1,1,35.25,40.900168,-74.213425,38.5,323.1,45.0,77.7,3.623,6,4.193,12,0,41.0,3,0,1,0.0025,0.00015,1.2359,0.2315,0.1873,179.15


📺 Sample LCD Display (20x4 Characters)

Rotating or fixed fields shown while driving
Screen 1: General Status

Speed:  35.2 MPH     SoC: 78.3%
Trip:   1.23 mi      MPGe: 179.4
V: 322.5V   I: 45.2A
Energy: 0.231 kWh
Screen 2: Cell & Temp Stats

Lo Cell: 3.621V ID:6
Hi Cell: 4.195V ID:12
Therm: 41.2°C ID:3
Bal: OFF  Err: 0


⚡ Technologies Used

Arduino Uno or Mega

SparkFun CAN Bus Shield (MCP2515 based)

TinyGPS & NEO-6M GPS Module

SerLCD over I2C

SanDisk microSD 16GB+ with SPI Adapter

📦 Future Improvements

Add graphical interface (Python/React) to visualize CSV logs

Add real-time WiFi sync (e.g., ESP32-based wireless logger)

Implement error alerts on LCD (e.g., high cell temp warning)

Optimize for battery buffering & low-power sleep modes

🙌 Credits

Developed with support from:

Professor Robert Roller

Bergen Community College STEM Center

HackBCC 2025 Smart Transportation Track

EV Conversion Team (Hardware & Integration) 

Code design and documentation by Omer Kilic 💡⚙️

💬 Contact

Feel free to reach out for collaboration, enhancements, or open-source deployment:
📧 okilic@me.bergen.edu

Let’s build the future of smart vehicles — one byte at a time.
