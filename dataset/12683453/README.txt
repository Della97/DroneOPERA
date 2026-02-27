This README.txt file was generated on 20201223 by Thiago Rodrigues
and contains the most recent instructions. 
The most recent associated data posted are the data that should be used for this project.  

-------------------
GENERAL INFORMATION
-------------------

1. Title of Dataset: Data Collected with Package Delivery Quadcopter Drone

#
# Authors: Include contact information for at least the 
# first author and corresponding author (if not the same), 
# specifically email address, phone number (optional, but preferred), and institution. 
# Contact information for all authors is preferred.
#

2. Author Information
<create a new entry for each additional author>

First Author Contact Information
    Name: Thiago A. Rodrigues
    Institution: Carnegie Mellon University 
    Address: 5000 Forbes Avenue, Pittsburgh, Pennsylvania, USA, 15213
    Email: tarodrig@andrew.cmu.edu
	Phone Number: 412-251-4948  

Author Contact Information
    Name: Jay Patrikar
    Institution: Carnegie Mellon University 
    Address: 5000 Forbes Avenue, Pittsburgh, Pennsylvania, USA, 15213
    Email: jpatrika@andrew.cmu.edu
	Phone Number:

Author Contact Information
    Name: Arnav Choudhry
    Institution: Carnegie Mellon University 
    Address: 5000 Forbes Avenue, Pittsburgh, Pennsylvania, USA, 15213
    Email: achoudhr@andrew.cmu.edu
	Phone Number:

Author Contact Information
    Name: Jacob Feldgoise
    Institution: Carnegie Mellon University 
    Address: 5000 Forbes Avenue, Pittsburgh, Pennsylvania, USA, 15213
    Email: jfeldgoi@andrew.cmu.edu
	Phone Number:

Author Contact Information
    Name: Vaibhav Arcot
    Institution: University of Pennsylvania 
    Address: 3330 Walnut Street, Philadelphia, Pennsylvania, USA, 19104
    Email: yvarcot@seas.upenn.edu
	Phone Number:

Author Contact Information
    Name: Aradhana Gahlaut
    Institution: Carnegie Mellon University 
    Address: 5000 Forbes Avenue, Pittsburgh, Pennsylvania, USA, 15213
    Email: agahlaut@andrew.cmu.edu
	Phone Number:

Author Contact Information
    Name: Sophia Lau
    Institution: Carnegie Mellon University 
    Address: 5000 Forbes Avenue, Pittsburgh, Pennsylvania, USA, 15213
    Email: sjlau@andrew.cmu.edu
	Phone Number:

Author Contact Information
    Name: Brady Moon
    Institution: Carnegie Mellon University 
    Address: 5000 Forbes Avenue, Pittsburgh, Pennsylvania, USA, 15213
    Email: bradym@andrew.cmu.edu
	Phone Number:

Author Contact Information
    Name: Bastian Wagner
    Institution: Baden-Wuerttemberg Cooperative State University (DHBW), Germany
    Address: Stuttgart, Baden-Württemberg, Germany
    Email: bastianw@andrew.cmu.edu
	Phone Number:

Author Contact Information
    Name: H. Scott Matthews 
    Institution: Carnegie Mellon University 
    Address: 5000 Forbes Avenue, Pittsburgh, Pennsylvania, USA, 15213
    Email: hsm@cmu.edu
	Phone Number:

Author Contact Information
    Name: Sebastian Scherer
    Institution: Carnegie Mellon University 
    Address: 5000 Forbes Avenue, Pittsburgh, Pennsylvania, USA, 15213
    Email: basti@andrew.cmu.edu
	Phone Number:

Author Contact Information
    Name: Constantine Samaras
    Institution: Carnegie Mellon University 
    Address: 5000 Forbes Avenue, Pittsburgh, Pennsylvania, USA, 15213
    Email: csamaras@cmu.edu
	Phone Number:

---------------------
DATA & FILE OVERVIEW
---------------------

#
# Directory of Files in Dataset: List and define the different 
# files included in the dataset. This serves as its table of 
# contents. 
#

Directory of Files:
   A. Filename: parameters.csv
      Short description: This file contains a list with all flights and the flight parameters.

   B. Filename: flights.zip
      Short description: This file contains a csv file for each flight, with information on wind, state, and battery current and voltage.  

   C. Filename: flights.csv
      Short description: This file combines "paramters.csv" and all data from "flights.zip" in one csv file.     

   D. Filename: raw_files.zip        
      Short description: This file contains the raw data collected by each sensor for each flight.   
    


Additional Notes on File Relationships, Context, or Content 
(for example, if a user wants to reuse and/or cite your data, 
what information would you want them to know?): 

The data contained in this CSV was simultaneously collected during 209 flight campaigns from the following on-board sensors: Wind sensor: FT Technologies FT205 UAV-mountable, pre-calibrated ultrasonic wind sensor with accuracy of ± 0.1 m/s and refresh rate of 10 Hz; Position: 3DM-GX5-45 GNSS/INS sensor pack. These sensors use a built-in Kalman filtering system to fuse the GPS and IMU data. The sensor has a maximum output rate of 10Hz with accuracy of ± 2 m$ RMS horizontal, ± 5 m$ RMS vertical; Current and Voltage: Mauch Electronics PL-200 sensor. This sensor can record currents up to 200 A and voltages up to 33 V. Analogue readings from the sensor were converted into a digital format using an 8 channel 17 bit analogue-to-digital converter (ADC). Data syncing and recording was handled using the Robot Operating System (ROS) running on a low-power Raspberry Pi Zero W. Data was recorded on the Raspberry Pi's microSD card. The data provided by each sensor were synchronized to a frequency of approximately 5Hz using the ApproximateTime message filter policy of Robot Operating System (ROS).

raw_files.zip was added on 20200921 and correspond to version 2 of the record.
The raw files are v2.0 ROSBag files with .bag extensions.
Each .bag has data from 4 topics as lists below:

anemometer
This topic is of the type anemometer/Anemometer. This a custom message type to record info from the anemometer. The topic is published by the anememeter ROS driver. The details in the ROS message are:
```
speed: Magnitude of wind as measured by the anemometer (m/s)
angle: Magnetic angle of wind as measured by the anemometer 
```
To see the message please install the msg topic from the  anemometer code repo.
battery
This topic is of the type sensor_msgs/BatteryState. This is a standard meassage in ROS. The topic is published by the ADC driver in ROS. We populate the following fields:
```

voltage: Voltage in Volts
current: Negative when discharging (A)  
```
imu/data
This topic is of the type sensor_msgs/Imu. This is a standard meassage in ROS. The topic is published by the  microstrain_3dm_gx5_45 driver in ROS.
nav/odom

This topic is of the type nav_msgs/Odometry. This is a standard meassage in ROS. The topic is published by the  microstrain_3dm_gx5_45 driver in ROS.

### Typical commands and output:
#### To see info on a rosbag file:
Needs a ROS installation on the system.
##### Command:
` rosbag info raw.bag `
##### Output:
path:        raw.bag
version:     2.0
duration:    2:02s (122s)
start:       Apr 02 2019 19:23:35.98 (1554247415.98)
end:         Apr 02 2019 19:25:38.47 (1554247538.47)
size:        1.6 MB
messages:    4498
compression: none [2/2 chunks]
types:       
anemometer/Anemometer[22baa219c9d286acdbc33d44de439f38]
nav_msgs/Odometry        [cd5e73d190d741a2f92e81eda573aca7]
sensor_msgs/BatteryState [476f837fa6771f6e16e3bf4ef96f8770]
sensor_msgs/Imu          [6a62c6daae103f4ff57a132d6f95cec2]
topics:  
 /anemometer    820 msgs    : anemometer/Anemometer   
/battery      1228 msgs    : sensor_msgs/BatteryState
/imu/data     1226 msgs    : sensor_msgs/Imu         
/nav/odom     1224 msgs    : nav_msgs/Odometry 
#### To play the rosbag file:
Needs a ROS installation on the system.
##### Command:
` rosbag play raw.bag `
##### Output:
[ INFO] [1600664147.105012828]: Opening raw.bag
Waiting 0.2 seconds after advertising topics... done.
Hit space to toggle paused, or 's' to step.
[DELAYED]  Bag Time: 1554247415.982276   Duration: 0.000000 / 122.
[RUNNING]  Bag Time: 1554247415.982276   Duration: 0.000000 / 122. 
[RUNNING]  Bag Time: 1554247415.982276   Duration: 0.000000 / 122. 
[RUNNING]  Bag Time: 1554247415.982521   Duration: 0.000245 / 122. 
[RUNNING]  Bag Time: 1554247415.982647   Duration: 0.000371 / 122.  

            
#
# File Naming Convention: Define your File Naming Convention 
# (FNC), the framework used for naming your files systematically 
# to describe what they contain, which could be combined with the
# Directory of Files. 
#

File Naming Convention: raw_files.zip contains numbered folders that correspond to the flight code. Inside of each folder there is a file named raw.bag with the raw data of that flight. 

File Naming Convention: The file flights.zip contains numbered csv files that correspond to the flight code. 


#
# Data Description: A data description, dictionary, or codebook
# defines the variables and abbreviations used in a dataset. This
# information can be included in the README file, in a separate 
# file, or as part of the data file. If it is in a separate file
# or in the data file, explain where this information is located
# and ensure that it is accessible without specialized software.
# (We recommend using plain text files or tabular plain text CSV
# files exported from spreadsheet software.) 
#

-----------------------------------------
DATA DESCRIPTION FOR: [parameters.csv]
-----------------------------------------
<create sections for each dataset included>


1. Number of variables: 7


2. Number of cases/rows: 209 


3. Missing data codes: The dataset has no missing data, but in the case of missing codes, the dataset would use "NA" to denote missing data.


4. Variable List

    A. Name: flight
       Description: an integer that represents the code of the flight performed. A flight is defined as the data set recorded from the take-off to landing in a predefined route.

    B. Name: speed
       Description: programmed horizontal ground speed during cruise in meters per second (m/s).

    C. Name: payload
       Description: mass of the payload attached to aircraft in grams (g). The payload used was confined in a standard USPS Small Flat Rate Box.

    D. Name: altitude
       Description: predefined altitude in meters (m). The aircraft takes off vertically until it reaches the preset altitude.

    E. Name: date
       Description: when the flight was conducted in the YYYY-MM-DD format

    F. Name: local_time
       Description: local time when the flight started in the 24:00-hour format.

    G. Name: route
       Description: A predefined path followed by the aircraft. Routes R1 to R7 indicate flights where the drone completed a cruise movement. The differences among routes R1 to R7 reflect variations on the starting point or variations on the altitude during cruise. The differences among routes can be assessed by plotting variables position_x, position_y and position_z. Routes A refer to ancillary ground tests where the drone remained on the ground and did not take off. Route A1 refers to a test with the drone running with no propellers and no motor movement; Route A2 to a test with the drone running with no propellers and minimum motor movement; and Route A3 to a test with the drone running with propellers and minimum motor movement. Route H refers to a test with the drone hovering with no horizonal movement. In summary:   
	R1 to R7 = full flights completing a cruise movement;
	A1 = Ancillary ground test with no propellers and no motor movement;
	A2 = Ancillary ground test with no propellers and minimum movement; 
	A3 = Ancillary ground test with propellers and minimum movement;
	H = Hover test with no horizontal movement.  

-----------------------------------------
DATA DESCRIPTION FOR: [flights.zip]
-----------------------------------------

1. Number of variables: Each csv file within has 21 variables


2. Number of cases/rows: The number of rows varies according to the duration of the flight.

+----------+------+----------+------+----------+------+----------+------+----------+------+
| flight # | rows | flight # | rows | flight # | rows | flight # | rows | flight # | rows |
+----------+------+----------+------+----------+------+----------+------+----------+------+
| 1        | 1339 | 101      | 1098 | 148      | 2027 | 192      | 1060 | 236      | 1036 |
| 2        | 1809 | 102      | 1688 | 149      | 1158 | 193      | 1453 | 237      | 1897 |
| 3        | 1201 | 105      | 1477 | 150      | 1215 | 194      | 1366 | 238      | 1444 |
| 4        | 1141 | 106      | 1299 | 151      | 999  | 195      | 1394 | 239      | 1674 |
| 5        | 1445 | 107      | 1300 | 152      | 1290 | 196      | 1775 | 240      | 1338 |
| 6        | 1364 | 108      | 1073 | 153      | 1257 | 197      | 916  | 241      | 1152 |
| 7        | 1197 | 109      | 1691 | 154      | 1180 | 198      | 1020 | 242      | 1765 |
| 8        | 1558 | 110      | 1628 | 155      | 1256 | 199      | 1550 | 243      | 1619 |
| 10       | 922  | 111      | 906  | 156      | 1447 | 200      | 1312 | 246      | 1366 |
| 12       | 775  | 112      | 1342 | 157      | 1005 | 201      | 1563 | 247      | 1245 |
| 14       | 708  | 113      | 1331 | 158      | 1127 | 202      | 1427 | 248      | 1593 |
| 15       | 756  | 114      | 1176 | 159      | 1068 | 203      | 951  | 249      | 1103 |
| 16       | 840  | 115      | 994  | 160      | 1453 | 204      | 1112 | 250      | 1101 |
| 17       | 928  | 116      | 902  | 162      | 1206 | 205      | 1079 | 251      | 1177 |
| 18       | 1135 | 117      | 1073 | 163      | 1360 | 206      | 1239 | 252      | 1286 |
| 20       | 934  | 118      | 1704 | 164      | 1287 | 207      | 1102 | 253      | 1021 |
| 23       | 848  | 119      | 1333 | 165      | 1374 | 208      | 1320 | 254      | 1103 |
| 59       | 941  | 120      | 921  | 166      | 1281 | 209      | 1169 | 255      | 1102 |
| 60       | 1003 | 121      | 1220 | 167      | 951  | 210      | 1677 | 256      | 1150 |
| 68       | 1144 | 122      | 1163 | 168      | 1206 | 211      | 881  | 257      | 1401 |
| 76       | 1622 | 123      | 1428 | 169      | 1222 | 212      | 805  | 258      | 1462 |
| 77       | 909  | 124      | 996  | 170      | 1100 | 213      | 392  | 260      | 1632 |
| 78       | 1257 | 125      | 1523 | 171      | 882  | 214      | 910  | 261      | 1287 |
| 79       | 1043 | 126      | 1719 | 172      | 1476 | 215      | 391  | 262      | 1149 |
| 80       | 1181 | 127      | 1086 | 173      | 1577 | 216      | 392  | 263      | 1590 |
| 81       | 1194 | 128      | 1363 | 174      | 1146 | 217      | 935  | 264      | 1515 |
| 82       | 1218 | 129      | 1248 | 175      | 1304 | 218      | 953  | 267      | 1009 |
| 83       | 1191 | 130      | 1038 | 176      | 1357 | 219      | 842  | 268      | 1511 |
| 84       | 1517 | 131      | 1644 | 177      | 929  | 221      | 809  | 269      | 1317 |
| 85       | 1514 | 134      | 1074 | 178      | 925  | 222      | 950  | 270      | 1466 |
| 86       | 1218 | 135      | 1309 | 179      | 1413 | 223      | 857  | 271      | 1219 |
| 87       | 1671 | 136      | 1183 | 180      | 1148 | 224      | 1178 | 272      | 2856 |
| 88       | 1369 | 137      | 1315 | 181      | 1172 | 225      | 1163 | 275      | 983  |
| 91       | 1185 | 138      | 1339 | 182      | 1207 | 226      | 1251 | 276      | 984  |
| 92       | 1174 | 139      | 1045 | 183      | 907  | 227      | 1443 | 277      | 860  |
| 93       | 1214 | 140      | 1320 | 184      | 1668 | 228      | 1271 | 278      | 1197 |
| 94       | 1461 | 141      | 1574 | 185      | 1269 | 229      | 1053 | 279      | 988  |
| 95       | 1066 | 142      | 1185 | 186      | 1128 | 230      | 1098 |          |      |
| 96       | 1341 | 143      | 1120 | 187      | 920  | 231      | 1325 |          |      |
| 97       | 1577 | 144      | 1158 | 188      | 2042 | 232      | 1336 |          |      |
| 98       | 871  | 145      | 1470 | 189      | 988  | 233      | 1446 |          |      |
| 99       | 1274 | 146      | 1218 | 190      | 1312 | 234      | 1263 |          |      |
| 100      | 1042 | 147      | 1629 | 191      | 1287 | 235      | 943  |          |      |
+----------+------+----------+------+----------+------+----------+------+----------+------+

3. Missing data codes: The dataset has no missing data, but in the case of missing codes, the dataset would use "NA" to denote missing data.


4. Variable List

    A. Name: time
       Description: Time elapsed in flight in seconds (s).

    B. Name: wind_speed
       Description: airspeed provided by the anemometer in meters per second (m/s).

    C. Name: wind_angle
       Description: angle in degrees (deg) of the air flowing through the anemometer with respect to the north.

    D. Name: battery_voltage
       Description: system voltage in Volts (V) measured immediately after the battery.

    E. Name: battery_current
       Description: system current in Ampere (A) measured immediately after the battery.

    F. Name: position_x
       Description: longitude of the aircraft in degrees (deg).

    G. Name: position_y
       Description: latitude of the aircraft in degrees (deg).

    H. Name: position_z
       Description: altitude of the aircraft in meters (m) with respect to the sea-level.

    I. Name: orientation_x; _y; _z; _w
       Description: aircraft orientation in quaternions.

    J. Name: velocity_x; _y; _z
       Description: velocity components of ground speed in meters per second (m/s).

    K. Name: angular_x; _y; _z
       Description: angular volocity components in radians per second (rad/s).

    L. Name: linear_acceleration_x; _y; _z
       Description: ground acceleration in meters per squared second (m/s^2).
   
   
-----------------------------------------
DATA DESCRIPTION FOR: [flights.csv]
-----------------------------------------
This file combines the data avaliable in flights.zip and parameters.csv in a single CSV file. 

1. Number of variables: 28


2. Number of cases/rows: 257,896.


3. Missing data codes: The dataset has no missing data, but in the case of missing codes, the dataset would use "NA" to denote missing data.


4. Variable List
    A. Name: flight
       Description: an integer that represents the code of the flight performed. A flight is defined as the data set recorded from the take-off to landing in a predefined route.

    B. Name: time
       Description: Time elapsed in flight in seconds (s).

    C. Name: wind_speed
       Description: airspeed provided by the anemometer in meters per second (m/s).

    D. Name: wind_angle
       Description: angle in degrees (deg) of the air flowing through the anemometer with respect to the north.

    E. Name: battery_voltage
       Description: system voltage in Volts (V) measured immediately after the battery.

    F. Name: battery_current
       Description: system current in Ampere (A) measured immediately after the battery.

    G. Name: position_x
       Description: longitude of the aircraft in degrees (deg).

    H. Name: position_y
       Description: latitude of the aircraft in degrees (deg).

    I. Name: position_z
       Description: altitude of the aircraft in meters (m) with respect to the sea-level.

    J. Name: orientation_x; _y; _z; _w
       Description: aircraft orientation in quaternions.

    K. Name: velocity_x; _y; _z
       Description: velocity components of ground speed in meters per second (m/s).

    L. Name: angular_x; _y; _z
       Description: angular volocity components in radians per second (rad/s).

    M. Name: linear_acceleration_x; _y; _z
       Description: ground acceleration in meters per squared second (m/s^2).

    N. Name: speed
       Description: programmed horizontal ground speed during cruise in meters per second (m/s).

    O. Name: payload
       Description: mass of the payload attached to aircraft in grams (g). The payload used was confined in a standard USPS Small Flat Rate Box.

    P. Name: altitude
       Description: predefined altitude in meters (m). The aircraft takes off vertically until it reaches the preset altitude.

    Q. Name: date
       Description: when the flight was conducted in the YYYY-MM-DD format

    R. Name: local_time
       Description: local time when the flight started in the 24:00-hour format.

    S. Name: route
       Description: A predefined path followed by the aircraft. Routes R1 to R7 indicate flights where the drone completed a cruise movement. The differences among routes R1 to R7 reflect variations on the starting point or variations on the altitude during cruise. The differences among routes can be assessed by plotting variables position_x, position_y and position_z. Routes A refer to ancillary ground tests where the drone remained on the ground and did not take off. Route A1 refers to a test with the drone running with no propellers and no motor movement; Route A2 to a test with the drone running with no propellers and minimum motor movement; and Route A3 to a test with the drone running with propellers and minimum motor movement. Route H refers to a test with the drone hovering with no horizonal movement. In summary:   
	R1 to R7 = full flights completing a cruise movement;
	A1 = Ancillary ground test with no propellers and no motor movement;
	A2 = Ancillary ground test with no propellers and minimum movement; 
	A3 = Ancillary ground test with propellers and minimum movement;
	H = Hover test with no horizontal movement.      

--------------------------
METHODOLOGICAL INFORMATION
--------------------------
#
# Software: If specialized software(s) generated your data or
# are necessary to interpret it, please provide for each (if
# applicable): software name, version, system requirements,
# and developer. 
#If you developed the software, please provide (if applicable): 
#A copy of the software’s binary executable compatible with the system requirements described above. 
#A source snapshot or distribution if the source code is not stored in a publicly available online repository.
#All software source components, including pointers to source(s) for third-party components (if any)

1. Software-specific information: 
<create a new entry for each qualifying software program>

Name: N/A
Version: N/A
System Requirements: N/A
Open Source? (Y/N): N/A

(if available and applicable)
Executable URL: N/A
Source Repository URL: https://bitbucket.org/castacks/workspace/projects/DOE
Developer: N/A
Product URL: N/A
Software source components: N/A


Additional Notes(such as, will this software not run on 
certain operating systems?): N/A


#
# Equipment: If specialized equipment generated your data,
# please provide for each (if applicable): equipment name,
# manufacturer, model, and calibration information. Be sure
# to include specialized file format information in the data
# dictionary.
#

2. Equipment-specific information:
<create a new entry for each qualifying piece of equipment>

Manufacturer: DJI
Model: Matrice 100 

(if applicable)
Embedded Software / Firmware Name:
Embedded Software / Firmware Version:
Additional Notes: We use the DJI Matrice 100 quadrotor platform to represent multirotor UAVs. The Matrice 100 is a fully programmable and customizable UAS with a max flight speed of 17 m/s (in GPS mode) and a max takeoff weight of 3600 grams. The system has an on-board autopilot that provides autonomous capabilities. Its standard battery has a capacity of 4500 mAh which gives it a flight time of 22 minutes without any additional payload.

Manufacturer: FT Technologies
Model: FT205 UAV-mountable, pre-calibrated ultrasonic wind sensor.

(if applicable)
Embedded Software / Firmware Name: N/A 
Embedded Software / Firmware Version: N/A
Additional Notes: The sensor is accurate up to $\pm 0.1 m/s$ and has a refresh rate of 10Hz. We use the device's built-in filtering process to obtain reliable data. UART communication is used to record the data from the sensor.

Manufacturer: MICROSTRAIN
Model: 3DM-GX5-45 GNSS/INS sensor pack.

(if applicable)
Embedded Software / Firmware Name: N/A
Embedded Software / Firmware Version: N/A
Additional Notes: These sensors use a built-in Kalman filtering system to fuse the GPS and IMU data. The sensor has a maximum output rate of 10Hz. The sensor records data in N(North)-E(East)-D(Down) frame fixed at takeoff point.

Manufacturer: Mauch Electronics
Model: PL-200 sensor

(if applicable)
Embedded Software / Firmware Name: N/A
Embedded Software / Firmware Version: N/A
Additional Notes: This sensor is based on the Allegra ACS758-200U hall current sensor, and can record currents up to 200A and voltages up to 33V. The sensor board is only installed into the "positive" (red) main wire from the LiPo; the "negative "black" wire stays untouched, which reduces the risk the sensor board might short circuit. Analogue readings from the sensor are converted into a digital format using a 8 channel 17 bit analogue-to-digital converter (ADC). The ADC is based on the MCP3424 from Microchip Technologies Inc and is a delta-sigma A/D converter with low noise differential inputs.


#
# Dates of Data Collection: List the dates and/or times of
# data collection.
#

3. Date of data collection (single date, range, approximate date): 20190704 - 20191024

