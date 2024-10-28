# Georgia Institute of Technology
Fall 2024 Capstone Design Project

S31-Autonomous GT Campus Navigation Drone

<br>To experiment the drone system within the Gazebo environment, please refer to [Drone_Sim](https://github.com/jaep99/Drone_Sim).

# Raspberry Pi 5 to Pixhawk Connection Setup

## Installation to Raspberry Pi 5

    sudo apt-get update
    sudo apt-get upgrade
    sudo apt-get install python3-pip
    sudo apt-get install python3-dev
    sudo apt-get install python3-future
    sudo apt-get install screen 
    sudo apt-get install libwxgtk3.2-dev
    sudo apt-get install libxml2 libxslt1-dev
    sudo pip install pyserial
    sudo pip install dronekit
    sudo pip install MAVProxy

## Unable to Locate Package
Unable to locate package suggests that the specific packages might not be available in the repositories for Raspberry Pi 5 setup. 
Ensure that package list is up to date.

    sudo apt-get update

Also add missing libraries and/ or repositories, if needed

    sudo apt-get install libgtk-3-dev
    sudo apt-get install libwxgtk-media3.2-1 libwxgtk-webview3.2-1

Update after making new changes

    sudo apt-get update

Verify Installations

    <package> --version
    dpkg -l | grep <package>
    

## Externally-Mannaged-Enviroment Error
Externally managed enviroment occurs due to changes in how Python packages are managed in Linux distributions. These will typically happen when pip installations are restricted in some enviroments.

To override use:

    sudo pip install <package> --break-system-packages

However, using Python Virtual Environment is RECOMMENDED.
This isolates Python environment from the system and allows installing packages without affecting system level packages.

Instal virtualenv

    sudo apt-get install python3-virtualenv
    
Create a virtual environment

    virtualenv <virtualEnvironmentName>

Activate virtual environment

    source <virtualEnvironmentName>/bin/activate

Install packages inside virtual enviroment (e.g. pyserial)

    pip install <package>
    pip install pyserial

Deactivate virtual environment

    deactivate

## Set up RPI for UART Communication
Go into Raspberry Pi Software Configuration Tool

    sudo raspi-config

Interface Options > Serial Port

"Would you like a login shell to be accessible over serial?"

  \> \<No\>
  
"Would you like the serial port hardware to be enabled?"

  \> \<Yes\>

### Disable Bluetooth

    sudo nano /boot/config.txt

Make sure to navigate to the right path to config.txt (e.g. if config.txt is in firmware)

    sudo nano /boot/firmware/config.txt

Scroll down to [all]

comment out what is already under [all] and add:

    enable_uart=1
    dtoverlay=disable-bt

Save and Exit

# Run Pixhawk With Pi

    mavproxy.py --master=/dev/ttyAMA0

# MAVProxy Command Reference

| Command                   | Description                                                      |
|----------------------------|------------------------------------------------------------------|
| `rc N PWM`                 | Set RC channel N override to PWM (PMW =0 disables override)      |
| `link list`                | List all links                                                  |
| `link set N`               | Set link N to primary                                            |
| `link add X`               | Add new link                                                    |
| `link remove N`            | Remove link                                                     |
| `wp list`                  | Download and show waypoints                                     |
| `fence list`               | Download and show geofence                                      |
| `fence enable`             | Enable geofence                                                 |
| `fence disable`            | Disable geofence                                                |
| `arm throttle`             | Arm vehicle                                                     |
| `disarm`                   | Disarm vehicle                                                  |
| `disarm force`             | Force vehicle disarm (heli)                                     |
| `auto`                     | Engage auto mode                                                |
| `wp set N`                 | Set current auto waypoint to N                                  |
| `setspeed N`               | Override auto speed to N m/s                                    |
| `param show X`             | Show current value of parameter X                               |
| `param set X N`            | Set parameter X to value N                                      |
| `param download`           | Download parameter definitions from ArduPilot website           |
| `param help X`             | Display definition of parameter X                               |
| `mode loiter`              | Engage loiter mode                                              |
| `mode rtl`                 | Engage RTL mode                                                 |
| `mode manual`              | Engage manual mode                                              |
| `mode guided LAT LON ALT`  | Engage guided mode and move to specified location               |
| `relay set N [0\|1]`       | Set relay N to open or closed                                   |
| `servo set N PWM`          | Override servo N to PWM value                                   |
| `terrain check LAT LON`    | Get terrain height at specified location                        |
| `formatsdcard`             | Formats SD card                                                 |


# AdHoc Network Setup

# SSH Setup to Drone (Raspberry PI)


