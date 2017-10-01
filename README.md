# ThermalDrone

![thermal-drone](https://raw.githubusercontent.com/ArathornII/ThermalDrone/master/thermal-drone.jpg)

## Table of Contents:
1. [Introduction](#10-introduction)
2. [Progress](#20-progress)
3. [Current Design](30-current-design)
4. [Obstacles](#40-obstacles)
5. [Improvements](#50-improvements)
6. [Setup](#60-setup)
7. [Resources](#70-resources)

## 1.0 Introduction

### 1.1 Questions & Goals

The questions fundemental to this project are as follows: How can we automate indoor thermal imaging? How can we predict thermodynamic changes to an enclosed space resulting from changes made to a building?

The ultimate purpose of this project is to create an UAV capable capable of collecting and analyzing large data sets. Specifcally, the drone should be able to take distance measurements in various directions for navigation and localization. Also, the drone must be able to capture ambient air-temperature readings and wall-temperature readings. Finally, the drone must collect visual images for 3D-modeling and thermal composite imaging.

Once all data collection tools are fully-functional, the UAV will collect differentiated data sets for wall temperatures and air temperature. Furthermore, data would be collected before and after thermal modifications were made to a room. This will allow a machine learning model to predict thermal changes to a room based on deliberate modification to the room's physical structure. These predictions would likely be displayed through a 3-D model of the room comprised of visual and thermal images collected by the UAV.


### 1.2 Applications

So far, there are two primary applications for this technology. First, the UAV could become an essential tool for improving the thermal efficiency of industrial-sized factories. Currently, companies typically heat or cool factories through large heating units which are bolted to the ceiling and simply expected to heat a specific surrounding volume. This is not a scientific approach and often results in significant air-temperature fluxuations throughout the factory. Our technology will assist companies in optimizing selection and placement of heating and cooling units in factories through data collection and predictive modeling.

This technology could also provide homeowners with an affordable tool to improve the thermal efficiency of their home. Similar to in a factory setting, the UAV would help homeowners place heating systems to optimize their coverage. Furthermore, systematic thermal imaging would easily detect individual weaknesses in thermal insultation.


### 1.3 Sponsorship & Contacts

This research project is conducted through the MIT Research Laboratory of Electronics' Grossman Group. Key contacts are below:

* Nicola Ferralis (Research Scientist, Grossman Group)
* Jacob Feldgoise (Undergraduate, Carnegie Mellon University): feldgoise@cmu.edu
* Adrian Butterworth (Undergraduate, Imperial College of London)



## 2.0 Progress

- [x] Select & order parts
- [x] Fly drone via python script from laptop
- [x] Fly drone via Raspberry Pi
- [x] Capture sonar data
- [x] Capture temperature sensor data
- [x] Record thermal images
- [x] Record visual images
- [ ] Implement thermal camera composite imaging
- [ ] Implement object avoidance
- [ ] Photograph room with thermal camera via drone
- [ ] Devise indoor localization system to identify location of images and sensor readings
- [ ] Stitch thermal images into a 3D model
- [ ] Implement machine learning to predict temperatures changes based on captured data sets



## 3.0 Current Design

### 3.1 Selecting the Drone

We considered five primary criteria when choosing which drone to purchase: assembly/configuration time, payload capacity, size, price, and programmability. Based on these criteria, we selected the 3DR Solo for this project. The 3DR Solo was the best choice mostly because it features a MAVlink Python library called "DroneKit-Python".

### 3.2 Communication



### 3.3 Multiprocessing





## 4.0 Obstacles

### 4.1 Upgrading 3DR Solo Firmware from 1.3.1 to 1.5.2

**DO NOT ATTEMPT THIS!** DroneKit-Python is not yet compatible with the latest firmware release, so programming the drone in 1.5.2 is effectively useless. Hopefully, compatibility issues will be resolved in the future.

### 4.2 Taking off in GPS-denied environment

#### 4.2.1 GUIDED_NOGPS Mode

#### 4.2.2 RC Overrides

## 5.0 Improvements

### 5.1 Custom Design

### 5.2 Wired MAVlink Connection (via Accessory Bay)

### 5.3 Optical Flow Rangefinder

### 5.4 Raspberry Pi 3B for Image Processing

## 6.0 Setup


### 6.1 Configuring the Raspberry Pi



### 6.2 Configuring your Computer


## 7.0 Resources


### 7.1 BME280 Temperature Sensor

[Using BME280 temperature/humidity/pressure sensor with Raspberry Pi](https://xdevs.com/guide/thp_rpi/)


### 7.2 Flight

[Altitude Hold Mode](http://ardupilot.org/copter/docs/altholdmode.html)
[Autopilot Firmware Upgrade from 1.3.1 to 1.5.2](https://3drpilots.com/threads/autopilot-firmware-upgrade-from-1-3-1-to-1-5-2.8819/)
[Copter Commands in Guided Mode](http://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html)
[Create setter functions rather than setting attributes directly](https://github.com/dronekit/dronekit-python/pull/566)
[Move/direct Copter and send commands in GUIDED_NOGPS mode using DroneKit Python](https://github.com/studroid/dronekit-python/blob/84b68270d31798568602f36d06ee8c85c2b100ce/examples/change_attribute/change_attitude.py)
[Guiding and Controlling Copter](http://python.dronekit.io/1.5.0/guide/copter/guided_mode.html)
[Indoor copter flight](https://github.com/dronekit/dronekit-python/issues/697)
[Indoor Flying Guidelines](http://ardupilot.org/copter/docs/indoor-flying.html)
[Quaternions](http://diydrones.com/profiles/blogs/post-5-maaxx-europe-quaternions-control-code-and-pids)
[QGroundControl User Guide](https://www.gitbook.com/book/donlakeflyer/qgroundcontrol-user-guide/details)
[3DR Solo Development Area](https://3drpilots.com/threads/solo-development-area.5062/)
[3DR Solo User Guide](https://3dr.com/wp-content/uploads/2016/01/v8_01_05_16.pdf)
[DroneKit-Python Documentation](http://python.dronekit.io)

#### 7.2.1 Communication between RPi and Drone

[API Documentation for PyMAVLink](https://www.samba.org/tridge/UAV/pymavlink/apidocs/index.html)
[Python MAVLink interface and utilities](https://github.com/ArduPilot/pymavlink)
[Communicating with Raspberry Pi via MAVLink](http://ardupilot.org/dev/docs/raspberry-pi-via-mavlink.html#communicating-with-raspberry-pi-via-mavlink)
[Companion Computers](http://python.dronekit.io/1.5.0/guide/companion-computers.html)
[Solo Development Guide](https://dev.3dr.com)
[Mavlink (developer page)](https://www.pixhawk.org/dev/mavlink?s[]=mavlink#mavlink_developer_page)
[MAVLINK Common Message Set](http://mavlink.org/messages/common)
[MAVLink Mission Command Messages (MAV_CMD)](http://ardupilot.org/copter/docs/common-mavlink-mission-command-messages-mav_cmd.html#mavlink-mission-command-messages-mav-cmd)
[MavLink Tutorial for Absolute Dummies (Part I)](http://api.ning.com/files/i*tFWQTF2R*7Mmw7hksAU-u9IABKNDO9apguOiSOCfvi2znk1tXhur0Bt00jTOldFvob-Sczg3*lDcgChG26QaHZpzEcISM5/MAVLINK_FOR_DUMMIESPart1_v.1.1.pdf)
[MAVlink Message Definitions](https://github.com/mavlink/mavlink/blob/master/message_definitions/v1.0/common.xml)
[MAVProxy Landing Page](http://qgroundcontrol.org/mavlink/mavproxy_startpage)
[MAVProxy Ardupilot Documentation](http://ardupilot.github.io/MAVProxy/html/index.html#mavproxy)
[MAVProxy Cheetsheet](http://ardupilot.github.io/MAVProxy/html/_static/files/MAVProxyCheetsheet.pdf)
[Operating Arducopter without GPS over MAVLink](http://diydrones.com/group/arducopterusergroup/forum/topics/operating-arducopter-without-gps-over-mavlink?page=1&commentId=705844%3AComment%3A1962580&x=1#705844Comment1962580)
[3DR Solo Breakout Board Specs](https://github.com/3drobotics/Pixhawk_OS_Hardware/tree/master/Accessory_Breakout_X1)
[Vehicle State and Settings](http://python.dronekit.io/guide/vehicle_state_and_parameters.html)
[Which MAVLink commands can fly a copter without GPS?](https://discuss.ardupilot.org/t/which-mavlink-commands-can-fly-a-copter-without-gps/9353/1)

### 7.3 Image Processing

[Basic Recipes](http://picamera.readthedocs.io/en/release-1.10/recipes1.html#basic-recipes)
[Accessing the Raspberry Pi Camera with OpenCV and Python](https://www.pyimagesearch.com/2015/03/30/accessing-the-raspberry-pi-camera-with-opencv-and-python/)
[applyColorMap for pseudocoloring in OpenCV ( C++ / Python )](https://www.learnopencv.com/applycolormap-for-pseudocoloring-in-opencv-c-python/)
[Camera Module](https://www.raspberrypi.org/documentation/hardware/camera/)
[FLIR Lepton Hookup Guide](https://learn.sparkfun.com/tutorials/flir-lepton-hookup-guide)
[Quick and dirty pure python library for interfacing with FLIR lepton](https://github.com/groupgets/pylepton)
[How to install/use the Raspberry Pi Camera](https://thepihut.com/blogs/raspberry-pi-tutorials/16021420-how-to-install-use-the-raspberry-pi-camera)
[Image size (Python, OpenCV)](https://stackoverflow.com/questions/13033278/image-size-python-opencv)
[OpenCV - Saving images to a particular folder of choice](https://stackoverflow.com/questions/41586429/opencv-saving-images-to-a-particular-folder-of-choice)
[python OpenCV - add alpha channel to RGB image](https://stackoverflow.com/questions/32290096/python-opencv-add-alpha-channel-to-rgb-image)

### 7.4 Data Logging

[File Handling Cheat Sheet in Python](http://www.pythonforbeginners.com/cheatsheet/python-file-handling)
[Reading and Writing Files in Python](http://www.pythonforbeginners.com/files/reading-and-writing-files-in-python)

### 7.5 Multiprocessing

[Multiprocessing — Process-based parallelism](https://docs.python.org/3/library/multiprocessing.html)
[An introduction to parallel programming using Python's multiprocessing module](http://sebastianraschka.com/Articles/2014_multiprocessing.html)
[Communication Between Processes](https://pymotw.com/2/multiprocessing/communication.html)
[Multiprocessing with Python](https://www.raspberrypi.org/magpi/multiprocessing-with-python/)
[Multiprocessing vs Threading Python](https://stackoverflow.com/questions/3044580/multiprocessing-vs-threading-python)
[What does if __name__ == “__main__”: do?](https://stackoverflow.com/questions/419163/what-does-if-name-main-do)
[Python 201: A multiprocessing tutorial](https://www.blog.pythonlibrary.org/2016/08/02/python-201-a-multiprocessing-tutorial/)

### 7.6 Object Avoidance and Rangefinding

[LIDAR-Lite Rangefinder](http://ardupilot.org/copter/docs/common-rangefinder-lidarlite.html#lidar-lite-rangefinder)
[Script to Read SR-04](https://github.com/feranick/Pi-bot/blob/master/Utilities/sonar/sonar_distance.py)
[PX4FLOW Optical Flow Camera Board Overview](http://ardupilot.org/copter/docs/common-px4flow-overview.html#px4flow-optical-flow-camera-board-overview)
[PX4FLOW V1.3.1 OPTICAL FLOW Camera](http://www.hobbywow.com/en-px4flow-v1-3-1-optical-flow-camera-and-mb1043-sonar-sensor-for-px4-pixhawk-pix-flight-control-p240168.htm)

### 7.7 Raspberry Pi

[Raspbian Installation](https://www.raspberrypi.org/downloads/raspbian/)
[Installing the SciPy Stack](https://www.scipy.org/install.html)
[Raspberry Pi Power Requirements](https://www.raspberrypi.org/help/faqs/#topPower)
[Raspberry Pi Pinout](https://pinout.xyz)







