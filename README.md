# ThermalDrone

![thermal-drone](https://raw.githubusercontent.com/ArathornII/ThermalDrone/master/thermal-drone.jpg)

## Table of Contents:
1. [Introduction](#10-introduction)
2. [Progress](#20-progress)
3. [Current Design](#30-current-design)
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

### 3.2 Onboard Computer

  We used the Raspberry Pi Zero W as the onboard computer for this project. It is extremely light but still has the 40 GPIO pins of a full-sized Raspberry Pi. Also, the "W" model comes with intergrated wifi and bluetooth which is exteremly useful. Finally, the price of a Raspberry Pi Zero W is only $10. The only downside is that the Raspberry Pi Zero models lacks the processing power of the Raspberry Pi 3 Model B. Future projects should test the Raspberry Pi 3 to see if the additional proccessing power is worth its weight, literally.

### 3.3 Sensors

The following sensors were used in this project:
* 6 Sonars (SR-04)
* 1 Temperature Sensor (BMP280)
* 1 Optical Camera (Raspberry Pi Camera Module v2)
* 1 Thermal Camera (Lepton FLiR)


### 3.4 Communication

![communication](https://raw.githubusercontent.com/ArathornII/ThermalDrone/master/communication.jpg)

  All the sensors mentioned in section 3.2 are directly wired into the Raspberry Pi's 40-pin GPIO board. The full pinout can be found below. The temperature sensor, optical camera, and thermal camera all provide information essential to the thermal data collection componant of the project. The sonars are primarily used for navigation and object avoidance.
  
  The RPi inputs and processes the data from the 9 sensors, and then instructs the drone to take specific actions. These commands are sent from the RPi to the drone as MAVlink messages. These messages are transmitted via a wifi network broadcasted by the 3DR Solo's controller.

### 3.5 Multiprocessing

![multiprocessing](https://raw.githubusercontent.com/ArathornII/ThermalDrone/master/multiprocessing.jpg)

  The most recently-updated sensor reading scripts for this project utilize multiprocessing. Implementation of multiprocessing, despite the RPi Zero's single core, has improved the read speed for all six sonars (SR-04)  by over 10 times.

## 4.0 Obstacles

   So far, numerous problems have impeded the development process. The majority of issues have originated from attempts to control the drone in a GPS-denied environment, but other issues exist as well.

### 4.1 Upgrading 3DR Solo Firmware from 1.3.1 to 1.5.2

   DO NOT ATTEMPT THIS! DroneKit-Python is not yet compatible with the latest firmware release, so programming the drone in 1.5.2 is effectively useless. Hopefully, compatibility issues will be resolved in the future.

### 4.2 Taking off in GPS-denied environment

   This project would be infinitely easier if we could utilize GPS in an indoor space. The 3DR Solo's ArduCopter firmware is primarily designed for GPS navigation and offers few good options for working in a GPS-denied environment.

#### 4.2.1 GUIDED_NOGPS Mode

  GUIDED_NOGPS mode should theoretically allow the UAV to recieved GUIDED commands in a GPS-denied environment. However, tests using this mode have not yet succeded in controlled takeoff. I have not been able to determine the reason for failure.

#### 4.2.2 STABILIZE Mode

  STABILIZE mode is one of the most promising modes for flight in a GPS-denied environment because it holds the pitch and yaw such that the UAV remains level but allows for easy modification of the altitude. Also, STABILIZE mode does not require a GPS lock to initiate flight. However, STABILIZE mode does require that the thrust joystick is set to the lowest postion (not equilibirum) before initiating takeoff. This presents a seemingly insurmountable challenge for autonomous flight where the user should not need to touch let alone manipulate the controller.

#### 4.2.3 RC Overrides

  RC overrides essentially involve sending MAVlink commands to the drone that act like transmissions from the controller. This is incredibly dangerous because it inpairs the controller's function as a failsafe in case of a severe drone malfunction. So far, we have not tested this option for navigation in a GPS-denied environment.

### 4.3 Optical Camera

  The viusal camera can malfunction when there is a faulty connection between the camera module and the RPi's camera port. This could be the result of a broken cable, but more likely, the camera module itself or the camera port are broken.

## 5.0 Improvements

### 5.1 Custom Drone Design

   The best -- but most time consuming -- approach is to build a custom drone from scratch with all the neccessary sensors included in the original design. This is a feasible but unreasonable option given the current pace of the project.

### 5.2 Wired MAVlink Connection (via Accessory Bay)

  A wired serial MAVlink connection between the onboard computer (Raspberry Pi) and the 3DR Solo drone would massively decrease communication latency and resulting issues. Such a connection would be routed through the [3DR Solo's Accessory Bay](https://dev.3dr.com/hardware-accessorybay.html). This improvement would require manufactoring a custom breakout board for the Accessory Bay, which would then serve as the connection between the Raspberry Pi and the drone's Pixhawk autopilot hardware.

### 5.3 Optical Flow Rangefinder

  An Optical Flow Rangefinder is a module that points towards the ground and calculates distance through use of a high resoultion camera and 3-axis gyroscope. Therefore, Optical Flow ranges will be much more accurate than readings from $5 SR-04 sonars. Also, an Optical Flow Rangefinder can be used in calculations performed by the UAV's Extended Kalman Filter (EKF) if it is connected to the Pixhawk via the 3DR Solo's Accessory Bay.

### 5.4 Raspberry Pi 3B for Image Processing

  The RPi Zero is an incredibly useful device for its size, but it is limited by its single-core processor and less-than-optimal processing power. A RPi 3 Model B will ultimately be worth the extra weight because it will be capable of taking sonar readings -- essential input for navigation -- at far greater speeds.

### 5.5 Localization

  While sonars can be accurate, they are effectively useless beyond 1-1.5 meters. A potentially better localization system would include use of Ultra-wideband (UWB) beacons or a similar beacon-based technology.

## 6.0 Setup


### 6.1 Configuring the Raspberry Pi

  Configuring the Raspberry Pi is fairly simple. We recommend using a MicroSD card of at least 16 GB. Please follow the directions below:

#### 6.1.1 Install Operating System

  We will install **Raspbian Stretch Lite**, which means the Raspberry Pi will not use a GUI. Instead, you will communicate with the devie through the linux command line. To begin, [download](https://www.raspberrypi.org/downloads/raspbian/) the software and follow the [instructions](https://www.raspberrypi.org/documentation/installation/installing-images/README.md) on the Raspberry Pi website for your operating system.

  Then, insert your MicroSD card into the Raspberry Pi and boot-up the device with a **display, mouse, and keyboard attached**. You should connect your Raspberry Pi to a display through a Mini HDMI cable (RPi Zero W) or regular HDMI cable (All other RPi's). The default username and password are "pi" and "raspberry" respectively. You should change the password, but the leave the username alone.  To change the password, use the command ```passwd```. You will need to authenticate the device, then enter your custom password as prompted.

#### 6.1.2 Connect to Wifi

  Next, we will connect the Raspberry Pi to the internet. Please note that this step assumes you have a Raspberry Pi with an intergrated wifi module. Follow these [directions](https://www.raspberrypi.org/documentation/configuration/wireless/wireless-cli.md) and then restart the wifi interface with:

```sudo wpa_cli reconfigure```

#### 6.1.3 Enable Interfaces and Set the Timezone

  Next, we will enable the following interfaces: SSH, SPI, IC2, Camera.

  First, run the command: `sudo raspi-config`

  Select "Interfacing Options", then go there enable SSH, SPI, and IC2, Camera. Then, exit out of the configuration tool and restart your Raspberry Pi. It is possbile that the computer will do so automatically. We will use SSH to connect to the Raspberry Pi during field tests. The SPI and IC2 interfaces must be enabled for the BME280 temperature sensor and FLiR camera to function. The Camera interface allows the optical camera to function.

#### 6.1.4 Install Python Packages and Dependencies

  We need to install a large number of python packages for the various aspects of this project towork. Please copy and paste the following commands into your Raspberry Pi's CLI, and hit "return" after each one.

  First, we ensure all pre-installed packages and installers are up-to-date

```
sudo apt-get update
sudo apt-get upgrade
sudo python -m pip install --upgrade pip
sudo apt-get install build-essential python-pip python-dev python-smbus git
```


  Then, we install the SciPy stack:

```
pip install --user numpy scipy matplotlib ipython jupyter pandas sympy nose
```


  Next, we install a library to provide a cross-platform GPIO interface on the Raspberry Pi:

```
cd ~
git clone https://github.com/adafruit/Adafruit_Python_GPIO.git
cd Adafruit_Python_GPIO
sudo python setup.py install
```


  Then, we install packages and software neccesary to run the BME280:

```cd ~
git clone https://github.com/adafruit/Adafruit_Python_BME280.git
cd Adafruit_Python_BME280
sudo python setup.py install
```


 Then, we install packages for the Raspberry Pi optical camera:

```sudo apt-get install python-picamera python3-picamera
pip install "picamera[array]"
```


Next, we install packages to support image processing (OpenCV) on the Raspberry Pi:

```
sudo apt-get install build-essential cmake pkg-config
sudo apt-get install libjpeg-dev libtiff5-dev libjasper-dev libpng12-dev
sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev
sudo apt-get install libxvidcore-dev libx264-dev
sudo apt-get install libgtk2.0-dev libgtk-3-dev
sudo apt-get install libatlas-base-dev gfortran
sudo apt-get install python2.7-dev python3-dev
```

```cd ~
wget -O opencv.zip https://github.com/Itseez/opencv/archive/3.3.0.zip
unzip opencv.zip
wget -O opencv_contrib.zip https://github.com/Itseez/opencv_contrib/archive/3.3.0.zip
unzip opencv_contrib.zip
pip install numpy
```

```cd ~/opencv-3.3.0/
mkdir build
cd build
```

```
cmake -D CMAKE_BUILD_TYPE=RELEASE \
-D CMAKE_INSTALL_PREFIX=/usr/local \
-D INSTALL_PYTHON_EXAMPLES=ON \
-D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib-3.3.0/modules \
-D BUILD_EXAMPLES=ON ..
```

Now, before we move on to the actual compilation step, make sure you examine the output of CMake! Scroll down the section titled Python 2. Then make sure your Python 2 section includes valid paths to the `Interpreter` , `Libraries` , `numpy` and `packages path`. Now we will compile OpenCV. Please be aware that this process could take up to 4 hours to complete:

```
make
```

Once the compilation process has completed, run:

```
sudo make install
sudo ldconfig
```

Confirm that OpenCV has compiled correctly by running the following command:

```
ls -l /usr/local/lib/python2.7/site-packages/
```

You should see an output similar to the following:

```
total 1852
-rw-r--r-- 1 root staff 1895772 Mar 20 20:00 cv2.so
```


Next, we're going to install packages to run the FLiR Lepton:

```
sudo apt-get install python-opencv python-numpy
git clone https://github.com/groupgets/pylepton.git
cd pylepton
sudo python setup.py install
```


Finally, install the Dronekit-python package which will allow the Raspberry Pi to communicate with the 3DR Solo:

```
pip install dronekit
```

#### 6.1.5 Connecting the Raspberry Pi to the 3DR Solo

The last step is to connect the Raspberry Pi to the 3DR Solo drone. We do this by simply modifying the Raspberry Pi's wifi settings to connect to the controller's network.

#### 6.1.6 Raspberry Pi Pinout

![raspberry-pi-pinout](https://raw.githubusercontent.com/ArathornII/ThermalDrone/master/pinout.png)


### 6.2 Configuring your Computer

  The main purpose of your computer in the project is as a tool to edit, upload, and run python files on the onboard computer (Raspberry Pi). There are two steps to setting up your computer: installing a text editor and configuring SSH.


#### 6.2.1 Installing a Text Editor

  For Windows, we recommend using [Notepad++](http://csc.ucdavis.edu/~chaos/courses/nlp/Software/Windows/npp.html). For Mac and Linux, we recommend using [Atom](https://atom.io).
  
#### 6.2.2 Configuring SSH

For Windows, we recommend using [PuTTY](http://www.putty.org). For Mac, we recommend using the built-in UNIX [command line](http://accc.uic.edu/answer/how-do-i-use-ssh-and-sftp-mac-os-x). For Linux, we recommend using the built-in [command line](https://www.digitalocean.com/community/tutorials/how-to-use-ssh-to-connect-to-a-remote-server-in-ubuntu).

### 6.3 Setting up the sensors

#### 6.3.1 Sonars

  The current design uses six SR-04 sonars arranged to provided distance readings for the top, bottom, left, right, front, and back of UAV. The 5v pins of all six sonars can be wired together in parallel and connected to a single 5v GPIO pin on the Raspberry Pi. Similarly, ground pins of all six sonars can be wired together in parallel and connected to a single ground GPIO pin on the Raspberry Pi.

  However, please pay close attention to the following diagram which converts the echo pin voltage for use with a Raspberry Pi. If you follow the pinout shown above than all sonars should work properly upon first use. If you modify the pinout in any way, the GPIO pin numbers will need to be adjusted in the script.

![hc-sr04-wiring](https://raw.githubusercontent.com/ArathornII/ThermalDrone/master/hc-sr04-wiring.png)

#### 6.3.2 Optical Camera

The current design uses a single Raspberry Pi Camera v2 to capture images in the visual spectrum. A 15cm ribbon cable connects the camera module to the Raspberry Pi. Please note that the camera module requires a special ribbon cable to work with the Raspberry Pi Zero models.

#### 6.3.3 Temperature Sensor

  The current design includes a single BME280 Temperature, Humidity, and Pressure sensor. The device is mounted on the bottom of the UAV, however, future experiments should be conducted to determine the optimal location for such a sensor considering the decrease in local temperature caused by the UAV's propellers.
  
  We will only connect four of the BME280's pins to the Raspberry Pi. Please reference the following list and the pinout in section 6.1.1:
  * The 5V pin on the BME should connect to a 5V GPIO pin on the Raspberry Pi
  * The SDI pin on the BME should connect to a SDA GPIO pin on the Raspberry Pi
  * The SCK pin on the BME should connect to a **SCL** GPIO pin on the Raspberry Pi.
  * The Ground pin on the BME should connect to a Ground pin on the Raspberry Pi.


#### 6.3.4 Thermal Camera

  The current design uses a Lepton FLiR to take thermal images of the UAV's surroundings and determine relative temperatures. The FLiR is mounted on the front of the UAV in close proximity to the Optical Camera in order to minimize the transformations needed to achieve thermal composite imaging. The camera itself is very small and incredibly delicate, so please be very careful when touching it. The camera module must be mounted in a breakout board which can then be connected to the Raspberry Pi. The FLiR module must be fully in the breakout board or it will not work properly. All 8 pins on the breakout board must connect to corresponding pins on the Raspberry Pi. The SDA and SCL pins should be wired in parallel to those from the BME280 and then connected to Raspberry Pi pins 3 and 5 respectively (see pinout).

### 6.4 Running a program

Now that everything is configured, you can run a python program!

1. Turn on the 3DR Solo controller and wait for it to boot up before continuing.
2. Turn on the Raspberry Pi and ensure that it has been set to connect to the controller's wifi network. The Raspberry Pi should be connected to a portable power supply which is ideally attached to the 3DR Solo drone.
3. Connect your computer to the controller's wifi network.
4. Connect your computer to the Raspberry Pi via SSH. If you don't know the Raspberry Pi's IP address, use a tool like "Fing" to find it.
5. Upload whichever python file you wish to run to the Raspberry Pi.
6. Run the python program `filename.py` using the following command:
  ```
  python filename.py
 ```

*Please be extremely careful while conducting field tests with untested code! Be sure to attach a rope to the drone to guide it in case of a software failure. Also, always be prepared to use the "land" button on the controller*



## 7.0 Resources


### 7.1 BME280 Temperature Sensor

* [Using BME280 temperature/humidity/pressure sensor with Raspberry Pi](https://xdevs.com/guide/thp_rpi/)


### 7.2 Flight

* [Altitude Hold Mode](http://ardupilot.org/copter/docs/altholdmode.html)
* [Autopilot Firmware Upgrade from 1.3.1 to 1.5.2](https://3drpilots.com/threads/autopilot-firmware-upgrade-from-1-3-1-to-1-5-2.8819/)
* [Copter Commands in Guided Mode](http://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html)
* [Create setter functions rather than setting attributes directly](https://github.com/dronekit/dronekit-python/pull/566)
* [Move/direct Copter and send commands in GUIDED_NOGPS mode using DroneKit Python](https://github.com/studroid/dronekit-python/blob/84b68270d31798568602f36d06ee8c85c2b100ce/examples/change_attribute/change_attitude.py)
* [Guiding and Controlling Copter](http://python.dronekit.io/1.5.0/guide/copter/guided_mode.html)
* [Indoor copter flight](https://github.com/dronekit/dronekit-python/issues/697)
* [Indoor Flying Guidelines](http://ardupilot.org/copter/docs/indoor-flying.html)
* [Quaternions](http://diydrones.com/profiles/blogs/post-5-maaxx-europe-quaternions-control-code-and-pids)
* [QGroundControl User Guide](https://www.gitbook.com/book/donlakeflyer/qgroundcontrol-user-guide/details)
* [3DR Solo Development Area](https://3drpilots.com/threads/solo-development-area.5062/)
* [3DR Solo User Guide](https://3dr.com/wp-content/uploads/2016/01/v8_01_05_16.pdf)
* [DroneKit-Python Documentation](http://python.dronekit.io)

#### 7.2.1 Communication between RPi and Drone

* [API Documentation for PyMAVLink](https://www.samba.org/tridge/UAV/pymavlink/apidocs/index.html)
* [Python MAVLink interface and utilities](https://github.com/ArduPilot/pymavlink)
* [Communicating with Raspberry Pi via MAVLink](http://ardupilot.org/dev/docs/raspberry-pi-via-mavlink.html#communicating-with-raspberry-pi-via-mavlink)
* [Companion Computers](http://python.dronekit.io/1.5.0/guide/companion-computers.html)
* [Solo Development Guide](https://dev.3dr.com)
* [Mavlink (developer page)](https://www.pixhawk.org/dev/mavlink?s[]=mavlink#mavlink_developer_page)
* [MAVLINK Common Message Set](http://mavlink.org/messages/common)
* [MAVLink Mission Command Messages (MAV_CMD)](http://ardupilot.org/copter/docs/common-mavlink-mission-command-messages-mav_cmd.html#mavlink-mission-command-messages-mav-cmd)
* [MavLink Tutorial for Absolute Dummies (Part I)](http://api.ning.com/files/i*tFWQTF2R*7Mmw7hksAU-u9IABKNDO9apguOiSOCfvi2znk1tXhur0Bt00jTOldFvob-Sczg3*lDcgChG26QaHZpzEcISM5/MAVLINK_FOR_DUMMIESPart1_v.1.1.pdf)
* [MAVlink Message Definitions](https://github.com/mavlink/mavlink/blob/master/message_definitions/v1.0/common.xml)
* [MAVProxy Landing Page](http://qgroundcontrol.org/mavlink/mavproxy_startpage)
* [MAVProxy Ardupilot Documentation](http://ardupilot.github.io/MAVProxy/html/index.html#mavproxy)
* [MAVProxy Cheetsheet](http://ardupilot.github.io/MAVProxy/html/_static/files/MAVProxyCheetsheet.pdf)
* [Operating Arducopter without GPS over MAVLink](http://diydrones.com/group/arducopterusergroup/forum/topics/operating-arducopter-without-gps-over-mavlink?page=1&commentId=705844%3AComment%3A1962580&x=1#705844Comment1962580)
* [3DR Solo Breakout Board Specs](https://github.com/3drobotics/Pixhawk_OS_Hardware/tree/master/Accessory_Breakout_X1)
* [Vehicle State and Settings](http://python.dronekit.io/guide/vehicle_state_and_parameters.html)
* [Which MAVLink commands can fly a copter without GPS?](https://discuss.ardupilot.org/t/which-mavlink-commands-can-fly-a-copter-without-gps/9353/1)

### 7.3 Image Processing

* [Basic Recipes](http://picamera.readthedocs.io/en/release-1.10/recipes1.html#basic-recipes)
* [Accessing the Raspberry Pi Camera with OpenCV and Python](https://www.pyimagesearch.com/2015/03/30/accessing-the-raspberry-pi-camera-with-opencv-and-python/)
* [applyColorMap for pseudocoloring in OpenCV ( C++ / Python )](https://www.learnopencv.com/applycolormap-for-pseudocoloring-in-opencv-c-python/)
* [Camera Module](https://www.raspberrypi.org/documentation/hardware/camera/)
* [FLIR Lepton Hookup Guide](https://learn.sparkfun.com/tutorials/flir-lepton-hookup-guide)
* [Quick and dirty pure python library for interfacing with FLIR lepton](https://github.com/groupgets/pylepton)
* [How to install/use the Raspberry Pi Camera](https://thepihut.com/blogs/raspberry-pi-tutorials/16021420-how-to-install-use-the-raspberry-pi-camera)
* [Image size (Python, OpenCV)](https://stackoverflow.com/questions/13033278/image-size-python-opencv)
* [OpenCV - Saving images to a particular folder of choice](https://stackoverflow.com/questions/41586429/opencv-saving-images-to-a-particular-folder-of-choice)
* [python OpenCV - add alpha channel to RGB image](https://stackoverflow.com/questions/32290096/python-opencv-add-alpha-channel-to-rgb-image)
* [Install OpenCV 3 + Python on your Raspberry Pi](https://www.pyimagesearch.com/2017/09/04/raspbian-stretch-install-opencv-3-python-on-your-raspberry-pi/)

### 7.4 Data Logging

* [File Handling Cheat Sheet in Python](http://www.pythonforbeginners.com/cheatsheet/python-file-handling)
* [Reading and Writing Files in Python](http://www.pythonforbeginners.com/files/reading-and-writing-files-in-python)

### 7.5 Multiprocessing

* [Multiprocessing — Process-based parallelism](https://docs.python.org/3/library/multiprocessing.html)
* [An introduction to parallel programming using Python's multiprocessing module](http://sebastianraschka.com/Articles/2014_multiprocessing.html)
* [Communication Between Processes](https://pymotw.com/2/multiprocessing/communication.html)
* [Multiprocessing with Python](https://www.raspberrypi.org/magpi/multiprocessing-with-python/)
* [Multiprocessing vs Threading Python](https://stackoverflow.com/questions/3044580/multiprocessing-vs-threading-python)
* [What does if name == “main”: do?](https://stackoverflow.com/questions/419163/what-does-if-name-main-do)
* [Python 201: A multiprocessing tutorial](https://www.blog.pythonlibrary.org/2016/08/02/python-201-a-multiprocessing-tutorial/)

### 7.6 Object Avoidance and Rangefinding

* [LIDAR-Lite Rangefinder](http://ardupilot.org/copter/docs/common-rangefinder-lidarlite.html#lidar-lite-rangefinder)
* [Script to Read SR-04](https://github.com/feranick/Pi-bot/blob/master/Utilities/sonar/sonar_distance.py)
* [PX4FLOW Optical Flow Camera Board Overview](http://ardupilot.org/copter/docs/common-px4flow-overview.html#px4flow-optical-flow-camera-board-overview)
* [PX4FLOW V1.3.1 OPTICAL FLOW Camera](http://www.hobbywow.com/en-px4flow-v1-3-1-optical-flow-camera-and-mb1043-sonar-sensor-for-px4-pixhawk-pix-flight-control-p240168.htm)

### 7.7 Raspberry Pi

* [Raspbian Installation](https://www.raspberrypi.org/downloads/raspbian/)
* [Installing the SciPy Stack](https://www.scipy.org/install.html)
* [Raspberry Pi Power Requirements](https://www.raspberrypi.org/help/faqs/#topPower)
* [Raspberry Pi Pinout](https://pinout.xyz)







