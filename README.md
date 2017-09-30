# ThermalDrone

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

We considered five primary criteria when choosing which drone to purchase: assembly/configuration time, payload capacity, size, price, and programmability. Based on these criteria,

### 3.2 Communication


### 3.3 Multiprocessing


## 4.0 Obstacles



## 5.0 Improvements

### 5.1 Custom Design

### 5.2 Wired MAVlink Connection (via Accessory Bay)

### 5.3 Optical Flow Rangefinder



## 6.0 Setup




## 7.0 Resources

### 7.1 BME280 Temp Sensor


### 7.2 Flight


### 7.3 Image Processing


### 7.4 Data Logging


### 7.5 Multiprocessing


### 7.6 Object Avoidance and Rangefinding


#### 7.6.1 Communication between RPi and Drone


### 7.7 Raspberry Pi








