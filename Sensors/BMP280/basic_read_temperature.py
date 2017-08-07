from Adafruit_BME280 import *
import RPi.GPIO as GPIO
import time, sys, datetime, pytz, tzlocal, os, pickle, multiprocessing
from datetime import datetime
from dateutil import tz

sensor = BME280(t_mode=BME280_OSAMPLE_8, p_mode=BME280_OSAMPLE_8, h_mode=BME280_OSAMPLE_8)

for x in range(20):
    print "Temperature (C): " + str(sensor.read_temperature())
    time.sleep(0.2)
