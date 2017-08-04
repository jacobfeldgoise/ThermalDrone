import RPi.GPIO as GPIO
import time, sys, datetime, pytz, tzlocal, os
import numpy as np
from datetime import datetime
from dateutil import tz

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

pulse = 0.00002


if __name__ == "__main__":
    # All code -- including calling pre-defined functions -- goes here

    
