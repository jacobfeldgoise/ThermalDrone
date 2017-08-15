import time, tzlocal, pytz, io
from picamera.array import PiRGBArray
import picamera
import traceback
import numpy as np
import cv2
from pylepton import Lepton
from datetime import datetime                                                   # Needed to generate timestamp
from dateutil import tz                                                         # Needed to generate timestamp

def timestamp_img():

    #### This function generates a timestemp which will be written to the log file ####

    local_timezone = tzlocal.get_localzone()
    utc_time = datetime.utcnow()

    # Convert time zone
    local_time= utc_time.replace(tzinfo=pytz.utc).astimezone(local_timezone)
    timestamp_local = local_time.strftime('%m-%d-%Y-%H-%M-%S')
    return timestamp_local

def OverlayImage(base_img, overlay_img):

    dst = cv2.addWeighted(base_img,0.7,overlay_img,0.3,0)
    return dst

with Lepton("/dev/spidev0.1") as l:
    a,_ = l.capture()
cv2.normalize(a, a, 0, 65535, cv2.NORM_MINMAX) # extend contrast
np.right_shift(a, 8, a) # fit data into 8 bits
a = np.uint8(a)
a = cv2.applyColorMap(a, cv2.COLORMAP_JET)

(h, w) = a.shape[:2]

print h, w

center = (w / 2, h / 2)

M = cv2.getRotationMatrix2D(center, 180, 1.0)
flir = cv2.warpAffine(a, M, (w, h))
cv2.waitKey(0)

# Create the in-memory stream
stream = io.BytesIO()
with picamera.PiCamera() as camera:
    camera.start_preview()
    time.sleep(2)
    camera.capture(stream, format='jpeg')
# Construct a numpy array from the stream
data = np.fromstring(stream.getvalue(), dtype=np.uint8)
# "Decode" the image from the array, preserving colour
image = cv2.imdecode(data, 1)
# OpenCV returns an array with data in BGR order. If you want RGB instead
# use the following...
image = image[:, :, ::-1]
resized_image = cv2.resize(image, (80, 60))

(h, w) = resized_image.shape[:2]

print h, w

image_blended = OverlayImage(resized_image, flir)

print("Writing & timestamping image...")
cv2.imwrite("/home/pi/thermal_images/img-" + str(timestamp_img()) + ".jpg", image_blended) # write it
