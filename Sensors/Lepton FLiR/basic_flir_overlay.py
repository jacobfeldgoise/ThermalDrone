import time
from picamera.array import PiRGBArray
from picamera import PiCamera
import traceback
import numpy as np
import cv2
from pylepton import Lepton

def timestamp():

    #### This function generates a timestemp which will be written to the log file ####

    local_timezone = tzlocal.get_localzone()
    utc_time = datetime.utcnow()

    # Convert time zone
    local_time= utc_time.replace(tzinfo=pytz.utc).astimezone(local_timezone)
    timestamp_local = local_time.strftime('%m-%d-%Y %H:%M:%S %Z')
    return timestamp_local

def OverlayImage(src, overlay, posx, posy, S, D):

	for x in range(overlay.width):

		if x+posx < src.width:

			for y in range(overlay.height):

				if y+posy < src.width:

					source = cv.Get2D(src, y+posy, x+posx)
					over   = cv.Get2D(overlay, y, x)
					merger = [0, 0, 0, 0]

					for i in range(3):
						merger[i] = (S[i]*source[i]+D[i]*over[i])

					merged = tuple(merger)

					cv.Set2D(src, y+posy, x+posx, merged)

while True:
    with Lepton("/dev/spidev0.1") as l:
        a,_ = l.capture()
    cv2.normalize(a, a, 0, 65535, cv2.NORM_MINMAX) # extend contrast
    np.right_shift(a, 8, a) # fit data into 8 bits
    a = np.uint8(a)
    a = cv2.applyColorMap(a, cv2.COLORMAP_JET)

    (h, w) = a.shape[:2]
    center = (w / 2, h / 2)

    M = cv2.getRotationMatrix2D(center, 180, 1.0)
    flir = cv2.warpAffine(a, M, (w, h))
    cv2.waitKey(0)

    # initialize the regular camera and grab a reference to the raw camera capture
    camera = PiCamera()
    rawCapture = PiRGBArray(camera)

    # allow the regular camera to warmup
    time.sleep(0.1)

    # grab an image from the regular camera
    camera.capture(rawCapture, format="bgr")
    image = rawCapture.array

    posx = 0					        # Define a point (posx, posy) on the source
    posy = 0                            # image where the overlay will be placed
    S = (0.5, 0.5, 0.5, 0.5)			# Define blending coefficients S and D
    D = (0.5, 0.5, 0.5, 0.5)

    OverlayImage(image, flir, posx, posy, S, D)

    print("Writing & timestamping image...")
    cv2.imwrite("img-" + timestamp() + ".jpg", image) # write it
