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

with Lepton("/dev/spidev0.1") as l:
  a,_ = l.capture()
cv2.normalize(a, a, 0, 65535, cv2.NORM_MINMAX) # extend contrast
np.right_shift(a, 8, a) # fit data into 8 bits
a = np.uint8(a)
a = cv2.applyColorMap(a, cv2.COLORMAP_JET)

(h, w) = a.shape[:2]
center = (w / 2, h / 2)

M = cv2.getRotationMatrix2D(center, 180, 1.0)
rotated_image = cv2.warpAffine(a, M, (w, h))
cv2.waitKey(0)

cv2.imwrite("img-" + timestamp() + ".jpg", rotated_image) # write it
