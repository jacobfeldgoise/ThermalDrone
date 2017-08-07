import numpy as np
import cv2
from pylepton import Lepton

with Lepton() as l:
  a,_ = l.capture()
cv2.normalize(a, a, 0, 65535, cv2.NORM_MINMAX) # extend contrast
np.right_shift(a, 8, a) # fit data into 8 bits
cv2.imwrite("output.jpg", np.uint8(a)) # write itc
