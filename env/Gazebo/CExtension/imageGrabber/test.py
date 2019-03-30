import numpy as np

import cImageGrabber 
import scipy.misc

a=cImageGrabber.imageGrabber(240*320*3)
print(a.shape)

a=np.reshape(a,(240,320,3))



scipy.misc.imsave("out.jpg",a)
