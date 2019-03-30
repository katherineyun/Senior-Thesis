import numpy as np

import cYarpMath 


a=np.array(cYarpMath.dcm2axis(-1,0,0,0,1,0,0,0,-1))
print(type(a))
print(a.shape)

print(a)
