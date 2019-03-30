# distutils: language = c++
cimport numpy as np
from libcpp.vector cimport vector 


    

# Declare the prototype of the C function we are interested in calling
cdef extern from "yarpMath.cpp":
    ctypedef struct SwigPyObject:
        void *ptr
    vector[double] dcmToAxis(void* R)  

    



def dcm2axis(R): #
    """ R is a yarp.Matrix()
    """
    cdef SwigPyObject *swig_obj = <SwigPyObject*>R.this
    
    # Call the C function
    v = dcmToAxis(swig_obj.ptr)
    
    return v
