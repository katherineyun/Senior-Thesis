# distutils: language = c++
cimport numpy as np
#include yarp classes
from libc cimport float
from libcpp.vector cimport vector
    

# Declare the prototype of the C function we are interested in calling
cdef extern from "yarpMath.cpp":
    #c vector = python list
    vector[double] dcmToAxis(double r11, double r12, double r13,
            double r21, double r22, double r23,
            double r31, double r32, double r33)  

    



def dcm2axis(float r11, float r12, float r13,
            float r21, float r22, float r23,
           float r31, float r32, float r33): #
    """ R is a yarp.Matrix()
    """
    
    # Call the C function
    v = dcmToAxis(r11, r12, r13, 
                r21, r22, r23, 
                r31, r32, r33)
    
    return v
