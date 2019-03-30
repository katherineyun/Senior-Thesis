

# distutils: language = c++



# Declare the prototype of the C function we are interested in calling
cdef extern from "imageGrabber.cpp":
    ctypedef struct SwigPyObject:
        void *ptr

    unsigned char* getImagePointer(void* port) 

from libc.stdlib cimport free

from cpython cimport PyObject, Py_INCREF


# Import the Python-level symbols of numpy
import numpy as np

# Import the C-level symbols of numpy
cimport numpy as np

# Numpy must be initialized. When using numpy from C or Cython you must
# _always_ do that, or you will have segfaults
np.import_array()

# We need to build an array-wrapper class to deallocate our array when
# the Python object is deleted.

cdef class ArrayWrapper:
    cdef void* data_ptr
    cdef int size
    

    cdef set_data(self, int size, void* data_ptr):
        """ Set the data of the array
        This cannot be done in the constructor as it must recieve C-level
        arguments.
        Parameters:
        -----------
        size: int
            Length of the array.
        data_ptr: void*
            Pointer to the data            
        """
        self.data_ptr = data_ptr
        self.size = size
        

    def __array__(self):
        """ Here we use the __array__ method, that is called when numpy
            tries to get an array from the object."""
        cdef np.npy_intp shape[1]
        
        shape[0] = <np.npy_intp> self.size
        # Create a 1D array, of length 'size'
        ndarray = np.PyArray_SimpleNewFromData(1, shape,
                                               np.NPY_UBYTE, self.data_ptr)
        return ndarray

    def __dealloc__(self):
        """ Frees the array. This is called by Python when all the
        references to the object are gone. """
        if not self.data_ptr:
            free(<void*>self.data_ptr)


def imageGrabber(size,port):
    """ port is a PyObject*, before you pass it to c++ function, you can 
        1. cast it to the correct c++ datatype:
            you have to include the c++ header file by cdef extern from "<.h>" block
            Example for using cdef extern from "<.h>" block to include c++ declearation:
            cdef extern from "<yarp/sig/Matrix.h>":
                cdef cppclass Matrix:
                    Matrix() # include nullary constructor
                    Matrix(int r, int c) # include constructor with parameters
            then do something like MyCppFunc(<c++Type*> PyObject*)

        2. directly cast it to void*:
            then do somthing like MyCppFunc(<void*> PyObject*)
            finally, in c++ file, use reinterpret_cast to cast void* to the correct c++ pointer
    """

    cdef unsigned char *img #image pointer
    cdef np.ndarray ndarray

    cdef SwigPyObject *swig_obj = <SwigPyObject*>port.this
    
    # Usually you want
    # cdef SwigPyObject *swig_obj = <SwigPyObject*>pythonswig.this
    # cdef MyCppClass *mycpp_ptr = <MyCppClass*?>swig_obj.ptr
    # And if you want a proper instance instead of a pointer:
    # cdef MyCppClass my_instance = deref(mycpp_ptr)

    
    # Call the C function
    img = getImagePointer(swig_obj.ptr) # pass in the real void* pointer for the port
    
    array_wrapper = ArrayWrapper()
    
    array_wrapper.set_data(size, <void*> img) 
    ndarray = np.array(array_wrapper, copy=False)
    # Assign our object to the 'base' of the ndarray object
    ndarray.base = <PyObject*> array_wrapper
    # Increment the reference count, as the above assignement was done in
    # C, and Python does not know that there is this additional reference
    Py_INCREF(array_wrapper)
    

    return ndarray
