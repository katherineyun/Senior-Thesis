1. This program uses cython. You need to do "conda install cython=0.28.5"
2. see this website for a simple example: https://cython.readthedocs.io/en/latest/src/userguide/wrapping_CPlusPlus.html
3. setup.py is like a cmake file. All compilation related configuration is in that file.
	if there is a undefined symbol error, you should check 
	libraries=['YARP_OS', 'YARP_sig', 'YARP_init'],
        library_dirs=["/usr/local/lib"] 
4. To compile, open a terminal in this folder, and write "python setup.py build_ext --inplace"
5. cameraReader now has only 3 lines:
	a=imageGrabber(640*480*3)
	a = np.reshape(a, (480, 640, 3))
	scipy.misc.imsave("out"+str(self.envStepCounter)+".jpg", a) 
	you can also uses test.py with iCub_SIM for a basic test. In iCub_SIM, you should write
 	Network::connect("/icubSim/cam/right", "/test/img");
	In Gazebo, you should write
	Network::connect("/icub/cam/right", "/test/img");
6. The wrapper is implemented in cImageGrabber.pyx. You need to change 
	def imageGrabber(size) part and cdef extern from "imageGrabber.cpp":
	part to make it a class instead of a single function

