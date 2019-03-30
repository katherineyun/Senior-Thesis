0. If you cannot import and use this module, you need to delete the "build" folder and cYarpMath.cpython-36m-x86_64-linux-gnu.so and complie it on your own computer.
1. This program uses cython. You need to do "conda install cython=0.28.5" or pip3 install

2. setup.py is like a cmake file. All compilation related configuration is in that file.
	if there is a undefined symbol error, you should check 
	libraries=[...],
        library_dirs=[...] 
3. To compile, open a terminal in this folder, and write "python setup.py build_ext --inplace"
4. If you want to know more about Cython see this website for a simple example: https://cython.readthedocs.io/en/latest/src/userguide/wrapping_CPlusPlus.html


