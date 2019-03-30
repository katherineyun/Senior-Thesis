import numpy
from Cython.Distutils import build_ext
from distutils.extension import Extension
from Cython.Build import cythonize




if __name__ == '__main__':
    
    params=dict()
    extensions=[
        Extension('cYarpMath',sources=['cYarpMath.pyx'],
            depends=['yarpMath.cpp'],
            include_dirs=[numpy.get_include(), '/usr/local/include/yarp'],
            libraries=['YARP_math', 'YARP_sig', 'YARP_init'],
            library_dirs=['/usr/local/lib'],
            extra_compile_args=["-std=c++14"])
        ]
    # Override the C-extension building so that it knows about '.pyx'
    # Cython files
    params['cmdclass'] = dict(build_ext=build_ext)
    params['ext_modules']=cythonize(extensions)

    # Call the actual building/packaging function (see distutils docs)
    from numpy.distutils.core import setup
    setup(**params)