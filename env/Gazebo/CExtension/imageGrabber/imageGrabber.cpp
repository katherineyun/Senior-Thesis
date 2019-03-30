
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <iostream>
#include <Python.h>


using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#include <Python.h>

// this struct is used for dewrapping the swig object
typedef struct {
  PyObject_HEAD
  void *ptr; // This is the pointer to the actual C++ instance
  void *ty;  // swig_type_info originally, but shouldn't matter
  int own;
  PyObject *next;
} SwigPyObject;




unsigned char* getImagePointer(void* p){ 
  
  // cast from void* to the correct type of the port
	BufferedPort<ImageOf<PixelRgb>> *port=reinterpret_cast<BufferedPort<ImageOf<PixelRgb>> *>(p);
  
   ImageOf<PixelRgb>* img =NULL;
   
  while (true){
    
     img = port->read();

     if (img!=NULL) break;
     
  }
  
  unsigned char* imagePointer=img->getRawImage(); // get the pointer of the image
  
  
  if (imagePointer==NULL){
    cout<<"YARP::getRawImage returns a NULL pointer!"<<endl;
  }
  return imagePointer;

}





 

   



