# distutils: language = c++
# distutils: sources = Rectangle.cpp






# Decalre the class with cdef
cdef extern from "Rectangle.h" namespace "shapes":
	cdef cppclass Rectangle:
		Rectangle() except +
		Rectangle(int, int, int, int) except +
		int x0, y0, x1, y1
		int getArea()
		void getSize(int* width, int* height)
		void move(int, int)





# Create a Cython extension type which holds a C++ instance
# as an attribute and create a bunch of forwarding methods
# Python extension type.
cdef class PyRectangle:
	cdef Rectangle * c_rect  # hold a pointer to the C++ instance which we're wrapping

	def __cinit__(self, int x0, int y0, int x1, int y1):
		self.c_rect = new Rectangle(x0, y0, x1, y1)


	def __dealloc__(self):
		del self.c_rect

	def get_area(self):
		return self.c_rect.getArea()

	def get_size(self):
		cdef int width, height
		self.c_rect.getSize(&width, &height)
		return width, height

	def move(self, dx, dy):
		self.c_rect.move(dx, dy)