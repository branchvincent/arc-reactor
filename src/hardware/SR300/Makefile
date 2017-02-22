ANACONDA3  = /home/bk/anaconda3 #change this to your anaconda3 directory
REALSENSE  = reactor/src/hardware/SR300 #change this to where you have the realsense headers
CPP        = g++
CFLAGS     = -c
SRCS       =
TARGET     = _realsense
CXXSRCS    = realsense_wrap.cxx
PYTHON_INCLUDE    = -I$(ANACONDA3)/include/python3.5m
PYTHON_LIB = -L$(ANACONDA3)/lib -lpython3.5m
REALSENSE_INC = -I$(REALSENSE)
NUMPY_INC = -I$(ANACONDA3)/lib/python3.5/site-packages/numpy/core/include
REALSENSELIB = -lrealsense #requires that the user has librealsense.so in the LD_LIBRARY_PATH
INTERFACE  = realsense.i
SWIG       = swig
SWIGOPT    = -c++ -python 
SO		   = .so
CCSHARED   = -fPIC -std=c++11
CXXSHARED  = g++ -shared
OBJS       = $(SRCS:.cpp=.o) $(CXXSRCS:.cxx=.o)

all: $(SRCS)
	$(SWIG) $(SWIGOPT) $(INTERFACE)
	$(CPP) $(CCSHARED) $(CFLAGS) $(ICXXSRCS) $(SRCS) $(CXXSRCS) $(PYTHON_INCLUDE) $(REALSENSE_INC) $(NUMPY_INC)
	$(CXXSHARED) $(OBJS) -o $(TARGET)$(SO) $(PYTHON_INCLUDE) $(PYTHON_LIB) $(REALSENSELIB)

clean:
	rm -f *_wrap* *.o *~ *$(SO) mypython *.pyc .~* core


