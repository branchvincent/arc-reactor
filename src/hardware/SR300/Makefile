VERSION			= 2.7
CPP        		= g++
CFLAGS     		= -g -std=c++11 -fPIC -O3
PYTHON_INC 		= /usr/include/python$(VERSION)
PYTHON_LIB 		= python$(VERSION)
NUMPY_INC  		= $(PYTHON_INC)/site-packages/numpy/core/include
RS_INC 	   		= .
RS_LIB     		= realsense
SWIG       		= swig
SWIGOPT    		= -c++ -python
TARGET			= realsense

%_wrap.cxx: %.i
	$(SWIG) $(SWIGOPT) $<

%.py: %.i ;

%_wrap.o: %_wrap.cxx
	$(CPP) $(CFLAGS) -c $< -I$(PYTHON_INC) -I$(RS_INC) -I$(NUMPY_INC)

_%.so: %_wrap.o
	$(CPP) $(CFLAGS) -shared $< -o _$(TARGET).so -I$(PYTHON_INC) -I$(RS_INC) -l$(PYTHON_LIB) -l$(RS_LIB)

all: $(TARGET).py _$(TARGET).so

clean:
	rm -f *_wrap* *.o *.so $(TARGET).py
