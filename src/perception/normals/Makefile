VERSION			= 3.5m
CPP        		= g++
CFLAGS     		= -g -std=c++11 -fPIC -O3
PYTHON_INC 		= /usr/include/python$(VERSION)/
PYTHON_LIB 		= python$(VERSION)
SWIG       		= swig
SWIGOPT    		= -c++ -python
TARGET			= normals

%_wrap.cxx: %.i
	$(SWIG) $(SWIGOPT) $<

%.py: %.i ;

_%.so: %_wrap.cxx 
	$(CPP) $(CFLAGS) -shared $< normals.cpp -o _$(TARGET).so -I$(PYTHON_INC) -I. -l$(PYTHON_LIB)

all: $(TARGET).py _$(TARGET).so

clean:
	rm -f *_wrap* *.o *.so $(TARGET).py
