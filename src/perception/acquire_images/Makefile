CPP        		= g++
CFLAGS     		= -g -std=c++11 -fPIC -O3
TARGET			= acquire_images

$(TARGET): acquire_images.cpp example.hpp 
	$(CPP) $(CFLAGS) acquire_images.cpp -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -lrealsense -lglfw -lGL -o $(TARGET)


all: $(TARGET)

clean:
	rm -f *.o *.so $(TARGET)
