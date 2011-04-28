OBJS := $(addsuffix .o, $(basename $(wildcard *.cpp)))


robot: $(OBJS)
	/usr/local/gcc47/bin/g++ -g -L/sw/lib -L/usr/local/opencv/lib -lcurl -lopencv_core -lcurl -lopencv_highgui -lopencv_objdetect  -lopencv_imgproc  -L../VisiLibity/source_code -L../../cvblobslib_OpenCV_v6 -lblob -lvisilibity -o $@ $(OBJS) -Wl,-dead_strip

%.o: %.cpp
	/usr/local/gcc47/bin/g++ -Wall -g -O0 -fno-omit-frame-pointer -Wall -I/usr/local/include/opencv -I/usr/local/opencv/include/opencv  -I../../cvblobslib_OpenCV_v6 -I../VisiLibity/source_code -c $<

clean:
	rm -f $(OBJS) robot
