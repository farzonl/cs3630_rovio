OBJS := $(addsuffix .o, $(basename $(wildcard *.cpp)))


robot: $(OBJS)
	/usr/local/llvm/bin/clang++ -g -L/sw/lib -L/usr/local/opencv/lib -lcurl -lopencv_core -lcurl -lopencv_highgui -lopencv_objdetect  -lopencv_imgproc  -L../VisiLibity/source_code -L../../cvblobslib_OpenCV_v6 -lblob -lvisilibity -o $@ $(OBJS) -Wl,-dead_strip

%.o: %.cpp
	/usr/local/llvm/bin/clang++ -Wall -g -O0 -I/usr/local/include/opencv -I/usr/local/opencv/include/opencv  -I../../cvblobslib_OpenCV_v6 -I../VisiLibity/source_code -c $<

clean:
	rm -f $(OBJS) robot
