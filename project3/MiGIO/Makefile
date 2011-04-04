OBJS := $(addsuffix .o, $(basename $(wildcard *.cpp)))


robot: $(OBJS)
	g++ -L/sw/lib -L/usr/local/opencv/lib -lcurl -lopencv_core -lcurl -lopencv_highgui -lopencv_objdetect  -lopencv_imgproc  -L../VisiLibity/source_code -L../../cvblobslib_OpenCV_v6 -lblob -lvisilibity -o $@ $(OBJS)

%.o: %.cpp
	g++ -Wall -g -O -I/usr/local/include/opencv -I/usr/local/opencv/include/opencv  -I../../cvblobslib_OpenCV_v6 -I../VisiLibity/source_code -c $<

clean:
	rm -f $(OBJS) robot
