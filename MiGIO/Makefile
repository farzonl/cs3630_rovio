OBJS := $(addsuffix .o, $(basename $(wildcard *.cpp)))


robot: $(OBJS)
	g++ -L/sw/lib -L/usr/local/opencv/lib -lcurl -lopencv_core -lopencv_highgui -o $@ $(OBJS)

%.o: %.cpp
	g++ -g -O0 -I/usr/local/opencv/include/opencv -c $<

clean:
	rm -f $(OBJS) robot
