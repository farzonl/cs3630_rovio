OBJS := $(addsuffix .o, $(basename $(wildcard *.cpp)))


robot: $(OBJS)
	g++ -L/sw/lib -L/usr/local/opencv/lib -lcurl -lcv -lhighgui -o $@ $(OBJS)

%.o: %.cpp
	g++ -g -O0 -I/usr/local/include/opencv -I/usr/local/opencv/include/opencv -c $<

clean:
	rm -f $(OBJS) robot
