OBJS := $(addsuffix .o, $(basename $(wildcard *.cpp)))


robot: $(OBJS)
	g++ -lcurl -lcv -lcvaux -lhighgui -o $@ $(OBJS)

%.o: %.cpp
	g++ -g -O0 -I /usr/include/opencv -c $<

clean:
	rm -f $(OBJS) robot
