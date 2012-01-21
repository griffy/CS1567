OBJS=fakerobot.o fakerobotinterface.o ../firfilter.o
CFLAGS=-ggdb -g3
LIB_FLAGS=-L. -lrobot_if
CPP_LIB_FLAGS=$(LIB_FLAGS) -lrobot_if++
LIB_LINK=-lhighgui -lcv -lcxcore
LIB_LINK_NEW=-lopencv_core -lopencv_imgproc -lopencv_highgui -lm

all: $(OBJS)
	g++ $(CFLAGS) -o fakerobot.out $(OBJS) $(CPP_LIB_FLAGS) $(LIB_LINK)

new: $(OBJS)
	g++ $(CFLAGS) -o fakerobot.out $(OBJS) $(CPP_LIB_FLAGS) $(LIB_LINK_NEW)

fakerobot.o: fakerobot.cpp fakerobotinterface.h
	g++ $(CFLAGS) -c fakerobot.cpp

fakerobotinterface.o: fakerobotinterface.cpp fakerobotinterface.h
	g++ $(CFLAGS) -c fakerobotinterface.cpp

clean:
	rm -f *.o
	rm -f fakerobot.out
