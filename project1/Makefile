OBJS=project.o robot.o pose.o firfilter.o utilities.o PID.o
CFLAGS=-ggdb -g3
LIB_FLAGS=-L. -lrobot_if
CPP_LIB_FLAGS=$(LIB_FLAGS) -lrobot_if++
LIB_LINK=-lhighgui -lcv -lcxcore -lm
LIB_LINK_NEW=-lopencv_core -lopencv_imgproc -lopencv_highgui -lm

all: $(OBJS)
	g++ $(CFLAGS) -o project.out $(OBJS) $(CPP_LIB_FLAGS) $(LIB_LINK)

new: $(OBJS)
	g++ $(CFLAGS) -o project.out $(OBJS) $(CPP_LIB_FLAGS) $(LIB_LINK_NEW)

project.o: project.cpp robot.h pose.h firfilter.h PID.h
	g++ $(CFLAGS) -c project.cpp

robot.o: robot.cpp robot.h pose.h firfilter.h utilities.h PID.h
	g++ $(CFLAGS) -c robot.cpp

pose.o: pose.cpp pose.h
	g++ $(CFLAGS) -c pose.cpp

firfilter.o: firfilter.cpp firfilter.h
	g++ $(CFLAGS) -c firfilter.cpp

utilities.o: utilities.cpp utilities.h
	g++ $(CFLAGS) -c utilities.cpp

PID.o: PID.h PID.cpp
	g++ $(CFLAGS) -c PID.cpp

clean:
	rm -f *.o
	rm -f project.out
