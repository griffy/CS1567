OBJS=project.o robot.o firfilter.o
CFLAGS=-ggdb -g3
LIB_FLAGS=-L. -lrobot_if
CPP_LIB_FLAGS=$(LIB_FLAGS) -lstdc++ -lrobot_if++
LIB_LINK=-lhighgui -lcv -lcxcore
LIB_LINK_NEW=-lopencv_core -lopencv_imgproc -lopencv_highgui -lm

all: $(OBJS)
	g++ $(CFLAGS) -o project $(OBJS) $(CPP_LIB_FLAGS) $(LIB_LINK)

new: $(OBJS)
	g++ $(CFLAGS) -o project $(OBJS) $(CPP_LIB_FLAGS) $(LIB_LINK_NEW)

project.o: project.cpp robot.h firfilter.h
	g++ $(CFLAGS) -c project.cpp

robot.o: robot.cpp robot.h
	g++ $(CFLAGS) -c robot.cpp

firfilter.o: firfilter.cpp firfilter.h
	g++ $(CFLAGS) -c firfilter.cpp

## EXTRAS ##
# fake robot interface for emulating behavior of actual with
# collected sample data
fakerobotinterface.o: fakerobotinterface.cpp fakerobotinterface.h
	g++ $(CFLAGS) -c fakerobotinterface.cpp

# separate executable for collecting data
gatherdata: gatherdata.o
	g++ -o gatherdata gatherdata.o firfilter.o $(CPP_LIB_FLAGS) $(LIB_LINK)

gatherdata_new: gatherdata.o
	g++ -o gatherdata gatherdata.o firfilter.o $(CPP_LIB_FLAGS) $(LIB_LINK_NEW)

gatherdata.o: gatherdata.cpp firfilter.h
	g++ $(CFLAGS) -c gatherdata.cpp

clean:
	rm -rf *.o
	rm -rf project
	rm -rf gatherdata
