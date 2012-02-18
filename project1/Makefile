OBJS=project.o robot.o pose.o firfilter.o kalman.o rovioKalmanFilter.o utilities.o PID.o constants.h
DATA_OBJS = robot.o pose.o firfilter.o kalman.o rovioKalmanFilter.o utilities.o PID.o collectData.o constants.h
CFLAGS=-ggdb -g3
LIB_FLAGS=-L. -lrobot_if
CPP_LIB_FLAGS=$(LIB_FLAGS) -lrobot_if++
LIB_LINK=-lhighgui -lcv -lcxcore -lm -lgslcblas -L/usr/lib64/atlas -lclapack
LIB_LINK_NEW=-lopencv_core -lopencv_imgproc -lopencv_highgui -lm -lgslcblas -L/usr/lib64/atlas -llapack


all: $(OBJS)
	g++ $(CFLAGS) -o project.out $(OBJS) $(CPP_LIB_FLAGS) $(LIB_LINK)

data: $(DATA_OBJS) constants.h
	g++ $(CFLAGS) -o collectData.out $(DATA_OBJS) $(CPP_LIB_FLAGS) $(LIB_LINK)

new: $(OBJS) constants.h
	g++ $(CFLAGS) -o project.out $(OBJS) $(CPP_LIB_FLAGS) $(LIB_LINK_NEW)

project.o: project.cpp robot.h pose.h firfilter.h kalman.h PID.h constants.h
	g++ $(CFLAGS) -c project.cpp

robot.o: robot.cpp robot.h pose.h firfilter.h kalman.h utilities.h PID.h
	g++ $(CFLAGS) -c robot.cpp

pose.o: pose.cpp pose.h
	g++ $(CFLAGS) -c pose.cpp

firfilter.o: firfilter.cpp firfilter.h
	g++ $(CFLAGS) -c firfilter.cpp

kalman.o: kalman.cpp kalman.h
	g++ $(CFLAGS) -c kalman.cpp

rovioKalmanFilter.o: kalmanfilter/rovioKalmanFilter.c
	g++ ${CFLAGS} -Ikalmanfilter -c kalmanfilter/rovioKalmanFilter.c	

utilities.o: utilities.cpp utilities.h
	g++ $(CFLAGS) -c utilities.cpp

PID.o: PID.cpp PID.h
	g++ $(CFLAGS) -c PID.cpp

collectData.o: collectData.cpp
	g++ $(CFLAGS) -c collectData.cpp

clean:
	rm -f *.o
	rm -f *.gch
	rm -f project.out
	rm -f collectData.out
