OBJS=collectdata.o ../robot.o ../pose.o ../firfilter.o ../kalman.o ../rovioKalmanFilter.o ../utilities.o ../logger.o ../PID.o 
CFLAGS=-ggdb -g3
LIB_FLAGS=-L. -lrobot_if
CPP_LIB_FLAGS=$(LIB_FLAGS) -lrobot_if++
LIB_LINK=-lhighgui -lcv -lcxcore -lm -lgslcblas -L/usr/lib64/atlas -lclapack
LIB_LINK_NEW=-lopencv_core -lopencv_imgproc -lopencv_highgui -lm -lgslcblas -L/usr/lib64/atlas -llapack

all: $(OBJS)
	cd ..; make
	g++ $(CFLAGS) -o collectdata.out $(OBJS) $(CPP_LIB_FLAGS) $(LIB_LINK)

new: $(OBJS)
	cd ..; make new
	g++ $(CFLAGS) -o collectdata.out $(OBJS) $(CPP_LIB_FLAGS) $(LIB_LINK_NEW)

collectdata.o: collectdata.cpp
	g++ $(CFLAGS) -c collectdata.cpp

clean:
	rm -f *.o
	rm -f *.gch
	rm -f collectdata.out
