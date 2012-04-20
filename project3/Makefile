OBJS=project.o robot.o map_strategy.o path.o map.o cell.o camera.o wheel_encoders.o north_star.o position_sensor.o pose.o fir_filter.o kalman_filter.o rovioKalmanFilter.o utilities.o logger.o PID.o
CFLAGS=-ggdb -g3
LIB_FLAGS=-L. -lrobot_if
CPP_LIB_FLAGS=$(LIB_FLAGS) -lrobot_if++
LIB_LINK=-lhighgui -lcv -lcxcore -lm -lgslcblas -L/usr/lib64/atlas -lclapack
LIB_LINK_NEW=-lopencv_core -lopencv_imgproc -lopencv_highgui -lm -lgslcblas -L/usr/lib64/atlas -llapack

all: $(OBJS) constants.h
	g++ $(CFLAGS) -o project.out $(OBJS) $(CPP_LIB_FLAGS) $(LIB_LINK)

new: $(OBJS) 
	g++ $(CFLAGS) -o project.out $(OBJS) $(CPP_LIB_FLAGS) $(LIB_LINK_NEW)

project.o: project.cpp
	g++ $(CFLAGS) -c project.cpp

robot.o: robot.cpp robot.h
	g++ $(CFLAGS) -c robot.cpp

map_strategy.o: map_strategy.cpp map_strategy.h
	g++ $(CFLAGS) -c map_strategy.cpp

path.o: path.cpp path.h
	g++ $(CFLAGS) -c path.cpp
	
map.o: map.cpp map.h
	g++ $(CFLAGS) -c map.cpp

cell.o: cell.cpp cell.h
	g++ $(CFLAGS) -c cell.cpp
	
camera.o: camera.cpp camera.h
	g++ $(CFLAGS) -c camera.cpp

position_sensor.o: position_sensor.cpp position_sensor.h
	g++ $(CFLAGS) -c position_sensor.cpp

wheel_encoders.o: wheel_encoders.cpp wheel_encoders.h
	g++ $(CFLAGS) -c wheel_encoders.cpp

north_star.o: north_star.cpp north_star.h
	g++ $(CFLAGS) -c north_star.cpp

pose.o: pose.cpp pose.h
	g++ $(CFLAGS) -c pose.cpp

fir_filter.o: fir_filter.cpp fir_filter.h
	g++ $(CFLAGS) -c fir_filter.cpp

kalman_filter.o: kalman_filter.cpp kalman_filter.h
	g++ $(CFLAGS) -c kalman_filter.cpp

rovioKalmanFilter.o: lib/kalman/rovioKalmanFilter.c
	g++ ${CFLAGS} -Ilib/kalman -c lib/kalman/rovioKalmanFilter.c	

utilities.o: utilities.cpp utilities.h
	g++ $(CFLAGS) -c utilities.cpp

PID.o: PID.cpp PID.h
	g++ $(CFLAGS) -c PID.cpp

logger.o: logger.cpp logger.h
	g++ $(CFLAGS) -c logger.cpp

clean:
	rm -f *.o
	rm -f *.gch
	rm -f project.out
