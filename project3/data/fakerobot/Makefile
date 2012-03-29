OBJS=fake_robot_runner.o fake_robot_interface.o ../../fir_filter.o
CFLAGS=-ggdb -g3
LIB_FLAGS=-L. -lrobot_if
CPP_LIB_FLAGS=$(LIB_FLAGS) -lrobot_if++
LIB_LINK=-lhighgui -lcv -lcxcore
LIB_LINK_NEW=-lopencv_core -lopencv_imgproc -lopencv_highgui -lm

all: $(OBJS)
	g++ $(CFLAGS) -o fake_robot_runner.out $(OBJS) $(CPP_LIB_FLAGS) $(LIB_LINK)

new: $(OBJS)
	g++ $(CFLAGS) -o fake_robot_runner.out $(OBJS) $(CPP_LIB_FLAGS) $(LIB_LINK_NEW)

fake_robot_runner.o: fake_robot_runner.cpp fake_robot_interface.h
	g++ $(CFLAGS) -c fake_robot_runner.cpp

fake_robot_interface.o: fake_robot_interface.cpp fake_robot_interface.h
	g++ $(CFLAGS) -c fake_robot_interface.cpp

clean:
	rm -f *.o
	rm -f fake_robot_runner.out
