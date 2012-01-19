CFLAGS=-ggdb -g3
LIB_FLAGS=-L. -lrobot_if
CPP_LIB_FLAGS=${LIB_FLAGS} -lrobot_if++
LIB_LINK=-lhighgui -lcv -lcxcore

all: gather_data

gather_data: gather_data.cpp
	g++ ${CFLAGS} -c gather_data.cpp
	g++ ${CFLAGS} -o gather_data gather_data.o ${CPP_LIB_FLAGS} ${LIB_LINK}

clean:
	rm -rf *.o
	rm -rf gather_data
