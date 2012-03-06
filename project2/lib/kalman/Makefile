CFLAGS=-ggdb -g3
LIB_LINK=-lgslcblas -L/usr/lib64/atlas -lclapack
LIB_LINK_NEW=-lgslcblas -L/usr/lib64/atlas -llapack

all: rovioKalmanFilter.o rovioKalmanFilter_test.c
	g++ ${CFLAGS} -c rovioKalmanFilter_test.c
	g++ ${CFLAGS} -o rovioKalmanFilter_test rovioKalmanFilter_test.o rovioKalmanFilter.o ${LIB_LINK}

new: rovioKalmanFilter.o rovioKalmanFilter_test.c
	g++ ${CFLAGS} -c rovioKalmanFilter_test.c
	g++ ${CFLAGS} -o rovioKalmanFilter_test rovioKalmanFilter_test.o rovioKalmanFilter.o ${LIB_LINK_NEW}

rovioKalmanFilter.o: rovioKalmanFilter.c
	g++ ${CFLAGS} -c rovioKalmanFilter.c

clean:
	rm -f *.o rovioKalmanFilter_test TR.csv
