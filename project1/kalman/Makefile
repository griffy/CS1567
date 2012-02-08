all: rovioKalmanFilter_test

CFLAGS=-Wall -ggdb
LIBS=-lgslcblas -L/usr/lib64/atlas -lclapack

rovioKalmanFilter_test: rovioKalmanFilter.o rovioKalmanFilter_test.c
	gcc ${CFLAGS}  -c rovioKalmanFilter_test.c
	gcc ${CFLAGS} -o rovioKalmanFilter_test rovioKalmanFilter_test.o rovioKalmanFilter.o ${LIBS}

rovioKalmanFilter.o: rovioKalmanFilter.c
	gcc ${CFLAGS} -c rovioKalmanFilter.c

clean:
	rm -f *.o rovioKalmanFilter_test TR.csv
