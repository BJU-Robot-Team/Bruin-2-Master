CFLAGS=-std=c++11
CC=g++

build:
	mkdir build

coord.o: ../src/coord.cpp build
	$(CC) $(CFLAGS) ../src/coord.cpp -c -o build/coord.o

testCoord: testCoord.cpp coord.o build
	$(CC) $(CFLAGS) testCoord.cpp build/coord.o -o build/testCoord

test: testCoord build
	build/testCoord
