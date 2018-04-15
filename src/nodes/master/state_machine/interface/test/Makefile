CFLAGS=-std=c++11
CC=g++

build:
	mkdir build

build/coord.o: ../src/coord.cpp build
	$(CC) $(CFLAGS) ../src/coord.cpp -c -o build/coord.o

build/config.o: ../src/config.cpp build
	$(CC) $(CFLAGS) ../src/config.cpp -c -o build/config.o

testCoord: testCoord.cpp build/coord.o build/config.o build
	$(CC) $(CFLAGS) testCoord.cpp build/coord.o build/config.o -o build/testCoord

test: testCoord build
	build/testCoord
