CFLAGS=-std=c++11

build:
	mkdir build

coord.o: ../src/coord.cpp build
	g++ $(CFLAGS) ../src/coord.cpp -c -o build/coord.o

testCoord: testCoord.cpp coord.o build
	g++ $(FLAGS) testCoord.cpp build/coord.o -o build/testCoord

test: testCoord build
	build/testCoord
