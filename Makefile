all: kalmanstandalone
kalmanstandalone: kalmanstandalone.cpp
	g++ -I/usr/include/eigen3 -std=c++14 kalmanstandalone.cpp -o kalmanstandalone

run:
	./kalmanstandalone
