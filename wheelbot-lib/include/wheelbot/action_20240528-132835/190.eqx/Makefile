CC = g++
CXXFLAGS  = -std=c++20  -g -Wall -O3 -I -lstdc++ -mavx2 -msse2 -mfma -march=znver3
CXXFLAGS += -I/usr/local/include
CXXFLAGS += -I/usr/include

DEPS = neural_network.hpp
OBJ = main.o

%.o: %.c $(DEPS)
	$(CC) $(CXXFLAGS) -c -o $@ $<

main: $(OBJ)
	$(CC) $(CXXFLAGS) -o $@ $^
