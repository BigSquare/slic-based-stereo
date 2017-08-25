CXX ?= g++

CXXFLAGS += -c -O3 -Wall $(shell pkg-config --cflags opencv)
LDFLAGS += $(shell pkg-config --libs --static opencv)

all: stereo 

stereo: stereo.o; $(CXX) FileIO.cpp ./SLIC/SLIC.h ./SLIC/SLIC.cpp $< -o $@ $(LDFLAGS) -lstdc++ -fopenmp


%.o: %.cpp gradient.h census.h FileIO.cpp util.h; $(CXX) $< -o $@ $(CXXFLAGS) -lstdc++ -fopenmp
#change in above files trigger the make

clean: ; rm -f stereo.o stereo
