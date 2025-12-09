CXX = g++
CXXFLAGS = -O3 -std=c++17 -Iinclude

# Default layout: soa (you can change to aos if you want)
LAYOUT ?= soa

ifeq ($(LAYOUT),aos)
    CXXFLAGS += -DUSE_AOS
endif

ifeq ($(LAYOUT),soa)
    CXXFLAGS += -DUSE_SOA
endif

all: astar

astar: src/main.cpp src/astar.cpp
	$(CXX) $(CXXFLAGS) -o $@ $^

clean:
	rm -f astar
