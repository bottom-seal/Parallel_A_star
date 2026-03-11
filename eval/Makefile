CXX = g++
CXXFLAGS = -O3 -std=c++17 -I../include -pthread

PROFILE ?= 0
ifeq ($(PROFILE),1)
    CXXFLAGS += -pg -g
endif

LAYOUT ?= soa
ifeq ($(LAYOUT),aos)
    CXXFLAGS += -DUSE_AOS
endif
ifeq ($(LAYOUT),soa)
    CXXFLAGS += -DUSE_SOA
endif

# Build the pthread-capable version, but choose thread count at runtime (-t / --threads)
CXXFLAGS += -DPTHREAD_ASTAR

TARGET = astar
SRCS   = main.cpp astar.cpp

all: $(TARGET)

$(TARGET): $(SRCS)
	$(CXX) $(CXXFLAGS) -o $@ $^

clean:
	rm -f $(TARGET) gmon.out
