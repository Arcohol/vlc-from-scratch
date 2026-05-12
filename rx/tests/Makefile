CXX ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -pedantic -O2

.PHONY: all test clean

all: sim_rx

sim_rx: sim_rx.cpp ../vlc_rx.cpp ../vlc_rx.h
	$(CXX) $(CXXFLAGS) -I.. sim_rx.cpp ../vlc_rx.cpp -o $@

test: sim_rx
	./sim_rx

clean:
	rm -f sim_rx
