CXX ?= g++
NAVIO = ../../Navio
INCLUDES = -I ../../Navio

all:
	$(MAKE) -C ../../Navio all
	$(CXX) $(INCLUDES) control.cpp -L$(NAVIO) -lnavio -o control -static

clean:
	rm -f control
