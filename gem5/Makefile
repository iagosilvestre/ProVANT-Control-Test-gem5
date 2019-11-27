CXX ?= 	g++
NAVIO = ../../Navio
INCLUDES = -I ../../Navio

all:
	$(MAKE) -C ../../Navio all
	$(CXX) $(INCLUDES) gem5.cpp -L$(NAVIO) -lnavio -o gem5 -static

clean:
	rm -f gem5
