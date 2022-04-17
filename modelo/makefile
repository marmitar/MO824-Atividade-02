CC := g++
LDFLAGS := -lgurobi_c++ -lgurobi -lgurobi95

ifneq ($(strip $(DEBUG)),)
CXXFLAGS := -std=gnu++2b -Wall -Werror -Wpedantic -Wunused-result -O0 -ggdb3 -DDEBUG
else
CXXFLAGS := -std=gnu++2b -Wall -Werror -Wpedantic -Wunused-result -Ofast -DNDEBUG -D_FORTIFY_SOURCE=1 \
	-march=native -mtune=native -pipe -fivopts  -fmodulo-sched -fwhole-program -fno-plt -fno-PIC -fPIE -ffast-math -flto -fuse-linker-plugin
endif

modelo: main.cpp argparse.hpp elimination.hpp graph.hpp vertex.hpp coordinates.hpp
	$(CC) $(CXXFLAGS) $< -o $@ $(LDFLAGS)


CLONE := git clone
ARGPARSE_URL := https://github.com/p-ranav/argparse.git

argparse:
	$(CLONE) $(ARGPARSE_URL) argparse
	cp argparse/include/argparse/argparse.hpp argparse.hpp
	rm -rf argparse


PYTHON := python3

coordinates.hpp: ../coordinates.py ../coordenadas.txt
	$(PYTHON) $^ > $@
