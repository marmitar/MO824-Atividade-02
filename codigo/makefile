CC := g++
LDFLAGS := -lgurobi_c++ -lgurobi -lgurobi95

ifneq ($(strip $(SIZE)),)
GRAPH_SIZE := -DGRAPH_SIZE=$(SIZE)
endif

ifneq ($(strip $(NDEBUG)),)
CXXFLAGS := -std=gnu++2b -Wall -Werror -Wpedantic -Wunused-result -O0 -ggdb3
else
CXXFLAGS := -std=gnu++2b -Wall -Werror -Wpedantic -Wunused-result -march=native -mtune=native -pipe -fivopts -Ofast -fmodulo-sched -fwhole-program -fno-plt -fno-PIC -fPIE -ffast-math -flto -fuse-linker-plugin -DNDEBUG -D_FORTIFY_SOURCE=1
endif

modelo: modelo.cpp
	$(CC) $(CXXFLAGS) $(GRAPH_SIZE) $^ -o $@ $(LDFLAGS)
