CXX=clang++
CXXFLAGS=-g --std=c++17 -Wall -pedantic

BIN=bin

RRT_Sharp:
	$(CXX) -c -g -o ./bin/configspaceGraph.o ./src/configspaceGraph.cpp
	$(CXX) -c -g -o ./bin/workspaceGraph.o ./src/workspaceGraph.cpp
	$(CXX) --std=c++17 -o ./bin/RRT_Sharp ./src/RRT_Sharp.cpp ./bin/configspaceGraph.o ./bin/workspaceGraph.o

clean:
	rm -rf $(BIN)
	mkdir $(BIN)