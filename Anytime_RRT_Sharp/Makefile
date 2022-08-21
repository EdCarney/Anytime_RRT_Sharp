CXX=clang++
CXXFLAGS=--std=c++17 -g -Wall -pedantic

BIN=bin

RRT_Sharp:
	$(CXX) $(CXXFLAGS) -c -o ./bin/configspaceGraph.o ./src/configspaceGraph.cpp
	$(CXX) $(CXXFLAGS) -c -o ./bin/workspaceGraph.o ./src/workspaceGraph.cpp
	$(CXX) $(CXXFLAGS) -o ./bin/RRT_Sharp ./src/RRT_Sharp.cpp ./bin/configspaceGraph.o ./bin/workspaceGraph.o

clean:
	rm -rf $(BIN)
	mkdir $(BIN)