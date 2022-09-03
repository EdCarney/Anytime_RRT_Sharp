#include <chrono>
#include "ARRTS.hpp"
#include "ConfigspaceGraph.hpp"
#include "ConfigspaceNode.hpp"
#include "WorkspaceGraph.hpp"

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

// iterates through remainingNodes, determines if the addedNode is a better (cheaper) parent, and
// rewires the graphs if it is.
void rewireRemainingNodes(ArrtsService& service, vector<ConfigspaceNode>& remainingNodes, ConfigspaceNode addedNode);

// compares two nodes to determine which has a cheaper cost-to-go;
// returns true if nodeA is cheaper, and false otherwise
bool compareNodes(ConfigspaceNode nodeA, ConfigspaceNode nodeB, ArrtsService& service);

// checks if any of the newNode's (safe) neighbors function as a better (cheaper) parent than
// newNode's current parent, and updates newNode if it is (note this does NOT rewire the graphs
// on this update, and should only be used PRIOR to adding newNode to the graphs)
void tryConnectToBestNeighbor(ArrtsService& service, vector<ConfigspaceNode>& neighbors, ConfigspaceNode& newNode, ConfigspaceNode& parentNode);

// iterates through all nodes in the graphs and finds the node within
// the goal region with the lowest cost-to-go
ConfigspaceNode findBestNode(ArrtsService& service);