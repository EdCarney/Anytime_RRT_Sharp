#include <tuple>
#include "configspaceGraph.hpp"
#include "workspaceGraph.hpp"

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

// computes the gate (root) node based on the approximate gate location
// and orientation; outputs a ConfigspaceNode to add to the graph
ConfigspaceNode calcGateNode(double xPosition, double yPosition, double gateOrientation, double standOffRange);

// iterates through remainingNodes, determines if the addedNode is a better (cheaper) parent, and
// rewires the graphs if it is.
void rewireRemainingNodes(ConfigspaceGraph& G_configspace, WorkspaceGraph& G_workspace, vector<ConfigspaceNode> remainingNodes, ConfigspaceNode addedNode);

// compares two nodes to determine which has a cheaper cost-to-go;
// returns true if nodeA is cheaper, and false otherwise
bool compareNodes(ConfigspaceNode nodeA, ConfigspaceNode nodeB, ConfigspaceGraph& G_configspace);

// checks if any of the newNode's (safe) neighbors function as a better (cheaper) parent than
// newNode's current parent, and updates newNode if it is (note this does NOT rewire the graphs
// on this update, and should only be used PRIOR to adding newNode to the graphs)
vector<ConfigspaceNode> tryConnectToBestNeighbor(ConfigspaceGraph& G_configspace, WorkspaceGraph& G_workspace, vector<ConfigspaceNode> safeNearestNeighbors, ConfigspaceNode& newNode, ConfigspaceNode& parentNode);

// iterates through all nodes in the graphs and finds the node within
// the goal region with the lowest cost-to-go
ConfigspaceNode findBestNode(ConfigspaceGraph& G_configspace, WorkspaceGraph& G_workspace);

// calculates the absolute limites for the x and y graph coordinates based on the gateNode, graph
// goal region, and buffer; returns a tuple of (xMin, xMax, yMin, yMax)
tuple<double, double, double, double> calculateGraphLimits(WorkspaceGraph G_workspace, ConfigspaceNode gateNode, double buffer);