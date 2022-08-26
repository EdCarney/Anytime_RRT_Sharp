#include <math.h>
#include <fstream>
#include <cstring>
#include <vector>
#include "cppshrhelp.hpp"
#include "Geometry.hpp"

using namespace std;

#ifndef CONFIGSPACE_H
#define CONFIGSPACE_H

class ConfigspaceNode : public GraphNode
{
    public:
        ConfigspaceNode();
        ConfigspaceNode(double x, double y, int id, int parentId, double costVal, double thetaVal);
        double theta;
        double cost;        // cost-to-go for this node
};

class ConfigspaceGraph : Rectangle
{
    void buildGraph();
    void deleteGraph();

    public:
        int numNodeInd;                    // used to set the node id; is NOT modified by pruning

        vector<ConfigspaceNode> nodes;            // an array containing all nodes
        vector<Edge> edges;                    // an array containing all edges
        double minTheta, maxTheta;        // limits of the orientation theta
        double freeSpaceMeasure;        // a measure of the free space in the graph
        double zeta;                    // the volume of a unit ball in the free space
        double gamma_star;                // optimality constraint calculated from percollation theory
        int dim;                        // dimension of the free space

        // creates a node for the graph with position (x,y)
        // NOTE: this should only be used for the start node
        void createNode(double x, double y, double theta, double t);

        // adds a node to the graph
        // NOTE: this should be used for all other nodes
        ConfigspaceNode addNode(ConfigspaceNode node);

        // removes a node from a given array of nodes specified
        // as a pointer array
        vector<ConfigspaceNode> removeNode(vector<ConfigspaceNode> nodeArray, ConfigspaceNode nodeToRemove);

        // removes a set of nodes from the graph (i.e. the nodes array)
        void removeGraphNodes(vector<ConfigspaceNode> nodesToRemove);

        // function to replace a node in the current graph node array
        void replaceNode(ConfigspaceNode oldNode, ConfigspaceNode newNode);

        // creates an edge between nodes
        void addEdge(ConfigspaceNode parentNode, ConfigspaceNode newNode);

        // removes an edge between nodes
        void removeEdge(ConfigspaceNode parentNode, ConfigspaceNode childNode);
        void removeEdgesWithEndNode(ConfigspaceNode node);

        // defines freespace for problem
        // used when extending to a new node
        void defineFreespace(double minX, double minY, double minTheta, double maxX,
            double maxY, double maxTheta, int dimension, double obstacleVol);

        // find node from an ID
        ConfigspaceNode findNodeId(int nodeId);

        // find node array placement from an ID
        int findNodePlacement(int nodeId);

        // print data from the graph for displaying
        void printData(ConfigspaceNode finalNode, int probNum = 1);

        // finds the node closest to the given node
        // returns pointer to the closest node
        ConfigspaceNode findClosestNode(ConfigspaceNode node);

        // generates a random node in the graph freespace
        ConfigspaceNode generateRandomNode();

        // generates a random node in the graph freespace
        ConfigspaceNode generateBiasedNode(double biasedX, double biasedY);

        // calculate the cost between two nodes
        double computeCost(ConfigspaceNode node_1, ConfigspaceNode node_2);

        // calculate the radius of the ball to consider for the k-nearest neighbor
        double computeRadius(double epsilon);

        // get the k-nearest neighbors from the current node
        // will not return the centerNode's parent node in the array
        vector<ConfigspaceNode> findNeighbors(ConfigspaceNode centerNode, double radius, int k);

        // find the best node of the provided list of safe nodes to attempt to
        // connect to for RRT*
        ConfigspaceNode findBestNeighbor(ConfigspaceNode newNode, vector<ConfigspaceNode> safeNeighbors);

        // propagate cost updates to a node to all of its children
        void propagateCost(vector<ConfigspaceNode> updatedNodes);
		void propagateCost(ConfigspaceNode updatedNode);

        // recursively deletes a given set of nodes from the graph, starting
        // with the nodes' children first
        void trimTreeChildren(vector<ConfigspaceNode> removeNodes, int saveNodeId);

        // default constructor
        ConfigspaceGraph() { buildGraph(); }
};

#endif // CONFIGSPACE_H