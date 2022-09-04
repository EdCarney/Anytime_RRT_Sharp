#include <unordered_map>
#include <math.h>
#include <fstream>
#include <vector>
#include "cppshrhelp.hpp"
#include "ConfigspaceNode.hpp"
#include "Geometry.hpp"

using namespace std;

#ifndef CONFIGSPACE_GRAPH_H
#define CONFIGSPACE_GRAPH_H

class ConfigspaceGraph : Rectangle
{
    void buildGraph();
    void deleteGraph();

    unordered_map<int, vector<int>> _parentChildMap;

    vector<int> _getAllChildIds(vector<int> ids);
    void _addParentChildRelation(int id);
    void _removeParentChildRelation(int id);
    void _recomputeCost(vector<int> ids);

    // calculate the radius of the ball to consider for the k-nearest neighbor
    double _computeRadius(double epsilon);

    public:
        int numNodeInd;                 // used to set the node id; is NOT modified by pruning
        double minTheta, maxTheta;
        double freeSpaceMeasure;        // a measure of the free space in the graph
        double zeta;                    // the volume of a unit ball in the free space
        double gamma_star;              // optimality constraint calculated from percollation theory
        int dim;                        // dimension of the free space
        unordered_map<int, ConfigspaceNode> nodes;
        vector<Edge> edges;

        void setRootNode(State state);
        int addNode(ConfigspaceNode node);
        vector<ConfigspaceNode> removeNode(vector<ConfigspaceNode>& nodeVec, ConfigspaceNode nodeToRemove);

        void removeEdge(int parentId, int childId);

        // function to replace a node in the current graph node array
        void replaceNode(ConfigspaceNode oldNode, ConfigspaceNode newNode);

        // creates an edge between nodes
        void addEdge(GraphNode parentNode, GraphNode newNode);

        // defines freespace for problem
        // used when extending to a new node
        void defineFreespace(double minX, double minY, double minTheta, double maxX,
            double maxY, double maxTheta, int dimension, double obstacleVol);
        void defineFreespace(Rectangle limits, int dimension, double obstacleVol);
        
        void printData(ConfigspaceNode finalNode);

        ConfigspaceNode findClosestNode(GraphNode node);
        ConfigspaceNode generateRandomNode();
        ConfigspaceNode generateBiasedNode(double biasedX, double biasedY);

        double computeCost(Point p1, Point p2);

        // get the k-nearest neighbors from the current node
        // will not return the centerNode's parent node in the array
        vector<ConfigspaceNode> findNeighbors(GraphNode centerNode, double radius, int k);
        ConfigspaceNode findBestNeighbor(ConfigspaceNode newNode, vector<ConfigspaceNode> safeNeighbors);
        void propagateCost(vector<int> updatedNodeIds);
        void propagateCost(int updatedNodeId);
        ConfigspaceNode extendToNode(GraphNode parentNode, GraphNode newNode, double maxDist);
        ConfigspaceNode connectNodes(ConfigspaceNode parentNode, ConfigspaceNode newNode);

        // default constructor
        ConfigspaceGraph() { buildGraph(); }
};

#endif //CONFIGSPACE_GRAPH_H