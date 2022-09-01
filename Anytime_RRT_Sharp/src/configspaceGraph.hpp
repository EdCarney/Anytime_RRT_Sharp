#include <unordered_map>
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
    double _cost;
    void _buildConfigspaceNode();
    void _buildConfigspaceNode(GraphNode node);
    void _buildConfigspaceNode(double x, double y, double theta, int id, int parentId, double cost);

    public:
        ConfigspaceNode();
        ConfigspaceNode(GraphNode node);
        ConfigspaceNode(double x, double y, double theta, int id, int parentId, double cost);
        double cost();
        void setCost(double cost);
};

class ConfigspaceGraph : Rectangle
{
    void buildGraph();
    void deleteGraph();

    unordered_map<int, vector<int>> parentChildMap;

    vector<int> getAllChildIds(vector<int> ids);
    void addParentChildRelation(int id);
    void removeParentChildRelation(int id);
    void recomputeCost(vector<int> ids);
    
    // calculate the radius of the ball to consider for the k-nearest neighbor
    double _computeRadius(double epsilon);

    public:
        int numNodeInd;                    // used to set the node id; is NOT modified by pruning

        unordered_map<int, ConfigspaceNode> nodes;
        //vector<ConfigspaceNode> nodes;            // an array containing all nodes
        vector<Edge> edges;                    // an array containing all edges
        double minTheta, maxTheta;        // limits of the orientation theta
        double freeSpaceMeasure;        // a measure of the free space in the graph
        double zeta;                    // the volume of a unit ball in the free space
        double gamma_star;                // optimality constraint calculated from percollation theory
        int dim;                        // dimension of the free space

        // adds a node to the graph
        // NOTE: this should be used for all other nodes
        int addNode(ConfigspaceNode node);

        // removes a node from a given array of nodes specified
        // as a pointer array
        vector<ConfigspaceNode> removeNode(vector<ConfigspaceNode> nodeVec, ConfigspaceNode nodeToRemove);

        void removeEdge(int parentId, int childId);

        // function to replace a node in the current graph node array
        void replaceNode(ConfigspaceNode oldNode, ConfigspaceNode newNode);

        // creates an edge between nodes
        void addEdge(GraphNode parentNode, GraphNode newNode);

        // defines freespace for problem
        // used when extending to a new node
        void defineFreespace(double minX, double minY, double minTheta, double maxX,
            double maxY, double maxTheta, int dimension, double obstacleVol);

        // print data from the graph for displaying
        void printData(ConfigspaceNode finalNode, int probNum = 1);

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

#endif // CONFIGSPACE_H