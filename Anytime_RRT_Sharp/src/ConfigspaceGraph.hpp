#include <math.h>
#include <fstream>
#include <vector>
#include <unordered_map>
#include "cppshrhelp.hpp"
#include "ConfigspaceNode.hpp"
#include "DubinsEngine.hpp"
#include "Geometry2D.hpp"
#include "Geometry3D.hpp"

using namespace std;

#ifndef CONFIGSPACE_GRAPH_H
#define CONFIGSPACE_GRAPH_H

class ConfigspaceGraph : Rectangle
{
    void buildGraph();
    void deleteGraph();

    unordered_map<unsigned long, vector<unsigned long>> _parentChildMap;

    vector<unsigned long> _getAllChildIds(vector<unsigned long>& ids);
    void _addParentChildRelation(unsigned long id);
    void _removeParentChildRelation(unsigned long id);
    void _recomputeCost(vector<unsigned long>& ids);

    // calculate the radius of the ball to consider for the k-nearest neighbor
    double _computeRadius(double epsilon) const;

    public:
        unsigned long numNodeInd;                 // used to set the node id; is NOT modified by pruning
        double minTheta, maxTheta;
        double gamma_star;              // optimality constraint calculated from percollation theory
        int dim;                        // dimension of the free space
        unordered_map<unsigned long, ConfigspaceNode> nodes;
        vector<Edge> edges;

        void setRootNode(State state);
        int addNode(ConfigspaceNode node);
        vector<ConfigspaceNode>& removeNode(vector<ConfigspaceNode>& nodeVec, ConfigspaceNode& nodeToRemove);

        void removeEdge(unsigned long parentId, unsigned long childId);

        // function to replace a node in the current graph node array
        void replaceNode(ConfigspaceNode oldNode, ConfigspaceNode newNode);

        // creates an edge between nodes
        void addEdge(GraphNode parentNode, GraphNode newNode);

        void defineFreespace(Rectangle limits, int dimension, double obstacleVol);

        ConfigspaceNode& findClosestNode(GraphNode& node);
        ConfigspaceNode generateRandomNode() const;
        ConfigspaceNode generateBiasedNode(double biasedX, double biasedY, double biasedZ, double biasedTheta, double biasedRho) const;

        double computeCost(const State s1, const State s2) const;

        // get the k-nearest neighbors from the current node
        // will not return the centerNode's parent node in the array
        vector<ConfigspaceNode> findNeighbors(GraphNode& centerNode, double radius, int maxNumNeighbors);
        ConfigspaceNode findBestNeighbor(ConfigspaceNode& newNode, vector<ConfigspaceNode>& safeNeighbors);
        void propagateCost(vector<unsigned long>& updatedNodeIds);
        void propagateCost(unsigned long updatedNodeId);
        ConfigspaceNode extendToNode(ConfigspaceNode& parentNode, ConfigspaceNode& newNode, double maxDist) const;
        ConfigspaceNode connectNodes(ConfigspaceNode parentNode, ConfigspaceNode newNode);

        // default constructor
        ConfigspaceGraph() { buildGraph(); }
};

#endif //CONFIGSPACE_GRAPH_H