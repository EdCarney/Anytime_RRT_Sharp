/*
This graph data structure is heavily based on the graph data structure provided
by Dr. Michael Otte.
*/
#include <stdio.h>
#include <stdlib.h>

#ifndef CONFIGSPACE_H
#define CONFIGSPACE_H

struct BasicNode
{
	double x; double y; double theta; double t;
};

struct ConfigspaceNode
{
	int id;				// unique identifer for the node; 1-indexed
	double x;			// x coordinate location of the vehicle centroid
	double y;			// y coordinate location of the vehicle centroid
	double theta;		// rotation of the vehicle
	double v;			// linear velocity of the vehicle
	double w;			// rotational velocity of the vehicle
	double t;			// time of the node
	double a;			// translational acceleration control input applied 
	double gamma;		// rotational acceleration control input applied
	int parentNodeId;	// the id of the parent node for this node
	double cost;		// cost-to-go for this node

	// iteration point parameters
	// used for collision checking
	ConfigspaceNode* iterationPoints;
	int numIterationPoints;
};

struct Edge
{
	ConfigspaceNode startNode;		// the start node for the edge
	ConfigspaceNode endNode;		// the end node for the edge
};

class ConfigspaceGraph
{
public:

	int numNodes;					// total number of nodes in the graph
	int numEdges;					// total number of edges in the graph

	ConfigspaceNode* nodes;			// an array containing all nodes
	Edge* edges;					// an array containing all edges

	double minX, minY, maxX, maxY;	// limits of the graph freespace
	double minTheta, maxTheta;		// limits of the oreintation theta
	double minV, minW, maxV, maxW;	// limits of the angular and translational velocity
	double maxAbsA, maxAbsGamma;	// absolute limits of the angular and translational acceleration
	double freeSpaceMeasure;		// a meaure of the free space in the graph
	double zeta;					// the volume of a unit ball in the free space
	double gamma_star;				// optimality constraint calculated from percollation theory
	int dim;						// dimension of the free space

private:
	void buildGraph();
	void deleteGraph();

public:

	// creates a node for the graph with position (x,y)
	// NOTE: this should only be used for the start node
	void createNode(double x, double y, double theta, double v, double w, double t);

	// adds a node to the graph
	// NOTE: this should be used for all other nodes
	ConfigspaceNode addNode(ConfigspaceNode node);

	// removes a node from a given array of nodes specified
	// as a pointer array
	ConfigspaceNode* removeNode(ConfigspaceNode* nodeArray, ConfigspaceNode nodeToRemove);

	// function to replace a node in the current graph node array
	void replaceNode(ConfigspaceNode oldNode, ConfigspaceNode newNode);

	// creates an edge between nodes
	void addEdge(ConfigspaceNode parentNode, ConfigspaceNode newNode);

	// removes an edge between nodes
	void removeEdge(ConfigspaceNode parentToRemove, ConfigspaceNode childToRemove);

	// defines freespace for problem
	// used when extending to a new node
	void defineFreespace(double minX, double minY, double minTheta, double minV, double minW,
		double maxX, double maxY, double maxTheta, double maxV, double maxW, double newMaxAbsA, double newMaxAbsGamma,
		int dimension, double obstacleVol);

	// find node from an ID
	ConfigspaceNode findNodeId(int nodeId);

	// find node array placement from an ID
	int findNodePlacement(int nodeId);

	// print data from the graph for displaying
	void printData(int probNum, ConfigspaceNode finalNode);

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
	ConfigspaceNode* findNeighbors(ConfigspaceNode centerNode, double radius, int k, double goalX, double goalY, double goalRadius);

	// find the best node of the provided list of safe nodes to attempt to
	// connect to for RRT*
	ConfigspaceNode findBestNeighbor(ConfigspaceNode newNode, ConfigspaceNode* safeNeighbors);

	// propagate cost udates to a node to all of its children
	void propagateCost(ConfigspaceNode* updatedNode);

	// default constructor
	ConfigspaceGraph() { buildGraph(); }

	// default destructor
	~ConfigspaceGraph() { deleteGraph(); }


	////////////////////////////////////////////////////////


	ConfigspaceNode findClosestNode_basic(ConfigspaceNode node);
	double computeCost_basic(ConfigspaceNode node_1, ConfigspaceNode node_2);
	ConfigspaceNode* findNeighbors_basic(ConfigspaceNode centerNode, double radius, int k);
	ConfigspaceNode findBestNeighbor_basic(ConfigspaceNode newNode, ConfigspaceNode* safeNeighbors);
	ConfigspaceNode addNode_basic(ConfigspaceNode addedNode);
	void propagateCost_basic(ConfigspaceNode* updatedNodes);
	void replaceNode_basic(ConfigspaceNode oldNode, ConfigspaceNode newNode);
};

#endif CONFIGSPACE_H