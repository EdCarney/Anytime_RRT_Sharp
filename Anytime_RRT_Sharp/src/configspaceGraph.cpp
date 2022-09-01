#include "ConfigspaceGraph.hpp"

void ConfigspaceNode::_buildConfigspaceNode()
{
    _buildGraphNode();
    _cost = 0;
}

void ConfigspaceNode::_buildConfigspaceNode(GraphNode n)
{
    _buildGraphNode(n.x(), n.y(), n.theta(), n.id(), n.parentId());
    _cost = 0;
}

void ConfigspaceNode::_buildConfigspaceNode(double x, double y, double theta, int id, int parentId, double cost)
{
    _buildGraphNode(x, y, theta, id, parentId);
    _cost = cost;
}

ConfigspaceNode::ConfigspaceNode()
{
    _buildConfigspaceNode();
}

ConfigspaceNode::ConfigspaceNode(GraphNode node)
{
    _buildConfigspaceNode(node);
}

ConfigspaceNode::ConfigspaceNode(double x, double y, double theta, int id, int parentId, double cost)
{
    _buildConfigspaceNode(x, y, theta, id, parentId, cost);
}

double ConfigspaceNode::cost()
{
    return _cost;
}

void ConfigspaceNode::setCost(double cost)
{
    _cost = cost;
}

void ConfigspaceGraph::buildGraph()
{
    printf("Constructing a default empty configspace graph.\n");
    numNodeInd = 0;
    _minPoint = Point(0, 0);
    _maxPoint = Point(0, 0);
    minTheta = 0;
    maxTheta = 0;
    freeSpaceMeasure = 0;
    zeta = 0;
    dim = 0;
}

void ConfigspaceGraph::addParentChildRelation(int id)
{
    auto node = nodes[id];
    parentChildMap[node.parentId()].push_back(node.id());
}

void ConfigspaceGraph::removeParentChildRelation(int id)
{
    auto node = nodes[id];
    for (auto itr = parentChildMap[node.parentId()].begin(); itr < parentChildMap[node.parentId()].end(); ++itr)
        if (*itr == id)
            parentChildMap[node.parentId()].erase(itr);
}

void ConfigspaceGraph::defineFreespace(double minX, double minY, double newMinTheta, double maxX, double maxY, double newMaxTheta, int dimension, double obstacleVol)
{
    // set graph parameters
    _minPoint = Point(minX, minY);
    _maxPoint = Point(maxX, maxY);
    minTheta = newMinTheta;
    maxTheta = newMaxTheta;

    // compute dependent variables
    dim = dimension;
    zeta = 3.14159;
    freeSpaceMeasure = ((maxX - minX) * (maxY - minY)) - obstacleVol;
    gamma_star = 2 * pow((1.0 + 1.0 / dim) * (freeSpaceMeasure / (zeta * dim)), 1.0 / float(dim));
}

void ConfigspaceGraph::addEdge(GraphNode parentNode, GraphNode newNode)
{
    edges.push_back(Edge(parentNode, newNode));
}

void ConfigspaceGraph::removeEdge(int parentId, int childId)
{
    for (auto itr = edges.begin(); itr < edges.end(); ++itr)
    {
        if (itr->end().id() == childId && itr->start().id() == parentId)
        {
            edges.erase(itr);
            return;
        }
    }
}

vector<ConfigspaceNode> ConfigspaceGraph::removeNode(vector<ConfigspaceNode> nodeVec, ConfigspaceNode nodeToRemove)
{
    for (auto itr = nodeVec.begin(); itr < nodeVec.end(); ++itr)
    {
        if (itr->id() == nodeToRemove.id())
        {
            nodeVec.erase(itr);
            return nodeVec;
        }
    }

    return nodeVec;
}

ConfigspaceNode ConfigspaceGraph::generateRandomNode()
{
    double randX, randY;

    randX = minX() + (minX() + static_cast <double> (rand())) / (static_cast <double> (RAND_MAX / (maxX() - minX())));
    randY = minY() + (minY() + static_cast <double> (rand())) / (static_cast <double> (RAND_MAX / (maxY() - minY())));

    return ConfigspaceNode(randX, randY, 0, 0, 0, 0);
}

ConfigspaceNode ConfigspaceGraph::generateBiasedNode(double biasedX, double biasedY)
{
    return ConfigspaceNode(biasedX, biasedY, 0, 0, 0, 0);
}

double ConfigspaceGraph::_computeRadius(double epsilon)
{
    double percDist = 0.0, circleRadius = 0.0;

    // calculate distance based on percollation theory

    int numNodes = nodes.size();
    double temp1, temp2, temp3;
    temp1 = log(numNodes) / numNodes;
    temp2 = 1 / float(dim);
    temp3 = pow(temp1, temp2);

    percDist = gamma_star * temp3;

    // set actual extend dist based on the min value
    // of the percDist and the epsilon dist
    circleRadius = percDist < epsilon ? percDist : epsilon;

    return circleRadius;
}

void ConfigspaceGraph::printData(ConfigspaceNode finalNode, int probNum)
{
    std::ofstream nodeFile, edgeFile, searchTreeFile, outputPathFile, highFidelityPath;

    // initialize all output files
    nodeFile.open("nodes_" + std::to_string(probNum) + ".txt");
    edgeFile.open("edges_" + std::to_string(probNum) + ".txt");
    searchTreeFile.open("search_tree_" + std::to_string(probNum) + ".txt");
    outputPathFile.open("output_path_" + std::to_string(probNum) + ".txt");

    int numNodes = nodes.size();
    int numEdges = edges.size();

    // print out node file
    nodeFile << numNodes << "\n";

    for (auto itr = nodes.begin(); itr != nodes.end(); ++itr)
        nodeFile << itr->second.x() << ", " << itr->second.y() << ", " << itr->second.theta() << ", " << itr->first << "\n";

    // print out edge file
    edgeFile << numEdges << "\n";

    for (int i = 0; i < numEdges; ++i)
        edgeFile << edges[i].start().id() << ", " << edges[i].end().id() << "\n";

    // print out search tree file
    for (int i = 0; i < numEdges; ++i)
    {
        searchTreeFile << edges[i].start().id() << ", " << edges[i].start().x() << ", " << edges[i].start().y()
            << ", " << edges[i].end().id() << ", " << edges[i].end().x() << ", " << edges[i].end().y() << "\n";
    }

    // print out output path
    ConfigspaceNode currentNode = finalNode;

    outputPathFile << currentNode.x() << ", " << currentNode.y() << ", " << currentNode.theta() << "\n";
    currentNode = nodes[currentNode.parentId()];

    while (currentNode.parentId())
    {
        outputPathFile << currentNode.x() << ", " << currentNode.y() << ", " << currentNode.theta() << "\n";
        currentNode = nodes[currentNode.parentId()];
    }
    outputPathFile << nodes[1].x() << ", " << nodes[1].y() << ", " << nodes[1].theta() << "\n";

    printf("Printing nodes to nodes_%d.txt.\n", probNum);
    printf("Printing edges to edges_%d.txt.\n", probNum);
    printf("Printing search tree to search_tree_%d.txt.\n", probNum);
    printf("Printing output path to output_path_%d.txt.\n", probNum);

    // close files
    nodeFile.close();
    edgeFile.close();
    searchTreeFile.close();
    outputPathFile.close();
}

ConfigspaceNode ConfigspaceGraph::findClosestNode(GraphNode node)
{
    // initialize distance with first node
    // use euclidean distance of given node from existing nodes
    double dist, shortestDist = INFINITY;
    int closestNodeId = 0;

    // track these in unordered map?
    for (auto itr = nodes.begin(); itr != nodes.end(); ++itr)
    {
        dist = itr->second.distanceTo(node);
        if (dist < shortestDist)
        {
            shortestDist = dist;
            closestNodeId = itr->first;
        }
    }
    return nodes[closestNodeId];
}

double ConfigspaceGraph::computeCost(Point p1, Point p2)
{
    return p1.distanceTo(p2);
}

vector<ConfigspaceNode> ConfigspaceGraph::findNeighbors(GraphNode centerNode, double epsilon, int k)
{
    double dist, radius = _computeRadius(epsilon);
    vector<ConfigspaceNode> neighbors(0);

    for (auto itr = nodes.begin(); itr != nodes.end(); ++itr)
    {
        dist = itr->second.distanceTo(centerNode);
        if (dist < radius && centerNode.parentId() != itr->second.id())
        {
            neighbors.push_back(itr->second);
            if (neighbors.size() >= k)
                return neighbors;
        }
    }
    return neighbors;
}

ConfigspaceNode ConfigspaceGraph::findBestNeighbor(ConfigspaceNode newNode, vector<ConfigspaceNode> safeNeighbors)
{
    ConfigspaceNode bestNeighbor;
    double tempBestCost = 0, bestCost = INFINITY;

    for (ConfigspaceNode n : safeNeighbors)
    {
        tempBestCost = n.cost() + computeCost(newNode, n);
        if (tempBestCost < bestCost)
        {
            bestCost = tempBestCost;
            bestNeighbor = n;
        }
    }
	return bestNeighbor;
}

int ConfigspaceGraph::addNode(ConfigspaceNode node)
{
    node.setId(++numNodeInd);
    nodes[node.id()] = node;
    addParentChildRelation(node.id());
    return node.id();
}

void ConfigspaceGraph::propagateCost(int updatedNodeId)
{
    vector<int> updateNodes(1, updatedNodeId);
    propagateCost(updateNodes);
}

void ConfigspaceGraph::propagateCost(vector<int> updatedNodeIds)
{
    if (updatedNodeIds.empty())
        return;

    vector<int> children = getAllChildIds(updatedNodeIds);
    recomputeCost(children);
    propagateCost(children);
}

vector<int> ConfigspaceGraph::getAllChildIds(vector<int> ids)
{
    vector<int> childIds, tempChildIds;
    for (int id : ids)
    {
        tempChildIds = parentChildMap[id];
        childIds.insert(childIds.begin(), tempChildIds.begin(), tempChildIds.end());
    }
    return childIds;
}

void ConfigspaceGraph::recomputeCost(vector<int> ids)
{
    ConfigspaceNode node, parent;
    for (int id : ids)
    {
        node = nodes[id];
        parent= nodes[node.parentId()];
        node.setCost(parent.cost() + computeCost(node, parent));
    }
}

void ConfigspaceGraph::replaceNode(ConfigspaceNode oldNode, ConfigspaceNode newNode)
{
    removeParentChildRelation(oldNode.id());
    nodes.erase(oldNode.id());

    nodes[newNode.id()] = newNode;
    addParentChildRelation(newNode.id());
}

ConfigspaceNode ConfigspaceGraph::extendToNode(GraphNode parentNode, GraphNode newNode, double maxDist)
{
    ConfigspaceNode currentNode;
    double dist = parentNode.distanceTo(newNode);
    double x, y, theta, cost;

    if (dist >= maxDist)
    {
        x = parentNode.x() + ((newNode.x() - parentNode.x()) / dist) * maxDist;
        y = parentNode.y() + ((newNode.y() - parentNode.y()) / dist) * maxDist;
        theta = 0;
    }
    else
    {
        x = newNode.x();
        y = newNode.y();
        theta = 0;
    }
    cost = nodes[parentNode.id()].cost() + computeCost(parentNode, Point(x, y));

    return ConfigspaceNode(x, y, theta, 0, parentNode.id(), cost);
}

ConfigspaceNode ConfigspaceGraph::connectNodes(ConfigspaceNode parentNode, ConfigspaceNode newNode)
{
    newNode.setParentId(parentNode.id());
    newNode.setCost(parentNode.cost() + computeCost(parentNode, newNode));
    return newNode;
}