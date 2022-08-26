#include "configspaceGraph.hpp"

ConfigspaceNode::ConfigspaceNode()
{
    _buildGraphNode();
    cost = 0;
    theta = 0;
}

ConfigspaceNode::ConfigspaceNode(double x, double y, int id, int parentId, double costVal, double thetaVal)
{
    _buildGraphNode(x, y, id, parentId);
    cost = costVal;
    theta = thetaVal;
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
    nodes.reserve(5000);
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

void ConfigspaceGraph::addEdge(ConfigspaceNode parentNode, ConfigspaceNode newNode)
{
    edges.push_back(Edge(parentNode, newNode));
}

void ConfigspaceGraph::removeEdge(ConfigspaceNode parentNode, ConfigspaceNode childNode)
{
    for (auto itr = edges.begin(); itr < edges.end(); itr++)
    {
        if (itr->end().id() == childNode.id() && itr->start().id() == parentNode.id())
        {
            edges.erase(itr);
            return;
        }
    }
}

void ConfigspaceGraph::removeEdgesWithEndNode(ConfigspaceNode node)
{
    for (auto itr = edges.begin(); itr < edges.end(); itr++)
        if (itr->end().id() == node.id())
            edges.erase(itr);
}

vector<ConfigspaceNode> ConfigspaceGraph::removeNode(vector<ConfigspaceNode> nodeVec, ConfigspaceNode nodeToRemove)
{
    for (auto itr = nodeVec.begin(); itr < nodeVec.end(); itr++)
    {
        if (itr->id() == nodeToRemove.id())
        {
            nodeVec.erase(itr);
            return nodeVec;
        }
    }
    return nodeVec;
}

void ConfigspaceGraph::removeGraphNodes(vector<ConfigspaceNode> nodesToRemove)
{
    for (ConfigspaceNode n : nodesToRemove)
    {
        nodes = removeNode(nodes, n);
        removeEdgesWithEndNode(n);
    }
}

void ConfigspaceGraph::createNode(double x, double y, double theta, double t)
{
    nodes.push_back(ConfigspaceNode(x, y, nodes.size() + 1, 0, 0, 0));
}

ConfigspaceNode ConfigspaceGraph::findNodeId(int id)
{
    for (ConfigspaceNode n : nodes)
        if (n.id() == id)
            return n;
    return ConfigspaceNode();
}

int ConfigspaceGraph::findNodePlacement(int id)
{
    int size = nodes.size(), i = 0;
    while (i < size)
        if (nodes[i++].id() == id)
            return i - 1;
    return -1;
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

double ConfigspaceGraph::computeRadius(double epsilon)
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

void ConfigspaceGraph::trimTreeChildren(vector<ConfigspaceNode> removeNodes, int saveNodeId)
{
    vector<ConfigspaceNode> nodesToRemove;

    // get all nodes that have the removeNodes as a parent node
    for (ConfigspaceNode rn : removeNodes)
        for (ConfigspaceNode n : nodes)
            if (rn.id() == n.parentId())
                nodesToRemove.push_back(n);

    // if there are any nodes that have any of the remove nodes as a parent node,
    // then continue until there are no more children
    if (!nodesToRemove.empty())
        trimTreeChildren(nodesToRemove, 0);

    // after all children are obtained, start deleting them in the order of the
    // youngest children first (but save the node if it is the initital node)
    if (removeNodes.front().id() != saveNodeId)
        removeGraphNodes(removeNodes);
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

    for (int i = 0; i < numNodes - 1; ++i)
        nodeFile << nodes[i].x() << ", " << nodes[i].y() << ", " << nodes[i].theta << ", " << nodes[i].id() << "\n";

    nodeFile << nodes[numNodes - 1].x() << ", " << nodes[numNodes - 1].y() << ", "
        << nodes[numNodes - 1].theta << ", " << nodes[numNodes - 1].id() << "\n";

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

    outputPathFile << currentNode.x() << ", " << currentNode.y() << ", " << currentNode.theta << "\n";
    currentNode = findNodeId(currentNode.parentId());

    while (currentNode.parentId())
    {
        outputPathFile << currentNode.x() << ", " << currentNode.y() << ", " << currentNode.theta << "\n";
        currentNode = findNodeId(currentNode.parentId());
    }
    outputPathFile << nodes[0].x() << ", " << nodes[0].y() << ", " << nodes[0].theta << "\n";

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

ConfigspaceNode ConfigspaceGraph::findClosestNode(ConfigspaceNode node)
{
    // initialize distance with first node
    // use euclidean distance of given node from existing nodes
    double dist, shortestDist = INFINITY;
    int closestEntry = 0, numNodes = nodes.size();

    for (int i = 0; i < numNodes; i++)
    {
        dist = hypot(nodes[i].x() - node.x(), nodes[i].y() - node.y());;
        if (dist < shortestDist)
        {
            shortestDist = dist;
            closestEntry = i;
        }
    }
    return nodes[closestEntry];
}

double ConfigspaceGraph::computeCost(ConfigspaceNode node_1, ConfigspaceNode node_2)
{
    return hypot((node_1.x() - node_2.x()), (node_1.y() - node_2.y()));
}

vector<ConfigspaceNode> ConfigspaceGraph::findNeighbors(ConfigspaceNode centerNode, double radius, int k)
{
    double dist;
    vector<ConfigspaceNode> neighbors(0);

    for (ConfigspaceNode n : nodes)
    {
        dist = hypot((centerNode.x() - n.x()), (centerNode.y() - n.y()));
        if (dist < radius && centerNode.parentId() != n.id())
        {
            neighbors.push_back(n);
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
        tempBestCost = n.cost + computeCost(newNode, n);
        if (tempBestCost < bestCost)
        {
            bestCost = tempBestCost;
            bestNeighbor = n;
        }
    }
	return bestNeighbor;
}

void ConfigspaceGraph::addNode(ConfigspaceNode node)
{
    node.setId(++numNodeInd);
	//printf("Adding Node %d at (%f, %f, %f)\n", node.id(), node.x(), node.y(), node.theta);
    nodes.push_back(node);
}

void ConfigspaceGraph::propagateCost(ConfigspaceNode updatedNode)
{
    vector<ConfigspaceNode> updateNodes(1, updatedNode);
    propagateCost(updateNodes);
}

void ConfigspaceGraph::propagateCost(vector<ConfigspaceNode> updatedNodes)
{
    vector<ConfigspaceNode> tempNodesToUpdate, nodesToUpdate;

    if (updatedNodes.empty())
        return;

    for (ConfigspaceNode un : updatedNodes)
    {
        for (ConfigspaceNode n : nodes)
        {
            if (un.id() == n.parentId())
            {
                nodesToUpdate.push_back(n);
                n.cost = un.cost + computeCost(n, un);
            }
        }
    }
 
    propagateCost(nodesToUpdate);
}

void ConfigspaceGraph::replaceNode(ConfigspaceNode oldNode, ConfigspaceNode newNode)
{
    int oldNodePlace = findNodePlacement(oldNode.id());
    nodes[oldNodePlace] = newNode;
}
