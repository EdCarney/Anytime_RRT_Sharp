#include "ConfigspaceGraph.hpp"

double randInRange(double min, double max)
{
    return min + (min + static_cast <double> (rand())) / (static_cast <double> (RAND_MAX / (max - min)));
}

void ConfigspaceGraph::buildGraph()
{
    numNodeInd = 0;
    _minPoint = Point(0, 0, 0);
    _maxPoint = Point(0, 0, 0);
    minTheta = 0;
    maxTheta = 0;
    dim = 0;
}

void ConfigspaceGraph::_addParentChildRelation(int id)
{
    auto node = nodes[id];
    _parentChildMap[node.parentId()].push_back(node.id());
}

void ConfigspaceGraph::_removeParentChildRelation(int id)
{
    auto node = nodes[id];
    for (auto itr = _parentChildMap[node.parentId()].begin(); itr < _parentChildMap[node.parentId()].end(); ++itr)
        if (*itr == id)
            _parentChildMap[node.parentId()].erase(itr);
}

void ConfigspaceGraph::defineFreespace(Rectangle limits, int dimension, double obstacleVol)
{
    // set graph parameters
    _minPoint = limits.minPoint();
    _maxPoint = limits.maxPoint();
    minTheta = 0;
    maxTheta = 2.0 * M_PI;

    double zeta = 4 * M_PI / 3;                                 // the volume of a unit ball in the free space
    double freeSpaceMeasure = limits.volume() - obstacleVol;    // a measure of the free space in the graph

    // compute dependent variables
    dim = dimension;
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

vector<ConfigspaceNode>& ConfigspaceGraph::removeNode(vector<ConfigspaceNode>& nodeVec, ConfigspaceNode& nodeToRemove)
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

ConfigspaceNode ConfigspaceGraph::generateRandomNode() const
{
    double randX, randY, randZ, randTheta, randRho;

    randX = randInRange(minX(), maxX());
    randY = randInRange(minY(), maxY());
    randZ = randInRange(minZ(), maxZ());
    randTheta = randInRange(0, 2 * M_PI);
    randRho = randInRange(- M_PI / 6.0, M_PI / 6.0);

    return ConfigspaceNode(randX, randY, randZ, randTheta, randRho, 0, 0, 0);
}

ConfigspaceNode ConfigspaceGraph::generateBiasedNode(double biasedX, double biasedY, double biasedZ) const
{
    return ConfigspaceNode(biasedX, biasedY, biasedZ, 0, 0, 0, 0, 0);
}

double ConfigspaceGraph::_computeRadius(double epsilon) const
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

ConfigspaceNode& ConfigspaceGraph::findClosestNode(GraphNode& node)
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

double ConfigspaceGraph::computeCost(Point p1, Point p2) const
{
    return p1.distanceTo(p2);
}

vector<ConfigspaceNode> ConfigspaceGraph::findNeighbors(GraphNode& centerNode, double epsilon, int k)
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

ConfigspaceNode ConfigspaceGraph::findBestNeighbor(ConfigspaceNode& newNode, vector<ConfigspaceNode>& safeNeighbors)
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

void ConfigspaceGraph::setRootNode(State state)
{
    nodes.clear();
    _parentChildMap.clear();
    numNodeInd = 0;
    addNode(ConfigspaceNode(state.x(), state.y(), state.z(), state.theta(), state.rho(), numNodeInd, 0, 0));
}

int ConfigspaceGraph::addNode(ConfigspaceNode node)
{
    node.setId(++numNodeInd);
    nodes[node.id()] = node;
    nodes[node.id()].setPathTo(node.pathTo());
    _addParentChildRelation(node.id());
    return node.id();
}

void ConfigspaceGraph::propagateCost(int updatedNodeId)
{
    vector<int> updateNodes(1, updatedNodeId);
    propagateCost(updateNodes);
}

void ConfigspaceGraph::propagateCost(vector<int>& updatedNodeIds)
{
    if (updatedNodeIds.empty())
        return;

    vector<int> children = _getAllChildIds(updatedNodeIds);
    _recomputeCost(children);
    propagateCost(children);
}

vector<int> ConfigspaceGraph::_getAllChildIds(vector<int>& ids)
{
    vector<int> childIds, tempChildIds;
    for (int id : ids)
    {
        tempChildIds = _parentChildMap[id];
        childIds.insert(childIds.begin(), tempChildIds.begin(), tempChildIds.end());
    }
    return childIds;
}

void ConfigspaceGraph::_recomputeCost(vector<int>& ids)
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
    _removeParentChildRelation(oldNode.id());
    nodes.erase(oldNode.id());

    nodes[newNode.id()] = newNode;
    _addParentChildRelation(newNode.id());
}

ConfigspaceNode ConfigspaceGraph::extendToNode(ConfigspaceNode& parentNode, ConfigspaceNode& newNode, double maxDist) const
{
    double dist = parentNode.distanceTo(newNode);
    double x, y, z, cost;

    if (dist >= maxDist)
    {
        x = parentNode.x() + ((newNode.x() - parentNode.x()) / dist) * maxDist;
        y = parentNode.y() + ((newNode.y() - parentNode.y()) / dist) * maxDist;
        z = parentNode.z() + ((newNode.z() - parentNode.z()) / dist) * maxDist;
    }
    else
    {
        x = newNode.x();
        y = newNode.y();
        z = newNode.z();
    }
    cost = nodes.at(parentNode.id()).cost() + computeCost(parentNode, Point(x, y, z));

    ConfigspaceNode temp(x, y, z, newNode.theta(), newNode.rho(), 0, parentNode.id(), cost);
    temp.generatePathFrom(parentNode);

    return temp;
}

ConfigspaceNode ConfigspaceGraph::connectNodes(ConfigspaceNode parentNode, ConfigspaceNode newNode)
{
    newNode.setParentId(parentNode.id());
    newNode.setCost(parentNode.cost() + computeCost(parentNode, newNode));
    return newNode;
}