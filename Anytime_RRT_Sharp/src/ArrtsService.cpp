#include "ArrtsService.hpp"

void ArrtsService::_buildDefaultService()
{
    _path = vector<State>();
    _configspaceGraph = ConfigspaceGraph();
    _workspaceGraph = WorkspaceGraph();
}

void ArrtsService::_configureWorkspace(ArrtsParams params)
{
    _workspaceGraph.setGoalRegion(params.goal(), params.goalRadius());
    _workspaceGraph.defineFreespace(params.limits());
    _workspaceGraph.addObstacles(params.obstacles());
    _workspaceGraph.setVehicle(params.vehicle());
}

void ArrtsService::_configureConfigspace(ArrtsParams params)
{
    _configspaceGraph.defineFreespace(params.limits(), params.dimension(), params.obstacleVolume());
    _configspaceGraph.setRootNode(params.start());
}

void ArrtsService::_runAlgorithm(ArrtsParams params)
{
    printf("ObsVol: %f, NumObs: %lu, Freespace: [%f, %f, %f, %f]\n", params.obstacleVolume(), params.obstacles().size(), params.limits().minPoint().x(), params.limits().minPoint().y(), params.limits().maxPoint().x(), params.limits().maxPoint().y());
    printf("UAV Location: %f, %f, %f\n", params.goal().x(), params.goal().y(), params.goal().theta());
    printf("Root Node:    %f, %f, %f\n", params.start().x(), params.start().y(), params.start().theta());

    ConfigspaceNode tempNode, parentNode, newNode;
    vector<ConfigspaceNode> neighbors;
    bool goalRegionReached = false;

    int count = 0, tempId = 0;
    int goalBiasCount = (int)ceil(params.minNodeCount() * 0.01);
    double epsilon = 10.0;

    srand(time(NULL));

    auto start = high_resolution_clock::now();

    while(!goalRegionReached || count < params.minNodeCount())
    {
        // create a new node (not yet connected to the graph)
        tempNode = (count++ % goalBiasCount != 0) ? _configspaceGraph.generateRandomNode() : _configspaceGraph.generateBiasedNode(_workspaceGraph.goalRegion().x(), _workspaceGraph.goalRegion().y());
        // find the closest graph node and set it as the parent
        parentNode = _configspaceGraph.findClosestNode(tempNode);

        // skip if the parent node is already in the goal region
        if (!_workspaceGraph.checkAtGoal(parentNode))
        {
            // create a new node by extending from the parent to the temp node
            // (this includes a collision check); then compute cost
            newNode = _configspaceGraph.extendToNode(parentNode, tempNode, epsilon);

            // if there is a collision, newNode id will be set to its parent's id
            if (_workspaceGraph.nodeIsSafe(newNode) && _workspaceGraph.pathIsSafe(newNode, parentNode))
            {
                neighbors = _configspaceGraph.findNeighbors(newNode, epsilon, params.maxNeighborCount());

                for (auto itr = neighbors.begin(); itr < neighbors.end(); ++itr)
                    if (!_workspaceGraph.pathIsSafe(newNode, *itr))
                        neighbors.erase(itr);

                if (!neighbors.empty())
                {
                    _tryConnectToBestNeighbor(neighbors, newNode, parentNode);
                }

                // add new node and edge to the config graph
                tempId = _configspaceGraph.addNode(newNode);
                newNode = _configspaceGraph.nodes[tempId];
                _configspaceGraph.addEdge(parentNode, newNode);

                // if we haven't reached the goal yet and the added node is in the
                // goal region, then set goalRegionReached to true
                if (!goalRegionReached && _workspaceGraph.checkAtGoal(newNode))
                    goalRegionReached = true;

                // do the rewiring while there are nodes left in remainingNodes
                _rewireNodes(neighbors, newNode);
            }
        }
    }

    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);

    // find the best node in the goal region
    _setFinalNode();
    _setFinalPath();

    // print all the results to files
    printf("Total number of points: %lu\n", _configspaceGraph.nodes.size());
    printf("Final node at: (%f, %f)\n", _finalNode.x(), _finalNode.y());
    printf("Final cost is: %f\n", _finalNode.cost());
    printf("Total runtime is %lld ms\n", duration.count());
}

void ArrtsService::_rewireNodes(vector<ConfigspaceNode>& remainingNodes, ConfigspaceNode& addedNode)
{
    ConfigspaceNode remainingNodeParent, newNode;

    for (ConfigspaceNode rn : remainingNodes)
    {
        // check if it is cheaper for the current remaining node to use the added node as
        // its parent node
        if (!_compareNodes(rn, addedNode) && _workspaceGraph.pathIsSafe(rn, addedNode))
        {
            // if it's cheaper, then create the new node, set the new cost, and set
            // the parent (now the added node)
            newNode = _configspaceGraph.connectNodes(addedNode, rn);

            // get the old parent of the current remaining node, remove the old
            // edge, add the new edge, and replace the old remaining node
            remainingNodeParent = _configspaceGraph.nodes[rn.parentId()];
            _configspaceGraph.removeEdge(remainingNodeParent.id(), rn.id());
            _configspaceGraph.addEdge(addedNode, newNode);
            _configspaceGraph.replaceNode(rn, newNode);

            // propagate the cost update from using the new node down the tree
            _configspaceGraph.propagateCost(newNode.id());
        }
    }
}

void ArrtsService::_tryConnectToBestNeighbor(vector<ConfigspaceNode>& neighbors, ConfigspaceNode& newNode, ConfigspaceNode& parentNode)
{
    // find the best safe neighbor and connect newNode and the bestNeighbor
    // assign the resulting node to tempNode
    auto bestNeighbor = _configspaceGraph.findBestNeighbor(newNode, neighbors);
    auto tempNode = _configspaceGraph.connectNodes(bestNeighbor, newNode);

    // if the tempNode is cheaper then make that the newNode
    if (tempNode.cost() < newNode.cost())
    {
        newNode = tempNode;
        parentNode = bestNeighbor;
        _configspaceGraph.removeNode(neighbors, bestNeighbor);
    }
}

void ArrtsService::_setFinalPath()
{
    auto node = _finalNode;

    _path.clear();
    _path.push_back(node);

    while (node.parentId())
    {
        node = _configspaceGraph.nodes[node.parentId()];
        _path.push_back(node);
    }

    _path.push_back(_configspaceGraph.nodes[1]);
}

bool ArrtsService::_compareNodes(ConfigspaceNode n1, ConfigspaceNode n2)
{
    if (n1.cost() < (n2.cost() + _configspaceGraph.computeCost(n1, n2)))
        return true;
    return false;
}

void ArrtsService::_setFinalNode()
{
    double tempCost = 0, finalCost = INFINITY;
    ConfigspaceNode finalNode;
    
    for (auto itr = _configspaceGraph.nodes.begin(); itr != _configspaceGraph.nodes.end(); ++itr)
    {
        if (_workspaceGraph.checkAtGoal(itr->second))
        {
            tempCost = itr->second.cost();
            if (tempCost)
            {
                finalCost = tempCost;
                finalNode = itr->second;
            }
        }
    }
    _finalNode = finalNode;
}

vector<State> ArrtsService::calculatePath(ArrtsParams params)
{
    _buildDefaultService();
    _configureWorkspace(params);
    _configureConfigspace(params);
    _runAlgorithm(params);

    return _path;
}

void ArrtsService::exportDataToDirectory(string directory)
{
    ofstream nodeFile, edgeFile, searchTreeFile, outputPathFile, highFidelityPath;

    // initialize all output files
    nodeFile.open(directory + "/nodes.txt");
    edgeFile.open(directory + "/edges.txt");
    searchTreeFile.open(directory + "/search_tree.txt");
    outputPathFile.open(directory + "/output_path.txt");

    int numNodes = _configspaceGraph.nodes.size();
    int numEdges = _configspaceGraph.edges.size();

    // print out node file
    for (auto itr = _configspaceGraph.nodes.begin(); itr != _configspaceGraph.nodes.end(); ++itr)
        nodeFile << itr->second.x() << ", " << itr->second.y() << ", " << itr->second.theta() << ", " << itr->first << "\n";

    // print out edge file
    for (int i = 0; i < numEdges; ++i)
        edgeFile << _configspaceGraph.edges[i].start().id() << ", " << _configspaceGraph.edges[i].end().id() << "\n";

    // print out search tree file
    for (int i = 0; i < numEdges; ++i)
        searchTreeFile << _configspaceGraph.edges[i].start().id() << ", " << _configspaceGraph.edges[i].start().x() << ", "
            << _configspaceGraph.edges[i].start().y() << ", " << _configspaceGraph.edges[i].end().id() << ", " <<
            _configspaceGraph.edges[i].end().x() << ", " << _configspaceGraph.edges[i].end().y() << "\n";

    // print out output path
    ConfigspaceNode currentNode = _configspaceGraph.nodes[_finalNode.id()];

    outputPathFile << currentNode.x() << ", " << currentNode.y() << ", " << currentNode.theta() << "\n";
    currentNode = _configspaceGraph.nodes[currentNode.parentId()];

    while (currentNode.parentId())
    {
        outputPathFile << currentNode.x() << ", " << currentNode.y() << ", " << currentNode.theta() << "\n";
        currentNode = _configspaceGraph.nodes[currentNode.parentId()];
    }
    outputPathFile << _configspaceGraph.nodes[1].x() << ", " << _configspaceGraph.nodes[1].y() << ", " << _configspaceGraph.nodes[1].theta() << "\n";

    printf("Printing nodes to %s/nodes.txt.\n", directory.c_str());
    printf("Printing edges to %sedges.txt.\n", directory.c_str());
    printf("Printing search tree to %ssearch_tree.txt.\n", directory.c_str());
    printf("Printing output path to %soutput_path.txt.\n", directory.c_str());

    // close files
    nodeFile.close();
    edgeFile.close();
    searchTreeFile.close();
    outputPathFile.close();
}