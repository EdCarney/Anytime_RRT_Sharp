#include "ArrtsService.hpp"

void ArrtsService::_buildDefaultService()
{
    _path = vector<State>();
    _configspaceGraph = ConfigspaceGraph();
    _workspaceGraph = WorkspaceGraph();
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

void ArrtsService::_setFinalPathFromFinalNode()
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

void ArrtsService::_configureWorkspace(ArrtsParams params)
{
    _workspaceGraph.setGoalRegion(params.goal(), params.goalRadius());
    _workspaceGraph.defineFreespace(params.limits());
    _workspaceGraph.addObstacles(params.sphereObstacles());
    _workspaceGraph.setVehicle(params.vehicle());
}

void ArrtsService::_configureConfigspace(ArrtsParams params)
{
    _configspaceGraph.defineFreespace(params.limits(), params.dimension(), params.obstacleVolume());
    _configspaceGraph.setRootNode(params.start());
}

void ArrtsService::_runAlgorithm(ArrtsParams params)
{
    printf("ObsVol: %f, NumObs: %lu, Freespace: [%f, %f, %f, %f]\n", params.obstacleVolume(), params.sphereObstacles().size(), params.limits().minPoint().x(), params.limits().minPoint().y(), params.limits().maxPoint().x(), params.limits().maxPoint().y());
    printf("UAV Location: %f, %f, %f\n", params.goal().x(), params.goal().y(), params.goal().theta());
    printf("Root Node:    %f, %f, %f\n", params.start().x(), params.start().y(), params.start().theta());
    auto start = high_resolution_clock::now();

    ArrtsEngine::runArrtsOnGraphs(_configspaceGraph, _workspaceGraph, params);

    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);

    _setFinalNode();
    _setFinalPathFromFinalNode();

    printf("Total number of points: %lu\n", _configspaceGraph.nodes.size());
    printf("Final node at: (%f, %f)\n", _finalNode.x(), _finalNode.y());
    printf("Final cost is: %f\n", _finalNode.cost());
    printf("Total runtime is %lld ms\n", duration.count());
}

vector<State> ArrtsService::calculatePath(ArrtsParams params, string dataExportDir)
{
    _buildDefaultService();
    _configureWorkspace(params);
    _configureConfigspace(params);
    _runAlgorithm(params);
    _exportDataToDirectory(dataExportDir);

    return _path;
}

void ArrtsService::_exportDataToDirectory(string directory)
{
    if (directory.empty())
        return;

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
        nodeFile << itr->second.x() << ", " << itr->second.y() << ", " << itr->second.z() << ", " << itr->second.theta() << ", " << itr->first << "\n";

    // print out edge file
    for (int i = 0; i < numEdges; ++i)
        edgeFile << _configspaceGraph.edges[i].start().id() << ", " << _configspaceGraph.edges[i].end().id() << "\n";

    // print out search tree file
    for (int i = 0; i < numEdges; ++i)
        searchTreeFile << _configspaceGraph.edges[i].start().id() << ", " << _configspaceGraph.edges[i].start().x() << ", "
            << _configspaceGraph.edges[i].start().y() << ", " << _configspaceGraph.edges[i].start().z() << ", "
            << _configspaceGraph.edges[i].end().id() << ", " << _configspaceGraph.edges[i].end().x() << ", "
            << _configspaceGraph.edges[i].end().y() << ", " << _configspaceGraph.edges[i].end().z() << "\n";

    // print out output path
    ConfigspaceNode currentNode = _configspaceGraph.nodes[_finalNode.id()];

    outputPathFile << currentNode.x() << ", " << currentNode.y() << ", " << currentNode.z() << ", " << currentNode.theta() << "\n";
    currentNode = _configspaceGraph.nodes[currentNode.parentId()];

    while (currentNode.parentId())
    {
        outputPathFile << currentNode.x() << ", " << currentNode.y() << ", " << currentNode.z() << ", "  << currentNode.theta() << "\n";
        currentNode = _configspaceGraph.nodes[currentNode.parentId()];
    }
    outputPathFile << _configspaceGraph.nodes[1].x() << ", " << _configspaceGraph.nodes[1].y() << ", " << _configspaceGraph.nodes[1].z() << ", "  << _configspaceGraph.nodes[1].theta() << "\n";

    printf("Printing nodes to %s/nodes.txt.\n", directory.c_str());
    printf("Printing edges to %s/edges.txt.\n", directory.c_str());
    printf("Printing search tree to %s/search_tree.txt.\n", directory.c_str());
    printf("Printing output path to %s/output_path.txt.\n", directory.c_str());

    // close files
    nodeFile.close();
    edgeFile.close();
    searchTreeFile.close();
    outputPathFile.close();
}