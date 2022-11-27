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
    _workspaceGraph.addObstacles(params.obstacles());
    _workspaceGraph.setVehicle(params.vehicle());
}

void ArrtsService::_configureConfigspace(ArrtsParams params)
{
    _configspaceGraph.defineFreespace(params.limits(), params.dimension(), params.obstacleVolume());
    _configspaceGraph.setRootNode(params.start());
}

void ArrtsService::_runAlgorithm(ArrtsParams params, ManeuverType maneuverType)
{
    printf("ObsVol: %f, NumObs: %lu\n", params.obstacleVolume(), params.obstacles().size());
    printf("Freespace Min: [%f, %f, %f], Freespace Max: [%f, %f, %f]\n", params.limits().minPoint().x(), params.limits().minPoint().y(), params.limits().minPoint().z(), params.limits().maxPoint().x(), params.limits().maxPoint().y(), params.limits().maxPoint().z());
    printf("UAV Position: [%f, %f, %f], UAV Orientation [%f, %f]\n", params.goal().x(), params.goal().y(), params.goal().z(), params.goal().theta(), params.goal().rho());
    printf("Root Position: [%f, %f, %f], Root Orientation [%f, %f]\n", params.start().x(), params.start().y(), params.start().z(), params.start().theta(), params.start().rho());
    auto start = high_resolution_clock::now();

    ArrtsEngine::runArrtsOnGraphs(_configspaceGraph, _workspaceGraph, params, maneuverType);

    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);

    _setFinalNode();
    _setFinalPathFromFinalNode();

    printf("Total number of points: %lu\n", _configspaceGraph.nodes.size());
    printf("Final Position: [%f, %f, %f]\n", _finalNode.x(), _finalNode.y(), _finalNode.z());
    printf("Final Cost: %f\n", _finalNode.cost());
    printf("Total Runtime: %lld ms\n", duration.count());
}

vector<State> ArrtsService::calculatePath(ArrtsParams params, string dataExportDir, ManeuverType maneuverType)
{
    _buildDefaultService();
    _configureWorkspace(params);
    _configureConfigspace(params);
    _runAlgorithm(params, maneuverType);
    _exportDataToDirectory(dataExportDir);

    return _path;
}

void ArrtsService::_exportDataToDirectory(string directory)
{
    if (directory.empty())
        return;

    if (!filesystem::is_directory(directory))
        filesystem::create_directory(directory);

    ofstream nodeFile, edgeFile, searchTreeFile, outputPathFile, fullOutputPathFile;

    // initialize all output files
    nodeFile.open(directory + "/nodes.txt");
    edgeFile.open(directory + "/edges.txt");
    searchTreeFile.open(directory + "/search_tree.txt");
    outputPathFile.open(directory + "/output_path.txt");
    fullOutputPathFile.open(directory + "/full_output_path.txt");

    _printGraphNodesToFileStream(_configspaceGraph.nodes, nodeFile);
    _printEdgesToFileStream(_configspaceGraph.edges, edgeFile);
    _printSearchTreeToFileStream(_configspaceGraph.edges, searchTreeFile);

    // print out output path
    ConfigspaceNode currentNode = _configspaceGraph.nodes[_finalNode.id()];
    while (currentNode.parentId())
    {
        _printStatesToFileStream(currentNode.pathTo(), fullOutputPathFile);
        _printStateToFileStream(currentNode, outputPathFile);
        currentNode = _configspaceGraph.nodes.at(currentNode.parentId());
    }
    _printStateToFileStream(_configspaceGraph.nodes.at(1), outputPathFile);
    _printStateToFileStream(_configspaceGraph.nodes.at(1), fullOutputPathFile);

    printf("Printing nodes to %s/nodes.txt.\n", directory.c_str());
    printf("Printing edges to %s/edges.txt.\n", directory.c_str());
    printf("Printing search tree to %s/search_tree.txt.\n", directory.c_str());
    printf("Printing output path to %s/output_path.txt.\n", directory.c_str());

    // close files
    nodeFile.close();
    edgeFile.close();
    searchTreeFile.close();
    outputPathFile.close();
    fullOutputPathFile.close();
}

void ArrtsService::_printGraphNodesToFileStream(const unordered_map<unsigned long, ConfigspaceNode>& nodeMap, ofstream& fileStream) const
{
    for (auto itr = nodeMap.begin(); itr != nodeMap.end(); ++itr)
        _printGraphNodeToFileStream(itr->second, fileStream);
}

void ArrtsService::_printStatesToFileStream(const vector<State>& states, ofstream& fileStream) const
{
    for (State state : states)
        _printStateToFileStream(state, fileStream);
}

void ArrtsService::_printEdgesToFileStream(const vector<Edge> edges, ofstream& fileStream) const
{
    for (Edge edge : edges)
        _printBasicEdgeToFileStream(edge, fileStream);
}

void ArrtsService::_printSearchTreeToFileStream(const vector<Edge> edges, ofstream& fileStream) const
{
    for (Edge edge : edges)
        _printFullEdgeToFileStream(edge, fileStream);
}

void ArrtsService::_printGraphNodeToFileStream(const GraphNode& graphNode, ofstream& fileStream) const
{
    fileStream << graphNode.x() << " " << graphNode.y() << " " << graphNode.z() << " " << graphNode.theta() << " "
        << graphNode.rho() << " " << graphNode.id() << endl;
}

void ArrtsService::_printStateToFileStream(const State& state, ofstream& fileStream) const
{
    fileStream << state.x() << " " << state.y() << " " << state.z() << " "  << state.theta() << " "  << state.rho() << endl;
}

void ArrtsService::_printBasicEdgeToFileStream(const Edge& edge, ofstream& fileStream) const
{
    fileStream << edge.start().id() << " " << edge.end().id() << endl;
}

void ArrtsService::_printFullEdgeToFileStream(const Edge& edge, ofstream& fileStream) const
{
    fileStream << edge.start().id() << " " << edge.start().x() << " " << edge.start().y() << " " << edge.start().z() << " "
        << edge.end().id() << " " << edge.end().x() << " " << edge.end().y() << " " << edge.end().z() << endl;
}