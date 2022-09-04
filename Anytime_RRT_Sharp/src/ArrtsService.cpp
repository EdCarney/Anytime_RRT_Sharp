#include "ArrtsService.hpp"

ArrtsService::ArrtsService() { }

ArrtsService::ArrtsService(string dataDirectory)
{
    initializeFromDataDirectory(dataDirectory);
}

void ArrtsService::_calculateObstacleVolume()
{
    _obstacleVolume = 0.0;
    for (Obstacle o : _obstacles)
        _obstacleVolume += o.area();
}

void ArrtsService::_updateLimitsFromStates()
{
    double minX, maxX, minY, maxY;

    // include buffer percentage
    double bufferX = abs(startState().x() - goalState().x());
    double bufferY = abs(startState().y() - goalState().y());
    double buffer = max(bufferX, bufferY);
    buffer *= 0.5;

    if (startState().x() < goalState().x())
    {
        minX = startState().x() - buffer;
        maxX = goalState().x() + buffer;
    }
    else
    {
        minX = goalState().x() - buffer;
        maxX = startState().x() + buffer;
    }

    if (startState().y() < goalState().y())
    {
        minY = startState().y() - buffer;
        maxY = goalState().y() + buffer;
    }
    else
    {
        minY = goalState().y() - buffer;
        maxY = startState().y() + buffer;
    }
    
    setLimits(minX, minY, maxX, maxY);
}

void ArrtsService::_removeObstaclesNotInLimits()
{
    vector<Obstacle> newObstacles;
    for (Obstacle o : obstacles())
        if (o.intersects(_limits))
            newObstacles.push_back(o);
    _obstacles = newObstacles;
    _calculateObstacleVolume();
}

void ArrtsService::_configureWorkspace()
{
    _workspaceGraph.setGoalRegion(goalState(), _goalRadius);
    _workspaceGraph.defineFreespace(limits());
    _workspaceGraph.addObstacles(obstacles());
    _workspaceGraph.setVehicle(vehicle());
}

void ArrtsService::_configureConfigspace()
{
    _configspaceGraph.defineFreespace(_limits, _dimension, _obstacleVolume);
    _configspaceGraph.setRootNode(startState());
}

void ArrtsService::_runAlgorithm()
{
    printf("ObsVol: %f, NumObs: %lu, Freespace: [%f, %f, %f, %f]\n", _obstacleVolume, obstacles().size(), limits().minPoint().x(), limits().minPoint().y(), limits().maxPoint().x(), limits().maxPoint().y());
    printf("UAV Location: %f, %f, %f\n", goalState().x(), goalState().y(), goalState().theta());
    printf("Root Node:    %f, %f, %f\n", startState().x(), startState().y(), startState().theta());

    ConfigspaceNode tempNode, parentNode, newNode;
    vector<ConfigspaceNode> neighbors;
    bool goalRegionReached = false;

    int count = 0, tempId = 0;
    double epsilon = 10.0;

    srand(time(NULL));

    auto start = high_resolution_clock::now();

    while(!goalRegionReached || count < _maxCount)
    {
        // create a new node (not yet connected to the graph)
        tempNode = (count++ % _goalBiasCount != 0) ? _configspaceGraph.generateRandomNode() : _configspaceGraph.generateBiasedNode(_workspaceGraph.goalRegion().x(), _workspaceGraph.goalRegion().y());
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
                neighbors = _configspaceGraph.findNeighbors(newNode, epsilon, _maxNumNeighbors);

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
    ConfigspaceNode finalNode = _findBestNode();
    _getFinalPath();

    // print all the results to files
    printf("Total number of points: %lu\n", _configspaceGraph.nodes.size());
    printf("Final node at: (%f, %f)\n", finalNode.x(), finalNode.y());
    printf("Final cost is: %f\n", finalNode.cost());
    printf("Total runtime is %lld ms\n", duration.count());
    _configspaceGraph.printData(finalNode);
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

void ArrtsService::_getFinalPath()
{
    auto node = _findBestNode();

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

ConfigspaceNode ArrtsService::_findBestNode()
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
    return finalNode;
}

State ArrtsService::goalState() const
{
    return _goalState;
}

void ArrtsService::setGoalState(double x, double y, double theta)
{
    _goalState = { x, y, theta };
}

State ArrtsService::startState() const
{
    return _startState;
}

void ArrtsService::setStartState(double x, double y, double theta)
{
    _startState = { x, y, theta };
}

Rectangle ArrtsService::limits() const
{
    return _limits;
}

void ArrtsService::setLimits(Point minPoint, Point maxPoint)
{
    _limits = Rectangle(minPoint, maxPoint);
}

void ArrtsService::setLimits(double minX, double minY, double maxX, double maxY)
{
    _limits = Rectangle(minX, minY, maxX, maxY);
}

Vehicle ArrtsService::vehicle() const
{
    return _vehicle;
}

void ArrtsService::setVehicle(vector<double> x, vector<double> y)
{
    _vehicle = Vehicle(x, y);
}

ConfigspaceGraph& ArrtsService::configspaceGraph()
{
    return _configspaceGraph;
}

WorkspaceGraph& ArrtsService::workspaceGraph()
{
    return _workspaceGraph;
}

vector<Obstacle> ArrtsService::obstacles() const
{
    return _obstacles;
}

Obstacle ArrtsService::obstacles(int i) const
{
    if (i >= _obstacles.size() || i < 0)
        throw runtime_error("Attempt to read index beyond array limits in GetObstacle");

    return _obstacles[i];
}

void ArrtsService::addObstacle(double x, double y, double radius)
{
    _obstacles.push_back(Obstacle(x, y, radius));
    _calculateObstacleVolume();
}

void ArrtsService::addObstacles(const vector<double>& x, const vector<double>& y, const vector<double>& r)
{
    int size = x.size();
    for (int i = 0; i < size; ++i)
        _obstacles.push_back(Obstacle(x[i], y[i], r[i]));
    _calculateObstacleVolume();
}

void ArrtsService::readStatesFromFile(FILE* file)
{
    if (file == NULL)
        throw runtime_error("NULL file pointer in readStatesFromFile()");

    double startX, startY, startTheta;
    double goalX, goalY, goalTheta;

    // ignore first line (formatting)
    fscanf(file, "%*[^\n]\n");
    fscanf(file, "%lf,%lf,%lf", &startX, &startY, &startTheta);
    fscanf(file, "%lf,%lf,%lf", &goalX, &goalY, &goalTheta);
    fclose(file);

    setStartState(startX, startY, startTheta);
    setGoalState(goalX, goalY, goalTheta);
}

void ArrtsService::readVehicleFromFile(FILE* file)
{
    _vehicle = Vehicle(file);
}

void ArrtsService::readObstaclesFromFile(FILE* file)
{
    if (file == NULL)
        throw runtime_error("NULL file pointer in readObstaclesFromFile()");

    double xVal, yVal, rVal;
    vector<double> x, y, r;

    // ignore first line (formatting)
    fscanf(file, "%*[^\n]\n");
    while (fscanf(file, "%lf,%lf,%lf", &xVal, &yVal, &rVal) != EOF)
    {
        x.push_back(xVal);
        y.push_back(yVal);
        r.push_back(rVal);
    }
    fclose(file);
    addObstacles(x, y, r);
}

void ArrtsService::initializeFromDataDirectory(string dataDirectory)
{
    string statesFile = dataDirectory + "/" + DEFAULT_STATES_FILE;
    string vehicleFile = dataDirectory + "/" + DEFAULT_VEHICLE_FILE;
    string obstaclesFile = dataDirectory + "/" + DEFAULT_OBSTACLES_FILE;

    readStatesFromFile(fopen(statesFile.c_str(), "r"));
    readVehicleFromFile(fopen(vehicleFile.c_str(), "r"));
    readObstaclesFromFile(fopen(obstaclesFile.c_str(), "r"));

    _updateLimitsFromStates();
    _removeObstaclesNotInLimits();
}

vector<State> ArrtsService::calculatePath(double goalRadius, int maxCount, int goalBiasCount, int maxNumNeighbors)
{
    _goalRadius = goalRadius;
    _maxCount = maxCount;
    _goalBiasCount = goalBiasCount;
    _maxNumNeighbors = maxNumNeighbors;

    _configureWorkspace();
    _configureConfigspace();

    _runAlgorithm();

    return _path;
}