#include "RRT_Sharp.hpp"

using namespace std;
using namespace std::chrono;

int main()
{
    #pragma region Initialization 
    //------------------------------------------------------------------------//
    //-------this info should be ingested from the initialization script-------//
    //------------------------------------------------------------------------//

    // define arrays for the gate and obstacle information
    const double rootXPosition = 5.0;
    const double rootYPosition = 60.0;
    const double rootApproach = - M_PI / 2.0;

    const double obstacleXPosition[] = { 80, 73, 63, 53, 43, 33, 28, 25, 25, 25, 25, 35, 40, 45, 80, 85, 90, 95, 100, 100, 100, 100, 60 };
    const double obstacleYPosition[] = { 40, 30, 25, 25, 26, 25, 35, 47, 57, 67, 77, 80, 80, 80, 80, 80, 80, 80, 80, 0, 5, 10, 100 };
    const double obstacleRadius[]    = { 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8 };
    const int numObstacles = sizeof(obstacleRadius) / sizeof(double);

    // define additional input parameters for the goal node calculation
    const double standOffRange = 5.0;

    // initial UAV orientation
    const double uavStartX = 100.0, uavStartY = 60.0, uavStartTheta = 0.0;

    // goal region (UAV position) radius
    const double uavGoalRadius = 2.5;

    // vehicle rigid body information
    const double vehicleX[] = { -0.5, 0.5, 0.5, -0.5 };
    const double vehicleY[] = { -0.5, -0.5, 0.5, 0.5 };
    int numVehiclePoints = sizeof(vehicleX) / sizeof(double);

    // initialize variables for the graph limits
    double xMin = 0.0, yMin = 0.0, xMax = 0.0, yMax = 0.0;
    const double thetaMin = 0.0, thetaMax = 2 * M_PI;

    // define a buffer region and the side length to use when
    // defining the square freespace
    const double buffer = 40.0;

    // seed for random node generation
    srand (time(NULL));

    //------------------------------------------------------------------------//
    //------------------this info must be declared regardless-----------------//
    //------------------------------------------------------------------------//
    double obsVol = 0.0;
    int dimension = 2;
    int goalBiasCount = 100;
    int maxCount = 10000;

    ConfigspaceNode gateNode, tempNode, parentNode, newNode;
    vector<ConfigspaceNode> neighbors, safeNeighbors, remainingNodes;

    int k = 10, count = 0, tempId = 0;
    double circleRadius = 0.0, epsilon = 10.0;

    #pragma endregion Initializes all necessary variables (could be read-in from file)

    #pragma region Primary Function

    // build workspace and configspace graphs
    WorkspaceGraph G_workspace;
    ConfigspaceGraph G_configspace;

    // set the goal region (i.e. the UAV starting location)
    G_workspace.setGoalRegion(uavStartX, uavStartY, uavStartTheta, uavGoalRadius);

    // function to determine goal node based on approximate gate information
    gateNode = calcGateNode(rootXPosition, rootYPosition, rootApproach, standOffRange);

    // define the limits of the graph based on position of the gate
    // and the robot
    tie(xMin, xMax, yMin, yMax) = calculateGraphLimits(G_workspace, gateNode, buffer);

    G_workspace.defineFreespace(xMin, yMin, xMax, yMax);

    // set the obstacles for this iteration (we don't need to consider all obstacles
    // for every iteration); use the above defined freespace limits
    for (int i = 0; i < numObstacles; ++i)
        if (G_workspace.obstacleInFreespace(obstacleXPosition[i], obstacleYPosition[i], obstacleRadius[i]))
            G_workspace.addObstacle(obstacleXPosition[i], obstacleYPosition[i], obstacleRadius[i]);

    // compute the volume of the obstacles
    obsVol = G_workspace.computeObsVol();

    // set freespace of graphs based on the graph limits; some values will always
    // be the same (theta, v, w, a, gamma), while others will vary (x and y)
    G_configspace.defineFreespace(xMin, yMin, thetaMin, xMax, yMax, thetaMax, dimension, obsVol);

    // add gateNode to the graph
    G_configspace.addNode(gateNode);

    // add vehicle to the graph
    G_workspace.setVehicle(Vehicle(vehicleX, vehicleY, numVehiclePoints));

    printf("ObsVol: %f, NumObs: %lu, Freespace: [%f, %f, %f, %f]\n", obsVol, G_workspace.obstacles().size(), xMin, xMax, yMin, yMax);
    printf("UAV Location: %f, %f, %f\n", G_workspace.goalRegion().x(), G_workspace.goalRegion().y(), G_workspace.goalRegion().radius());
    printf("Root Node:    %f, %f, %f\n", G_configspace.nodes[1].x(), G_configspace.nodes[1].y(), G_configspace.nodes[1].theta);

    //------------------------------------------------------------------------//
    //------------------------start the RRT# iterations-----------------------//
    //------------------------------------------------------------------------//

    bool goalRegionReached = false;

    // do the RRT# thing

    auto start = high_resolution_clock::now();

    while(!goalRegionReached || count < maxCount)
    {
        // create a new node (not yet connected to the graph)
        tempNode = (count++ % goalBiasCount != 0) ? G_configspace.generateRandomNode() : G_configspace.generateBiasedNode(G_workspace.goalRegion().x(), G_workspace.goalRegion().y());

        // find the closest graph node and set it as the parent
        parentNode = G_configspace.findClosestNode(tempNode);

        // skip if the parent node is already in the goal region
        if (!G_workspace.checkAtGoal(parentNode))
        {
            // create a new node by extending from the parent to the temp node
            // (this includes a collision check); then compute cost
            newNode = G_workspace.extendToNode(parentNode, tempNode, epsilon);
            newNode.cost = parentNode.cost + G_configspace.computeCost(parentNode, newNode);

            // if there is a collision, newNode id will be set to its parent's id
            if (newNode.id() != parentNode.id())
            {
                // compute ball radius and find k safe neighbor nodes (i.e. no collision)
                // within that ball
                circleRadius = G_configspace.computeRadius(epsilon);
                neighbors = G_configspace.findNeighbors(newNode, circleRadius, k);
                safeNeighbors = G_workspace.checkSafety(newNode, neighbors);

                // if there were no safe neighbors then the first node id will be zero
                if (!safeNeighbors.empty())
                {
                    // reset the remaining nodes to the safe neighbors minus the one we connected to
                    remainingNodes = tryConnectToBestNeighbor(G_configspace, G_workspace, safeNeighbors, newNode, parentNode);
                }

                // add new node and edge to the config graph
                tempId = G_configspace.addNode(newNode);
                newNode = G_configspace.nodes[tempId];
                G_configspace.addEdge(parentNode, newNode);

                // if we haven't reached the goal yet and the added node is in the
                // goal region, then set goalRegionReached to true
                if (!goalRegionReached && G_workspace.checkAtGoal(newNode))
                    goalRegionReached = true;

                // do the rewiring while there are nodes left in remainingNodes
                rewireRemainingNodes(G_configspace, G_workspace, remainingNodes, newNode);
                remainingNodes.clear();
            }
        }
    }

    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);

    // find the best node in the goal region
    ConfigspaceNode finalNode = findBestNode(G_configspace, G_workspace);

    // print all the results to files
    printf("Total number of points: %lu\n", G_configspace.nodes.size());
    printf("Final node at: (%f, %f)\n", finalNode.x(), finalNode.y());
    printf("Final cost is: %f\n", finalNode.cost);
    printf("Total runtime is %lld ms\n", duration.count());
    G_configspace.printData(finalNode);

    return 0;
}

#pragma endregion Primary code for the Anytime RRT# implementation

tuple<double, double, double, double> calculateGraphLimits(WorkspaceGraph G_workspace, ConfigspaceNode gateNode, double buffer)
{
    double xMin, xMax, yMin, yMax;
    if (gateNode.x() < G_workspace.goalRegion().x())
    {
        xMin = gateNode.x() - buffer;
        xMax = G_workspace.goalRegion().x() + buffer;
    }
    else
    {
        xMin = G_workspace.goalRegion().x() - buffer;
        xMax = gateNode.x() + buffer;
    }

    if (gateNode.y() < G_workspace.goalRegion().y())
    {
        yMin = gateNode.y() - buffer;
        yMax = G_workspace.goalRegion().y() + buffer;
    }
    else
    {
        yMin = G_workspace.goalRegion().y() - buffer;
        yMax = gateNode.y() + buffer;
    }
    return make_tuple(xMin, xMax, yMin, yMax);
}

ConfigspaceNode findBestNode(ConfigspaceGraph& G_configspace, WorkspaceGraph& G_workspace)
{
    double tempCost = 0, finalCost = INFINITY;
    ConfigspaceNode finalNode;
    
    for (auto itr = G_configspace.nodes.begin(); itr != G_configspace.nodes.end(); ++itr)
    {
        if (G_workspace.checkAtGoal(itr->second))
        {
            tempCost = itr->second.cost;
            if (tempCost)
            {
                finalCost = tempCost;
                finalNode = itr->second;
            }
        }
    }
    return finalNode;
}

vector<ConfigspaceNode> tryConnectToBestNeighbor(ConfigspaceGraph& G_configspace, WorkspaceGraph& G_workspace, vector<ConfigspaceNode> safeNearestNeighbors, ConfigspaceNode& newNode, ConfigspaceNode& parentNode)
{
    // find the best safe neighbor and connect newNode and the bestNeighbor
    // assign the resulting node to tempNode
    ConfigspaceNode bestNeighbor = G_configspace.findBestNeighbor(newNode, safeNearestNeighbors);
    ConfigspaceNode tempNode = G_workspace.connectNodes(bestNeighbor, newNode);

    // compute the cost of tempNode using the best neighbor
    tempNode.cost = bestNeighbor.cost + G_configspace.computeCost(bestNeighbor, tempNode);

    // if the tempNode is cheaper then make that the newNode
    if (tempNode.cost < newNode.cost)
    {
        newNode = tempNode;
        parentNode = bestNeighbor;
    }

    // reset the remaining nodes to the safe neighbors minus the one we connected to
    return G_configspace.removeNode(safeNearestNeighbors, bestNeighbor);
}

void rewireRemainingNodes(ConfigspaceGraph& G_configspace, WorkspaceGraph& G_workspace, vector<ConfigspaceNode> remainingNodes, ConfigspaceNode addedNode)
{
    ConfigspaceNode remainingNodeParent, newNode;

    for (ConfigspaceNode rn : remainingNodes)
    {
        // check if it is cheaper for the current remaining node to use the added node as
        // its parent node
        if (!compareNodes(rn, addedNode, G_configspace))
        {
            // if it's cheaper, then create the new node, set the new cost, and set
            // the parent (now the added node)
            newNode = G_workspace.connectNodes(addedNode, rn);
            newNode.cost = addedNode.cost + G_configspace.computeCost(rn, addedNode);

            // get the old parent of the current remaining node, remove the old
            // edge, add the new edge, and replace the old remaining node
            remainingNodeParent = G_configspace.nodes[rn.parentId()];
            G_configspace.removeEdge(remainingNodeParent.id(), rn.id());
            G_configspace.addEdge(addedNode, newNode);
            G_configspace.replaceNode(rn, newNode);

            // propagate the cost update from using the new node down the tree
            // (this requires adding the new node to a small pointer array)
            G_configspace.propagateCost(newNode);
        }
    }
}

bool compareNodes(ConfigspaceNode nodeA, ConfigspaceNode nodeB, ConfigspaceGraph& G_configspace)
{
    if (nodeA.cost < (nodeB.cost + G_configspace.computeCost(nodeA, nodeB)))
        return true;
    return false;
}

ConfigspaceNode calcGateNode(double xPosition, double yPosition, double gateOrientation, double standOffRange)
{
    double x = xPosition + standOffRange * cos(gateOrientation - M_PI);
    double y = yPosition + standOffRange * sin(gateOrientation - M_PI);;
    return ConfigspaceNode(x, y, 0, 0, 0, gateOrientation);
}