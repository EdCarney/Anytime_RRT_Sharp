#include "RRT_Sharp.hpp"

using namespace std;
using namespace std::chrono;

int main()
{
    ArrtsService service;

    service.initializeFromDataDirectory("./test");

    const double standOffRange = 5.0;
    const double uavGoalRadius = 2.5;

    // initialize variables for the graph limits
    double xMin = 0.0, yMin = 0.0, xMax = 0.0, yMax = 0.0;
    const double thetaMin = 0.0, thetaMax = 2 * M_PI;

    // define a buffer region and the side length to use when
    // defining the square freespace
    const double buffer = 40.0;

    // seed for random node generation
    srand (time(NULL));

    int dimension = 2;

    #pragma region Primary Function

    // build workspace and configspace graphs
    WorkspaceGraph G_workspace;
    ConfigspaceGraph G_configspace;

    auto gateNode = calcGateNode(service.startState().x(), service.startState().y(), service.startState().theta(), standOffRange);
    G_workspace.setGoalRegion(service.goalState().x(), service.goalState().y(), service.goalState().theta(), uavGoalRadius);
    tie(xMin, xMax, yMin, yMax) = calculateGraphLimits(G_workspace, gateNode, buffer);
    G_workspace.defineFreespace(xMin, yMin, xMax, yMax);
    G_workspace.addObstacles(service.obstacles());
    G_workspace.setVehicle(service.vehicle());

    G_configspace.defineFreespace(xMin, yMin, thetaMin, xMax, yMax, thetaMax, dimension, G_workspace.obstacleVolume());
    G_configspace.addNode(gateNode);

    printf("ObsVol: %f, NumObs: %lu, Freespace: [%f, %f, %f, %f]\n", G_workspace.obstacleVolume(), G_workspace.obstacles().size(), xMin, xMax, yMin, yMax);
    printf("UAV Location: %f, %f, %f\n", G_workspace.goalRegion().x(), G_workspace.goalRegion().y(), G_workspace.goalRegion().radius());
    printf("Root Node:    %f, %f, %f\n", G_configspace.nodes[1].x(), G_configspace.nodes[1].y(), G_configspace.nodes[1].theta());

    //------------------------------------------------------------------------//
    //------------------------start the RRT# iterations-----------------------//
    //------------------------------------------------------------------------//

    ConfigspaceNode tempNode, parentNode, newNode;
    vector<ConfigspaceNode> neighbors;
    bool goalRegionReached = false;
    int goalBiasCount = 100, maxCount = 10000;

    int k = 15, count = 0, tempId = 0;
    double epsilon = 10.0;

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
            newNode = G_configspace.extendToNode(parentNode, tempNode, epsilon);

            // if there is a collision, newNode id will be set to its parent's id
            if (G_workspace.nodeIsSafe(newNode) && G_workspace.pathIsSafe(newNode, parentNode))
            {
                neighbors = G_configspace.findNeighbors(newNode, epsilon, k);

                for (auto itr = neighbors.begin(); itr < neighbors.end(); ++itr)
                    if (!G_workspace.pathIsSafe(newNode, *itr))
                        neighbors.erase(itr);

                if (!neighbors.empty())
                {
                    tryConnectToBestNeighbor(G_configspace, G_workspace, neighbors, newNode, parentNode);
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
                rewireRemainingNodes(G_configspace, G_workspace, neighbors, newNode);
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
    printf("Final cost is: %f\n", finalNode.cost());
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

void tryConnectToBestNeighbor(ConfigspaceGraph& G_configspace, WorkspaceGraph& G_workspace, vector<ConfigspaceNode>& neighbors, ConfigspaceNode& newNode, ConfigspaceNode& parentNode)
{
    // find the best safe neighbor and connect newNode and the bestNeighbor
    // assign the resulting node to tempNode
    ConfigspaceNode bestNeighbor = G_configspace.findBestNeighbor(newNode, neighbors);
    ConfigspaceNode tempNode = G_configspace.connectNodes(bestNeighbor, newNode);

    // if the tempNode is cheaper then make that the newNode
    if (tempNode.cost() < newNode.cost())
    {
        newNode = tempNode;
        parentNode = bestNeighbor;
        G_configspace.removeNode(neighbors, bestNeighbor);
    }
}

void rewireRemainingNodes(ConfigspaceGraph& G_configspace, WorkspaceGraph& G_workspace, vector<ConfigspaceNode>& remainingNodes, ConfigspaceNode addedNode)
{
    ConfigspaceNode remainingNodeParent, newNode;

    for (ConfigspaceNode rn : remainingNodes)
    {
        // check if it is cheaper for the current remaining node to use the added node as
        // its parent node
        if (!compareNodes(rn, addedNode, G_configspace) && G_workspace.pathIsSafe(rn, addedNode))
        {
            // if it's cheaper, then create the new node, set the new cost, and set
            // the parent (now the added node)
            newNode = G_configspace.connectNodes(addedNode, rn);

            // get the old parent of the current remaining node, remove the old
            // edge, add the new edge, and replace the old remaining node
            remainingNodeParent = G_configspace.nodes[rn.parentId()];
            G_configspace.removeEdge(remainingNodeParent.id(), rn.id());
            G_configspace.addEdge(addedNode, newNode);
            G_configspace.replaceNode(rn, newNode);

            // propagate the cost update from using the new node down the tree
            G_configspace.propagateCost(newNode.id());
        }
    }
}

bool compareNodes(ConfigspaceNode nodeA, ConfigspaceNode nodeB, ConfigspaceGraph& G_configspace)
{
    if (nodeA.cost() < (nodeB.cost() + G_configspace.computeCost(nodeA, nodeB)))
        return true;
    return false;
}

ConfigspaceNode calcGateNode(double xPosition, double yPosition, double gateOrientation, double standOffRange)
{
    double x = xPosition + standOffRange * cos(gateOrientation - M_PI);
    double y = yPosition + standOffRange * sin(gateOrientation - M_PI);
    return ConfigspaceNode(x, y, 0, 0, 0, gateOrientation);
}