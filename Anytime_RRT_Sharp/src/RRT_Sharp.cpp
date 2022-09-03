#include "RRT_Sharp.hpp"

using namespace std;
using namespace std::chrono;

int main()
{
    ArrtsService service("./test");

    const double uavGoalRadius = 2.5;
    service.calculatePath(uavGoalRadius);

    // seed for random node generation
    srand (time(NULL));

    #pragma region Primary Function

    printf("ObsVol: %f, NumObs: %lu, Freespace: [%f, %f, %f, %f]\n", service.obstacleVolume(), service.obstacles().size(), service.limits().minPoint().x(), service.limits().minPoint().y(), service.limits().maxPoint().x(), service.limits().maxPoint().y());
    printf("UAV Location: %f, %f, %f\n", service.startState().x(), service.startState().y(), service.startState().theta());
    printf("Root Node:    %f, %f, %f\n", service.goalState().x(), service.goalState().y(), service.goalState().theta());

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
        tempNode = (count++ % goalBiasCount != 0) ? service.configspaceGraph().generateRandomNode() : service.configspaceGraph().generateBiasedNode(service.workspaceGraph().goalRegion().x(), service.workspaceGraph().goalRegion().y());
        // find the closest graph node and set it as the parent
        parentNode = service.configspaceGraph().findClosestNode(tempNode);

        // skip if the parent node is already in the goal region
        if (!service.workspaceGraph().checkAtGoal(parentNode))
        {
            // create a new node by extending from the parent to the temp node
            // (this includes a collision check); then compute cost
            newNode = service.configspaceGraph().extendToNode(parentNode, tempNode, epsilon);

            // if there is a collision, newNode id will be set to its parent's id
            if (service.workspaceGraph().nodeIsSafe(newNode) && service.workspaceGraph().pathIsSafe(newNode, parentNode))
            {
                neighbors = service.configspaceGraph().findNeighbors(newNode, epsilon, k);

                for (auto itr = neighbors.begin(); itr < neighbors.end(); ++itr)
                    if (!service.workspaceGraph().pathIsSafe(newNode, *itr))
                        neighbors.erase(itr);

                if (!neighbors.empty())
                {
                    tryConnectToBestNeighbor(service, neighbors, newNode, parentNode);
                }

                // add new node and edge to the config graph
                tempId = service.configspaceGraph().addNode(newNode);
                newNode = service.configspaceGraph().nodes[tempId];
                service.configspaceGraph().addEdge(parentNode, newNode);

                // if we haven't reached the goal yet and the added node is in the
                // goal region, then set goalRegionReached to true
                if (!goalRegionReached && service.workspaceGraph().checkAtGoal(newNode))
                    goalRegionReached = true;

                // do the rewiring while there are nodes left in remainingNodes
                rewireRemainingNodes(service, neighbors, newNode);
            }
        }
    }

    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);

    // find the best node in the goal region
    ConfigspaceNode finalNode = findBestNode(service);

    // print all the results to files
    printf("Total number of points: %lu\n", service.configspaceGraph().nodes.size());
    printf("Final node at: (%f, %f)\n", finalNode.x(), finalNode.y());
    printf("Final cost is: %f\n", finalNode.cost());
    printf("Total runtime is %lld ms\n", duration.count());
    service.configspaceGraph().printData(finalNode);

    return 0;
}

#pragma endregion Primary code for the Anytime RRT# implementation

ConfigspaceNode findBestNode(ArrtsService& service)
{
    double tempCost = 0, finalCost = INFINITY;
    ConfigspaceNode finalNode;
    
    for (auto itr = service.configspaceGraph().nodes.begin(); itr != service.configspaceGraph().nodes.end(); ++itr)
    {
        if (service.workspaceGraph().checkAtGoal(itr->second))
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

void tryConnectToBestNeighbor(ArrtsService& service, vector<ConfigspaceNode>& neighbors, ConfigspaceNode& newNode, ConfigspaceNode& parentNode)
{
    // find the best safe neighbor and connect newNode and the bestNeighbor
    // assign the resulting node to tempNode
    ConfigspaceNode bestNeighbor = service.configspaceGraph().findBestNeighbor(newNode, neighbors);
    ConfigspaceNode tempNode = service.configspaceGraph().connectNodes(bestNeighbor, newNode);

    // if the tempNode is cheaper then make that the newNode
    if (tempNode.cost() < newNode.cost())
    {
        newNode = tempNode;
        parentNode = bestNeighbor;
        service.configspaceGraph().removeNode(neighbors, bestNeighbor);
    }
}

void rewireRemainingNodes(ArrtsService& service, vector<ConfigspaceNode>& remainingNodes, ConfigspaceNode addedNode)
{
    ConfigspaceNode remainingNodeParent, newNode;

    for (ConfigspaceNode rn : remainingNodes)
    {
        // check if it is cheaper for the current remaining node to use the added node as
        // its parent node
        if (!compareNodes(rn, addedNode, service) && service.workspaceGraph().pathIsSafe(rn, addedNode))
        {
            // if it's cheaper, then create the new node, set the new cost, and set
            // the parent (now the added node)
            newNode = service.configspaceGraph().connectNodes(addedNode, rn);

            // get the old parent of the current remaining node, remove the old
            // edge, add the new edge, and replace the old remaining node
            remainingNodeParent = service.configspaceGraph().nodes[rn.parentId()];
            service.configspaceGraph().removeEdge(remainingNodeParent.id(), rn.id());
            service.configspaceGraph().addEdge(addedNode, newNode);
            service.configspaceGraph().replaceNode(rn, newNode);

            // propagate the cost update from using the new node down the tree
            service.configspaceGraph().propagateCost(newNode.id());
        }
    }
}

bool compareNodes(ConfigspaceNode nodeA, ConfigspaceNode nodeB, ArrtsService& service)
{
    if (nodeA.cost() < (nodeB.cost() + service.configspaceGraph().computeCost(nodeA, nodeB)))
        return true;
    return false;
}