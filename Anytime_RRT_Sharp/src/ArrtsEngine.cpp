#include "ArrtsEngine.hpp"

void ArrtsEngine::_rewireNodes(ConfigspaceGraph& configGraph, WorkspaceGraph& workGraph, vector<ConfigspaceNode>& remainingNodes, ConfigspaceNode& addedNode)
{
    ConfigspaceNode remainingNodeParent, newNode;

    for (ConfigspaceNode rn : remainingNodes)
    {
        // check if it is cheaper for the current remaining node to use the added node as
        // its parent node
        if (!_compareNodes(configGraph, rn, addedNode) && workGraph.pathIsSafe(rn, addedNode))
        {
            // if it's cheaper, then create the new node, set the new cost, and set
            // the parent (now the added node)
            newNode = configGraph.connectNodes(addedNode, rn);

            // get the old parent of the current remaining node, remove the old
            // edge, add the new edge, and replace the old remaining node
            remainingNodeParent = configGraph.nodes[rn.parentId()];
            configGraph.removeEdge(remainingNodeParent.id(), rn.id());
            configGraph.addEdge(addedNode, newNode);
            configGraph.replaceNode(rn, newNode);

            // propagate the cost update from using the new node down the tree
            configGraph.propagateCost(newNode.id());
        }
    }
}

void ArrtsEngine::_tryConnectToBestNeighbor(ConfigspaceGraph& configGraph, vector<ConfigspaceNode>& neighbors, ConfigspaceNode& newNode, ConfigspaceNode& parentNode)
{
    // find the best safe neighbor and connect newNode and the bestNeighbor
    // assign the resulting node to tempNode
    auto bestNeighbor = configGraph.findBestNeighbor(newNode, neighbors);
    auto tempNode = configGraph.connectNodes(bestNeighbor, newNode);

    // if the tempNode is cheaper then make that the newNode
    if (tempNode.cost() < newNode.cost())
    {
        newNode = tempNode;
        parentNode = bestNeighbor;
        configGraph.removeNode(neighbors, bestNeighbor);
    }
}

bool ArrtsEngine::_compareNodes(ConfigspaceGraph& configGraph, ConfigspaceNode& n1, ConfigspaceNode& n2)
{
    if (n1.cost() < (n2.cost() + configGraph.computeCost(n1, n2)))
        return true;
    return false;
}

void ArrtsEngine::runArrtsOnGraphs(ConfigspaceGraph& configGraph, WorkspaceGraph& workGraph, ArrtsParams params)
{
    ConfigspaceNode tempNode, parentNode, newNode;
    vector<ConfigspaceNode> neighbors;
    bool goalRegionReached = false;

    int count = 0, tempId = 0;
    int goalBiasCount = (int)ceil(params.minNodeCount() * 0.01);
    double epsilonToVolRatio = 0.00001;
    double epsilon = workGraph.volume() * epsilonToVolRatio;

    printf("Epsilon/Volume ratio: %f\n", epsilon / workGraph.volume());
    printf("Epsilon: %f\n", epsilon);

    srand(time(NULL));

    while(!goalRegionReached || count < params.minNodeCount())
    {
        // create a new node (not yet connected to the graph)
        tempNode = (count++ % goalBiasCount != 0)
                 ? configGraph.generateRandomNode()
                 : configGraph.generateBiasedNode(workGraph.goalRegion().x(), workGraph.goalRegion().y(), workGraph.goalRegion().z());
        // find the closest graph node and set it as the parent
        parentNode = configGraph.findClosestNode(tempNode);

        // skip if the parent node is already in the goal region
        if (!workGraph.checkAtGoal(parentNode))
        {
            // create a new node by extending from the parent to the temp node
            // (this includes a collision check); then compute cost
            newNode = configGraph.extendToNode(parentNode, tempNode, epsilon);

            // if there is a collision, newNode id will be set to its parent's id
            if (workGraph.nodeIsSafe(newNode) && workGraph.pathIsSafe(newNode, parentNode))
            {
                neighbors = configGraph.findNeighbors(newNode, epsilon, params.maxNeighborCount());

                for (auto itr = neighbors.begin(); itr < neighbors.end(); ++itr)
                    if (!workGraph.pathIsSafe(newNode, *itr))
                        neighbors.erase(itr);

                if (!neighbors.empty())
                {
                    _tryConnectToBestNeighbor(configGraph, neighbors, newNode, parentNode);
                }

                // add new node and edge to the config graph
                tempId = configGraph.addNode(newNode);
                newNode = configGraph.nodes[tempId];
                configGraph.addEdge(parentNode, newNode);

                // if we haven't reached the goal yet and the added node is in the
                // goal region, then set goalRegionReached to true
                if (!goalRegionReached && workGraph.checkAtGoal(newNode))
                    goalRegionReached = true;

                // do the rewiring while there are nodes left in remainingNodes
                _rewireNodes(configGraph, workGraph, neighbors, newNode);
            }
        }
    }
}