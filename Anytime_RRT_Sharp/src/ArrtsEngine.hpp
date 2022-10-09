#include <vector>
#include "ArrtsParams.hpp"
#include "ConfigspaceGraph.hpp"
#include "ConfigspaceNode.hpp"
#include "DubinsEngine.hpp"
#include "Geometry2D.hpp"
#include "Geometry3D.hpp"
#include "WorkspaceGraph.hpp"

#ifndef ARRTS_ENGINE_H
#define ARRTS_ENGINE_H

#define REPORTING_PERCENTILE 5

using namespace std;

class ArrtsEngine
{
    static void _printProgress(int count, int minCount);
    static void _rewireNodes(ConfigspaceGraph& configGraph, WorkspaceGraph& workGraph, vector<ConfigspaceNode>& remainingNodes, ConfigspaceNode& addedNode);
    static void _tryConnectToBestNeighbor(ConfigspaceGraph& configGraph, vector<ConfigspaceNode>& neighbors, ConfigspaceNode& newNode, ConfigspaceNode& parentNode);
    static bool _compareNodes(ConfigspaceGraph& configGraph, ConfigspaceNode& n1, ConfigspaceNode& n2);

    public:
        static void runArrtsOnGraphs(ConfigspaceGraph& configGraph, WorkspaceGraph& workGraph, ArrtsParams params);

};

#endif //ARRTS_ENGINE_H