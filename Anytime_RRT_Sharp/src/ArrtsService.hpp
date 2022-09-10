#include <chrono>
#include <string>
#include <vector>
#include "ArrtsParams.hpp"
#include "ConfigspaceGraph.hpp"
#include "ConfigspaceNode.hpp"
#include "cppshrhelp.hpp"
#include "Geometry.hpp"
#include "WorkspaceGraph.hpp"

using namespace std;
using namespace std::chrono;

#ifndef ARRTS_SERVICE_H
#define ARRTS_SERVICE_H

class DLL_EXPORT ArrtsService
{
    private:
        vector<State> _path;
        ConfigspaceGraph _configspaceGraph;
        WorkspaceGraph _workspaceGraph;
        ConfigspaceNode _finalNode;

        void _buildDefaultService();

        void _configureWorkspace(ArrtsParams params);
        void _configureConfigspace(ArrtsParams params);
        void _runAlgorithm(ArrtsParams params);

        void _rewireNodes(vector<ConfigspaceNode>& remainingNodes, ConfigspaceNode& addedNode);
        void _tryConnectToBestNeighbor(vector<ConfigspaceNode>& neighbors, ConfigspaceNode& newNode, ConfigspaceNode& parentNode);
        void _setFinalPath();
        void _setFinalNode();
        bool _compareNodes(ConfigspaceNode n1, ConfigspaceNode n2);

    public:
        vector<State> DLL_EXPORT calculatePath(ArrtsParams params);
        void DLL_EXPORT exportDataToDirectory(string directory);
};

#endif