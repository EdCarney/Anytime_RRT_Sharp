#include <chrono>
#include <string>
#include <vector>
#include "ArrtsEngine.hpp"
#include "ArrtsParams.hpp"
#include "ConfigspaceGraph.hpp"
#include "ConfigspaceNode.hpp"
#include "cppshrhelp.hpp"
#include "Geometry2D.hpp"
#include "Geometry3D.hpp"
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
        void _setFinalNode();
        void _setFinalPathFromFinalNode();
        void _configureWorkspace(ArrtsParams params);
        void _configureConfigspace(ArrtsParams params);
        void _runAlgorithm(ArrtsParams params);
        void _exportDataToDirectory(string directory);

    public:
        vector<State> DLL_EXPORT calculatePath(ArrtsParams params, string dataExportDir = "");
};

#endif