#include "DubinsEngine.hpp"

bool DubinsEngine::_maneuverNotInMap(const GraphNode& start, const GraphNode& final)
{
    auto mapItr = _maneuverMap.find(make_tuple(start.id(), final.id()));
    return mapItr == _maneuverMap.end();
}

void DubinsEngine::_addManeuverToMap(const GraphNode& start, const GraphNode& final)
{
    State3d qi { start.x(), start.y(), start.z(), start.theta(), start.rho() };
    State3d qf { final.x(), final.y(), final.z(), final.theta(), final.rho() };

    DubinsManeuver3d maneuver(qi, qf, RHO_MIN, { PITCH_MIN_DEG, PITCH_MAX_DEG });
    vector<State> path;

    if (maneuver.length() > 0)
    {
        path.resize(NUM_SAMPLES);
        auto dubinsStates = maneuver.computeSampling(NUM_SAMPLES);
        for (int i = 0; i < NUM_SAMPLES; ++i)
            path[i] = State(dubinsStates.at(i).x, dubinsStates.at(i).y, dubinsStates.at(i).z, dubinsStates.at(i).theta, dubinsStates.at(i).gamma);
    }

    _maneuverMap[make_tuple(start.id(), final.id())] = { maneuver, path };
}

vector<State> DubinsEngine::generatePath(const GraphNode& start, const GraphNode& final)
{
    if (_maneuverNotInMap(start, final))
        _addManeuverToMap(start, final);

    return _maneuverMap[make_tuple(start.id(), final.id())].path;
}