#include "ManeuverEngine.hpp"

maneuverMap ManeuverEngine::_maneuverMap;

ManeuverType ManeuverEngine::maneuverType;

vector<State> ManeuverEngine::_generateDirectLinePath(const State& start, const State& final)
{
    Line line(start, final);
    vector<State> path(NUM_SAMPLES);
    Vector deltaVec;
    double delta = line.length() / (double)NUM_SAMPLES;
    
    for (int i = 0; i < NUM_SAMPLES; ++i)
    {
        deltaVec = line.tangent() * i * delta;
        path[i] = State(start.x() + deltaVec.x(), start.y() + deltaVec.y(), start.z() + deltaVec.z(), final.theta(), final.rho());
    }

    return path;
}

double ManeuverEngine::_getDirectLinePathLength(const State& start, const State& final)
{
    return start.distanceTo(final);
}

vector<State> ManeuverEngine::_generateDubinsPath(const State& start, const State& final)
{
    DubinsData dubinsData = _generateDubinsData(start, final);
    return dubinsData.path;
}

double ManeuverEngine::_getDubinsPathLength(const State& start, const State& final)
{
    DubinsData dubinsData = _generateDubinsData(start, final);
    return dubinsData.maneuver.length();
}

DubinsData ManeuverEngine::_generateDubinsData(const State& start, const State& final)
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

    return { maneuver, path };
}

bool ManeuverEngine::_maneuverNotInMap(const GraphNode& start, const GraphNode& final)
{
    auto mapItr = _maneuverMap.find(make_tuple(start.id(), final.id()));
    return mapItr == _maneuverMap.end();
}

void ManeuverEngine::_addManeuverToMap(const GraphNode& start, const GraphNode& final)
{
    DubinsData dubinsData = _generateDubinsData(start, final);
    _maneuverMap[make_tuple(start.id(), final.id())] = dubinsData;
}

vector<State> ManeuverEngine::generatePathUsingMap(const GraphNode& start, const GraphNode& final)
{
    if (_maneuverNotInMap(start, final))
        _addManeuverToMap(start, final);

    return _maneuverMap[make_tuple(start.id(), final.id())].path;
}

vector<State> ManeuverEngine::generatePath(const State& start, const State& final)
{
    if (maneuverType == Dubins3d)
        return _generateDubinsPath(start, final);
    else if (maneuverType == DirectPath)
        return _generateDirectLinePath(start, final);
}

double ManeuverEngine::getPathLength(const State& start, const State& final)
{
    if (maneuverType == Dubins3d)
        return _getDubinsPathLength(start, final);
    else if (maneuverType == DirectPath)
        return _getDirectLinePathLength(start, final);
}