#include <vector>
#include <unordered_map>
#include "Geometry2D.hpp"
#include "math.h"
#include "Dubins3d/src/DubinsManeuver3d.hpp"

using namespace std;

#ifndef DUBINS_ENGINE_H
#define DUBINS_ENGINE_H

#define NUM_SAMPLES 100
#define RHO_MIN 5
#define PITCH_MIN_DEG -20.0 * M_PI / 180.0
#define PITCH_MAX_DEG 20.0 * M_PI / 180.0

typedef tuple<int, int> id_key_t;

struct key_hash : public unary_function<id_key_t, size_t>
{
    size_t operator()(const id_key_t& k) const
    {
        return get<0>(k) ^ get<1>(k);
    }
};

struct key_equal : public binary_function<id_key_t, id_key_t, bool>
{
   bool operator()(const id_key_t& v0, const id_key_t& v1) const
   {
      return (get<0>(v0) == get<0>(v1) && get<1>(v0) == get<1>(v1));
   }
};

struct DubinsData
{
    DubinsManeuver3d maneuver;
    vector<State> path;
};

typedef unordered_map<const id_key_t, DubinsData, key_hash, key_equal> maneuverMap;

class DubinsEngine
{
    static maneuverMap _maneuverMap;

    static DubinsData _generateData(const State& start, const State& final);
    static bool _maneuverNotInMap(const GraphNode& start, const GraphNode& final);
    static void _addManeuverToMap(const GraphNode& start, const GraphNode& final);

    public:
        static vector<State> generatePath(const State& start, const State& final);
        static vector<State> generatePathUsingMap(const GraphNode& start, const GraphNode& final);
        static double getPathLength(const State& start, const State& final);
};

#endif //DUBINS_ENGINE_H