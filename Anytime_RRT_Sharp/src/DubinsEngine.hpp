#include <vector>
#include <unordered_map>
#include "Geometry2D.hpp"
#include "math.h"
#include "Dubins3d/src/DubinsManeuver3d.hpp"

using namespace std;

#ifndef DUBINS_ENGINE_H
#define DUBINS_ENGINE_H

#define NUM_SAMPLES 100
#define RHO_MIN 10
#define PITCH_MIN_DEG -15.0 * M_PI / 180.0
#define PITCH_MAX_DEG 15.0 * M_PI / 180.0

typedef tuple<int, int> key_t;

struct key_hash : public unary_function<key_t, size_t>
{
    size_t operator()(const key_t& k) const
    {
        return get<0>(k) ^ get<1>(k);
    }
};

struct key_equal : public binary_function<key_t, key_t, bool>
{
   bool operator()(const key_t& v0, const key_t& v1) const
   {
      return (get<0>(v0) == get<0>(v1) && get<1>(v0) == get<1>(v1));
   }
};

struct DubinsData
{
    DubinsManeuver3d maneuver;
    vector<State> path;
};

typedef std::unordered_map<const key_t, DubinsData, key_hash, key_equal> manuverMap;

class DubinsEngine
{
    static manuverMap _maneuverMap;

    static bool _maneuverNotInMap(const GraphNode& start, const GraphNode& end);
    static void _addManeuverToMap(const GraphNode& start, const GraphNode& end);

    public:
        static vector<State> generatePath(const GraphNode& start, const GraphNode& final);
};

#endif //DUBINS_ENGINE_H