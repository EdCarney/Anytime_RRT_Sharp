#include "Vehicle.hpp"

Vehicle::Vehicle()
{
	nodes = NULL;
	offsetNodes = NULL;
	numNodes = 0;
	state = { 0.0, 0.0, 0.0 };
	maxPointRadius = 0.0;
}

void Vehicle::updateState(State newState)
{
    state = { newState.x, newState.y, newState.theta };

	// update body nodes based on deltas
	for (int i = 0; i < numNodes; i++)
	{
		nodes[i].x = state.x + cos(state.theta) * offsetNodes[i].x - sin(state.theta) * offsetNodes[i].y;
		nodes[i].y = state.y + sin(state.theta) * offsetNodes[i].x + cos(state.theta) * offsetNodes[i].y;
	}
}