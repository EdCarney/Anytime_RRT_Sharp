#include <iostream>
#include <time.h> 
#include <math.h>
#include <cmath>

#include "configspaceGraph.h"
#include "workspaceGraph.h"

using namespace std;

// computes the gate (root) node based on the approximate gate location
// and orientation; outputs a ConfigspaceNode to add to the graph
ConfigspaceNode calcGateNode(double xPosition, double yPosition, double approachAngle, double standOffRange)
{
	ConfigspaceNode gateNode;

	gateNode.x = xPosition + standOffRange * cos(approachAngle - M_PI);
	gateNode.y = yPosition + standOffRange * sin(approachAngle - M_PI);
	gateNode.theta = approachAngle;

	gateNode.v = 0.0; gateNode.w = 0.0; gateNode.t = 0.0;
	gateNode.a = 0.0; gateNode.gamma = 0.0; gateNode.cost = 0.0;
	gateNode.parentNodeId = 0;

	return gateNode;
}

int main()
{

	//------------------------------------------------------------------------//
	//-------this info should be ingested from the intialization script-------//
	//------------------------------------------------------------------------//
	int numGates = 1, numObstacles = 23;

	// define arrays for the gate and obstacle information
	double[numGates] approxGateXPosition, approxGateYPosition, approxGateApproach;
	double[numObstacles] obstacleXPosition, obstacleYPosition, obstacleRadius;

	approxGateXPosition = { 5.0 };
	approxGateYPosition = { 60.0 };
	approxGateApproach = { M_PI / 2 };

	obstacleXPosition = { 80, 73, 63, 53, 43, 33, 28, 25, 25, 25, 25, 35, 40, 45, 80, 85, 90, 95, 100, 100, 100, 100, 60 };
	obstacleYPosition = { 40, 30, 25, 25, 26, 25, 35, 47, 57, 67, 77, 80, 80, 80, 80, 80, 80, 80, 80, 0, 5, 10, 100 };
	obstacleRadius = { 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8 };

	// define additional input parameters for the goal node calculation
	double standOffRange = 5.0

	// initial UAV ore

	// initialize variables for the graph limits (which will
	// change with each gate iteration)
	double xMin = 0.0, yMin = 0.0, xMax = 0.0, yMax = 0.0;

	// define a buffer region and the side length to use when
	// defining the square freespace
	double buffer = 10.0, sideLength = 0.0;
	//------------------------------------------------------------------------//
	//------------------------------------------------------------------------//

	for (int gate = 0; gate < numGates; gate++)
	{
		// build workspace and configspace graphs
		WorkspaceGraph G_workspace;
		ConfigspaceGraph G_configspace;

		if (gate == 0)
		{
			// set the goal region (i.e. the UAV starting location)
			// if it's not the first iteration, then the goal region will be set based
			// on the last known position of the UAV
			G_workspace.addGoalRegion(<GATE_INFO>);
		}
		else
		{
			// add the goal region based on the exit node of the
			// last run
			G_workspace.addGoalRegion(<PREVIOUS_START_NODE>);
		}

		// Deriks function to determine goal node based on approximate gate information
		gateNode = calcGateNode(approxGateXPosition[gate], approxGateYPosition[gate], approxGateOrientation[gate]);

		// define the limits of the graph based on position of the gate
		// and the robot
		//sideLength = G_workspace.goalRegion.x 

		// set freespace of graphs based on the 
		G_workspace.defineFreespace();
		G_configspace.defineFreespace();

		// set the obstacles for this iteration (we don't need to consider all obstacles
		// for every iteration); use the above defined freespace limits
		for (int i = 0; i < numObstacles; i++)
		{
			if (G_workspace.obstacleInFreespace());
			{
				G_workspace.addObstacle();
			}
		}

		// compute the volume of the obstacles
		obsVol = G_workspace.computeObsVol();

		// add gateNode to the graph
		G_configspace.createNode(gateNode);

		///////////////////////////////////////////////////////////////////////////////////////////

		// start the anytime RRT# iterations
		while(!G_workspace.atGate())
		{
			iterationRuntime = 0.0;
			maxIterationRuntime = 1.0;
			while(iterationRuntime < maxIterationRuntime || !G_workspace.goalRegionReached())
			{
				// do the RRT# thing
			}

			// get the last n nodes in the tree
			lastNodes = G_configspace.getLastNodes();

			// print the last m nodes to a file format that can be
			// ingested by the simulation
			G_configspace.printNodes(lastNodes);

			// trim the tree to remove those nodes and all nodes with a cost
			// greater than the (n-m)th node (for a graph with n nodes)
			G_configspace.trimTree(lastNodes[0]);

			// update goal region in the workspace graph to the new
			// current last node in the tree (the (n-m)th node)
			G_workspace.updateGoalRegion(lastNodes[0]);
		}
	}
}