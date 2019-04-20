using namespace std;

#include <math.h>
#include <cmath>
#include <iostream>

struct ConfigspaceNode
{
	int id;				// unique identifer for the node; 1-indexed
	double x;			// x coordinate location of the vehicle centroid
	double y;			// y coordinate location of the vehicle centroid
	double theta;		// rotation of the vehicle
	double v;			// linear velocity of the vehicle
	double w;			// rotational velocity of the vehicle
	double t;			// time of the node
	double a;			// translational acceleration control input applied 
	double gamma;		// rotational acceleration control input applied
	int parentNodeId;	// the id of the parent node for this node
	double cost;		// cost-to-go for this node

	// iteration point parameters
	// used for collision checking
	ConfigspaceNode* iterationPoints;
	int numIterationPoints;
};


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
ConfigspaceNode node1;

double x = 0.0, y = 0.0, theta = 0.0, standoff = 0.0;

cout << "x: "; cin >> x;
cout << "y: "; cin >> y;
cout << "theta: "; cin >> theta;
cout << "standoff: "; cin >> standoff;

node1 = calcGateNode(x, y, theta, standoff);

printf("%f, %f, %f\n", node1.x, node1.y, node1.theta);

return 0;
}
