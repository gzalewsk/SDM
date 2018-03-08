#pragma once
//#ifndef NETWORKSCENARIO_H_
//#define NETWORKSCENARIO_H_

using namespace std;

#include "SimulationParameters.h"
#include "SupportingFunctions.h"
#include "Route.h"
#include "FrequencyGrids.h"
#include "QoTmodel.h"

class NetworkScenario {
public:
	string netName;
	int nodes;
	int links;
	int modes;
	vector<int> linkLength;
	vector<vector<int>> neigh;
	vector<vector<int>> dijkstraNeigh;
	vector<vector<int>> linkIndex;
	vector<vector<int>> linkNodes;
	vector<vector<int>> nodeLinks;
	vector<vector<int>> nodeOutLinks;
	vector<vector<int>> nodeInLinks;

	RouteManager routeManager;
	FlexGrid flexGrid;
	QoTmodel qotModel;

	string netDir;

	SimulationParameters simPar;
	SupportingFunctions supFun;

	NetworkScenario() {};
	NetworkScenario(SimulationParameters &simPar, SupportingFunctions &supFun);
	~NetworkScenario(void) {};

	void InitializeNetwork();
	void InitializePrecomputedRoutes();
};

//#endif