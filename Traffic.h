#pragma once

#include "Definitions.h"
#include "NetworkScenario.h"
#include "Route.h"

class SimulationParameters;
class SupportingFunctions;

using namespace std;

class Demand {
public:
	int src;
	int dest;
	double hd;

	vector<Route> candidateRoutes;

	Demand() {
		src = -1; dest = -1; hd = -1;
		candidateRoutes = vector<Route>();
	};

	Demand(int src, int dest, int hd) {
		this->src = src; this->dest = dest;	this->hd = hd;
		candidateRoutes = vector<Route>();
	};

	~Demand(void) {};

	void DisplayDemand() {
		cout << src << " -> " << dest;
		cout << " = " << hd << "Gbps" << endl;
	};
};

class Traffic {
public:
	int demands;
	vector<Demand> demandSet;

	int maxSliceVolume;

	SimulationParameters simPar;
	SupportingFunctions supFun;
	NetworkScenario netScen;
	
	Traffic() {};
	Traffic(SimulationParameters &simPar, SupportingFunctions &supFun, NetworkScenario &netScen);
	~Traffic(void) {};

	void InitializeTraffic(string setID, string netDir);
	void LoadBitRateDemands(string setID, string netDir);
	void DisplayDemands();
};
