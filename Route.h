#pragma once

#include "Definitions.h"
#include "SupportingFunctions.h"
#include "SimulationParameters.h"

//class SimulationParameters;
//class SupportingFunctions;
class NetworkScenario;

using namespace std;

class Route {
private:
	int length;
	int numOfRequiredSlices;
public:
	int src;
	int dest;
	int hops;
	int k;
	vector<int> linkVector;

	Route();
	Route(int src, int dest, vector<int> linkVector, NetworkScenario *netScen);
	Route(int src, int dest, vector<int> linkVector, int k, NetworkScenario *netScen);

	~Route() {};

	bool operator==(Route &rt);
	bool doesInclude(int link);
	bool doesCoincide(Route &rt);
	bool doesCoincideInDistinctLinks(Route &rt1, Route &rt2);	
    void display();

	void setLength(int len) { length = len; }
	void setNumOfSlices(int slices) { numOfRequiredSlices = slices; }
	int getLength() { return length; }
	int getNumOfSlices();
};

class RouteManager {
public:
	int numOfPrecomputedRoutes;
	int numOfCandidateRoutes;
	vector<vector<vector<Route>>> precomputedRoutes;
	
	SimulationParameters simPar;
	SupportingFunctions supFun;
	NetworkScenario *netScen;
	
	RouteManager() {};
	RouteManager(SimulationParameters &simPar, SupportingFunctions &supFun, NetworkScenario *netScen);
	~RouteManager() {};

	void LoadRoutes(string netDir);
	int ReadDirectedRoutesFromFile(string netDir);
	int ReadUndirectedRoutesFromFile(string netDir);
	void DisplayRoutes();
};
