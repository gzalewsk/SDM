#pragma once

#include "SimulationData.h"

class Routing {
public:
	SimulationData simData;

	Routing(SimulationData &simData);
	~Routing(void);

	vector<int> ShortestPath(int s, int t, vector<double> costsTab, double limit);
};

class NodeState {
public:
	int node;
	double cost;
	int pred;
	
	NodeState(int _node, double _cost, int _pred = -1) { node = _node; cost = _cost; pred = _pred; }

	bool operator<(const NodeState &a) const {
		
		if(cost > a.cost) return true;
		else if(cost < a.cost) return false;
		else return node < a.node;
	}
};

