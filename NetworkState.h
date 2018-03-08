#pragma once

#include "SimulationData.h"
#include "Lightpath.h"

class NetworkState {
public:
	int slices;
	int modes;
	int links;
	int nodes;
	int demands;
	vector<vector<vector<int>>> xems;	// allocation status (free slice = 0, allocated slice = 1)
	vector<vector<vector<int>>> yems;	// allocation status (the index of demand allocating given slice)
	vector<vector<vector<int>>> zems;	// allocation status (the maximal size of channel starting at this slot)
	vector<vector<int>> zes;	
	vector<vector<int>> zes_min;
	list<int> orders; //list of orders to serve
	vector<int> xe;
	vector<Lightpath> allocatedLightpaths;
	vector<LightpathNew> allocatedLightpathsNew;

	NetworkState();
	NetworkState(SimulationData &simData);
	~NetworkState(void) {};

	void InitState();
	void ClearState();
	int GetMaxSliceIndex();
	void DisplayState();
	bool Verify();
};