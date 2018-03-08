#include "StdAfx.h"
#include "NetworkState.h"


NetworkState::NetworkState() {

	nodes = 0;
	links = 0;
	modes = 0;
	demands = 0;
	slices = 0;
}

NetworkState::NetworkState(SimulationData &simData) {

	nodes = simData.net.nodes;
	links = simData.net.links;
	modes = simData.net.modes;
	demands = simData.traffic.demands;
	slices = simData.flexGrid.slices;
	InitState();
	ClearState();
	
}

void NetworkState::InitState() {

	xems = vector<vector<vector<int>>>(links);
	yems = vector<vector<vector<int>>>(links);
	zems = vector<vector<vector<int>>>(links);
	zes = vector<vector<int>>(links); 
	zes_min = vector<vector<int>>(links); 
	
	for (int e = 0; e < links; e++) {
		xems[e] = vector<vector<int>>(modes);
		yems[e] = vector<vector<int>>(modes);
		zems[e] = vector<vector<int>>(modes);
		zes[e] = vector<int>(slices);
		zes_min[e] = vector<int>(slices);
		for (int s = 0; s < slices; s++){
			zes[e][s] = slices - s;
			zes_min[e][s] = slices - s;
		}
		for (int m = 0; m < modes; m++) {
			xems[e][m] = vector<int>(slices);
			yems[e][m] = vector<int>(slices);
			zems[e][m] = vector<int>(slices);
			
			for (int s = 0; s < slices; s++){
				zems[e][m][s] = slices - s;
			}
		}
	}
	xe = vector<int>(links);

	allocatedLightpaths = vector<Lightpath>(demands);
	allocatedLightpathsNew = vector<LightpathNew>(demands);

}

void NetworkState::ClearState() {

	for(int e = 0; e < xems.size(); e++)
		for (int m = 0; m < xems[e].size(); m++)
			for (int s = 0; s < xems[e][m].size(); s++) {
				xems[e][m][s] = 0;
				yems[e][m][s] = -1;
				zems[e][m][s] = slices - s;
				zes[e][s] = slices - s;
				zes_min[e][s] = slices - s;
			}
	for(int e = 0; e < xe.size(); e++)
		xe[e] = 0;
}

int NetworkState::GetMaxSliceIndex() {

	int temp = 0;
	for(int e = 0; e < xe.size(); e++)
		if(xe[e] > temp)
			temp = xe[e];

	return temp;
}

void NetworkState::DisplayState() {

	for(int e = 0; e < xems.size(); e++) {
		cout << "Link " << e << " (" << xe[e] << "):" << endl;
		for (int m = 0; m < xems[e].size(); m++) {
			cout << "\tmode " << m << ":\t";
			for (int s = 0; s < xems[e][m].size(); s++)
				cout << xems[e][m][s] << " ";
			cout << endl;
		}
	}
}

bool NetworkState::Verify() {

	vector<int> xeValue = vector<int>(xe.size());

	cout << "  - Veryfing heuristic solution: ";

	for(int e = 0; e < xems.size(); e++) {
		xeValue[e] = 0;
		for (int m = 0; m < xems[e].size(); m++) {
			for (int s = 0; s < xems[e][m].size(); s++) {
				if (xems[e][m][s] < 0 || xems[e][m][s] > 1) {
					cout << "Warning: wrong value of xems (xems[" << e << "][" << m << "][" << s << "] == " << xems[e][m][s] << ")" << endl;
					exit(0);
				}
				if (xems[e][m][s] > 0 && xeValue[e] < s + 1)
					xeValue[e] = s + 1;
			}
		}
		if (xeValue[e] != xe[e]) {
			cout << "Warning: wrong value of xe (xe[" << e << "] = " << xe[e] << " != " << xeValue[e] << ")" << endl;
			exit(0);
		}
	}

	cout << " OK" << endl;

	return true;
}
