#include "StdAfx.h"
#include "Traffic.h"
#include <sstream>
#include <fstream>

#include "SimulationParameters.h"
#include "SupportingFunctions.h"
#include "NetworkScenario.h"

Traffic::Traffic(SimulationParameters &simPar, SupportingFunctions &supFun, NetworkScenario &netScen) {

	this->simPar = simPar;
	this->supFun = supFun;
	this->netScen = netScen;

	demandSet = vector<Demand>();
	demands = 0;

	maxSliceVolume = -1;
}

void Traffic::InitializeTraffic(string setID, string netDir) {

	LoadBitRateDemands(setID, netDir);

	cout << "    - " << demands << " demands loaded" << endl;
}

void Traffic::LoadBitRateDemands(string setID, string netDir) {

	int d, e, k, n, p, q, t, v, src, dest, hd;
	int demandsInFile, numOfSlices, len;
	Route rt;

	string line;
	vector<string> tokens, tokens1, tokens2, tokens3, tokens4;

	cout << "  - Loading generic bitrate demands" << endl;

	demands = 0;
	demandSet = vector<Demand>();
	netScen.routeManager.numOfCandidateRoutes = 0;
	int maxSliceDemand = 0;

	ostringstream id; id << netDir << "traffic\\bitrate\\" << setID << ".txt"; string fileName = id.str();
	fstream netFile(fileName.c_str(), ios::in);

	if (!netFile) {
		cout << endl << "Warning: file " << fileName.c_str() << " does not exist. The demand set not loaded" << endl << endl;
		exit(0);

	}
	else {
		getline(netFile, line, '\n'); supFun.Tokenize(line, tokens, " ");
		demandsInFile = atoi(tokens[0].c_str());
		tokens.clear();

		for (d = 0; d < demandsInFile; d++) {

			getline(netFile, line, '\n'); supFun.Tokenize(line, tokens, "\t");
			src = atoi(tokens[0].c_str());
			dest = atoi(tokens[1].c_str());
			hd = atoi(tokens[2].c_str());

			Demand demand = Demand(src, dest, hd);

			if (netScen.routeManager.precomputedRoutes[src][dest].size() == 0) {
				cout << "Warning: no candidate routes for demand " << d << " (nodes: " << src << "-" << dest << ")" << endl;
				exit(0);
			}
			else {
				int numOfRts = netScen.routeManager.precomputedRoutes[src][dest].size();
				demand.candidateRoutes = vector<Route>(numOfRts);
				for (k = 0; k < numOfRts; k++) {
					rt = netScen.routeManager.precomputedRoutes[src][dest][k];
					n = netScen.qotModel.ConvertBitrateToSlices(hd, rt.getLength());
					rt.setNumOfSlices(n);
					demand.candidateRoutes[k] = rt;

					if (maxSliceDemand < n)
						maxSliceDemand = n;
				}
				demandSet.push_back(demand);
				demands++;
				netScen.routeManager.numOfCandidateRoutes += netScen.routeManager.precomputedRoutes[src][dest].size();
			}

			tokens.clear();
		}
	}

	cout << "  - Maximal slice demand = " << maxSliceDemand << endl;
	maxSliceVolume = maxSliceDemand;
	if (maxSliceDemand > netScen.flexGrid.slices)
		netScen.flexGrid.UpdateNumberOfSlices(maxSliceVolume);
}

void Traffic::DisplayDemands() {

	cout << "  - Displaying demands" << endl;

	for (int d = 0; d < demands; d++) {
		cout << "    d" << d << ": ";
		if (d < demandSet.size())
			demandSet[d].DisplayDemand();
	}
}