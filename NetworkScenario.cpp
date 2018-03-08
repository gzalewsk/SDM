#include "StdAfx.h"
#include "NetworkScenario.h"
#include <sstream>
#include <fstream>

NetworkScenario::NetworkScenario(SimulationParameters &simPar, SupportingFunctions &supFun) {
	
	this->simPar = simPar;
	this->supFun = supFun;

	nodes = 0;
	links = 0;
	modes = simPar.modes;

	flexGrid = FlexGrid(simPar.spectrum, simPar.sliceWidth);
	qotModel = QoTmodel(simPar, supFun, this);

	switch(simPar.network) {
		case SIMPLE: netName = "SIMPLE"; break;
		case INT9: netName = "INT9"; break;
		case POL12: netName = "POL12"; break;
		case DT12: netName = "DT12"; break;
		case DT14: netName = "DT14"; break;
		case NSF14_GangFeng: netName = "NSF14_GangFeng"; break; 
		case TEL14: netName = "TEL14"; break;
		case NSF15: netName = "NSF15"; break;
		case EURO16: netName = "EURO16"; break;
		case BT22: netName = "BT22"; break;
		case TEL21N35E: netName = "TEL21N35E"; break;
		case UBN24: netName = "UBN24"; break;
		case US26: netName = "US26"; break;
		case EURO28: netName = "EURO28"; break;
		case TEL30: netName = "TEL30"; break;
		case TI44: netName = "TI44"; break;
		case EURO49: netName = "EURO49"; break;
		default: {
			cout << "Warning: wrong network topology" << endl;
			system("pause");
			exit(0);
		}
	}

	netDir = simPar.dataDir + netName + "\\";
}

void NetworkScenario::InitializeNetwork() {

	int d, i, j, e;

	string line;
	vector<string> tokens;

	cout << "- Loading network " << simPar.network << endl;			// ###

	ostringstream id; id << netDir << "network.txt"; string fileName = id.str();
	fstream netFile(fileName.c_str(), std::ios::in);

	if(netFile) {
		getline(netFile, line, '\n'); supFun.Tokenize(line, tokens, " ");
		nodes = atoi(tokens[0].c_str());
		tokens.clear();
	
		getline(netFile, line, '\n'); supFun.Tokenize(line, tokens, " ");
		links = atoi(tokens[0].c_str());
		tokens.clear();

		if(!simPar.hasDirectedLinks)
			links = links / 2;

		cout << "  - Nodes: " << nodes << ", Links: " << links << endl;			// ###

		linkLength = vector<int>(links);

		neigh = vector<vector<int>>(nodes);
		dijkstraNeigh = vector<vector<int>>(nodes);
		linkIndex = vector<vector<int>>(nodes);
		for(i = 0; i < nodes; i++) {
			linkIndex[i] = vector<int>(nodes, -1);
			neigh[i] = vector<int>();
			dijkstraNeigh[i] = vector<int>();
		}
		linkNodes = vector<vector<int>>(links);
		for(e = 0; e < links; e++)
			linkNodes[e] = vector<int>(2);
		nodeLinks = vector<vector<int>>(nodes);
		nodeOutLinks = vector<vector<int>>(nodes);
		nodeInLinks = vector<vector<int>>(nodes);
		for(i = 0; i < nodes; i++) {
			nodeLinks[i] = vector<int>();
			nodeOutLinks[i] = vector<int>();
			nodeInLinks[i] = vector<int>();
		}

		e = 0;
		for(i = 0; i < nodes; i++) {
			getline(netFile, line, '\n'); supFun.Tokenize(line, tokens, "\t");
			for(j = 0; j < nodes; j++) {
				d = atoi(tokens[j].c_str());
				if(simPar.hasDirectedLinks) {
					if(d > 0) {
						linkLength[e] = d * simPar.linkLengthMultiplier;
						neigh[i].push_back(e);
						dijkstraNeigh[i].push_back(j);
						linkIndex[i][j] = e;
						linkNodes[e][0] = i;
						linkNodes[e][1] = j;
						nodeLinks[i].push_back(e);
						nodeLinks[j].push_back(e);
						nodeOutLinks[i].push_back(e);
						nodeInLinks[j].push_back(e);
						e++;
					}
				} else {
					if(d > 0 && i < j) {
						linkLength[e] = d * simPar.linkLengthMultiplier;
						neigh[i].push_back(e);
						neigh[j].push_back(e);
						dijkstraNeigh[i].push_back(j);
						dijkstraNeigh[j].push_back(i);
						linkIndex[i][j] = e;
						linkIndex[j][i] = e;
						linkNodes[e][0] = i;
						linkNodes[e][1] = j;
						nodeLinks[i].push_back(e);
						nodeLinks[j].push_back(e);
						e++;
					}
				}
			}
			tokens.clear();
		}

		//for(e = 0; e < linkLength.size(); e++) {
		//	cout << "link" << e << " [" << linkNodes[e][0] << "->" << linkNodes[e][1] << "]: " << linkLength[e] << endl;	
		//}
		//system("pause");

		//for(i = 0; i < nodes; i++) {
		//	for(j = 0; j < nodes; j++)
		//		cout << linkIndex[i][j] << " ";
		//	cout << endl;
		//}

		//for(i = 0; i < nodes; i++) {
		//	cout << "    Neighbour of node " << i << ":" << endl;
		//	cout << "      links: ";
		//	for(j = 0; j < neigh[i].size(); j++)
		//		cout << neigh[i][j] << " ";
		//	cout << "      nodes: ";
		//	for(j = 0; j < dijkstraNeigh[i].size(); j++)
		//		cout << dijkstraNeigh[i][j] << " ";				
		//	cout << endl;
		//}

	} else {
		cout << endl << "Warning: file " << fileName.c_str() << " does not exist. The network is not loaded" << endl << endl;	// ###
		system("pause");
		exit(0);
	}
}

void NetworkScenario::InitializePrecomputedRoutes() {

	routeManager = RouteManager(simPar, supFun, this);
	routeManager.LoadRoutes(netDir);
}

