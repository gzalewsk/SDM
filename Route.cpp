#include "StdAfx.h"
#include "Route.h"
#include <sstream>
#include <fstream>

#include "SimulationParameters.h"
#include "SupportingFunctions.h"
#include "NetworkScenario.h"

//-----------------------------------------------------------------------------------------
Route::Route() {

	this->src = -1; this->dest = -1;
	hops = -1;
	numOfRequiredSlices = -1;
	k = -1;
	length = 0;
}

Route::Route(int src, int dest, vector<int> linkVector, NetworkScenario *netScen) {

	this->src = src; this->dest = dest;	
	this->linkVector = linkVector;
	hops = linkVector.size();
	numOfRequiredSlices = -1;
	k = -1;

	length = 0;
	for(int h = 0; h < hops; h++)
		length += netScen->linkLength[linkVector[h]];
}

Route::Route(int src, int dest, vector<int> linkVector, int k, NetworkScenario *netScen)  {

	this->src = src; this->dest = dest;	
	this->linkVector = linkVector;
	hops = linkVector.size();
	numOfRequiredSlices = -1;
	this->k = k;

	length = 0;
	for(int h = 0; h < hops; h++)
		length += netScen->linkLength[linkVector[h]];
}

bool Route::operator==(Route &rt) {

	if(hops == rt.hops && src == rt.src && dest == rt.dest) {
		bool isEqual = true;
		for(int h = 0; h < hops; h++) {
			if(linkVector[h] != rt.linkVector[h]) {
				isEqual = false;				
				break;
			}
		}
		return isEqual;
	} else
		return false;
}

bool Route::doesInclude(int link) {

	for(int h = 0; h < hops; h++)
		if(linkVector[h] == link)
			return true;

	return false;
}

bool Route::doesCoincide(Route &rt) {

	for(int h = 0; h < hops; h++) {
		for(int g = 0; g < rt.hops; g++) {
			if(linkVector[h] == rt.linkVector[g])
				return true;
		}
	}

	return false;
}

bool Route::doesCoincideInDistinctLinks(Route &rt1, Route &rt2) {

	return false;
}

int Route::getNumOfSlices() {

	if(numOfRequiredSlices == -1) {
		cout << "Warning: the required number of slices not defined" << endl;
		exit(0);
	} else
		return numOfRequiredSlices;
}

void Route::display() {

	cout << "[" << src << "->" << dest << "], k = " << k << ", length = " << length << ": ";

	cout << "{";
	for(int h = 0; h < hops; h++) {
		cout << linkVector[h];
		if(h < hops - 1)
			cout << ", ";
	}
	cout << "}";
	
	if(numOfRequiredSlices > 0)
		cout << ", slices = " << numOfRequiredSlices;

	cout << endl;
}

//-----------------------------------------------------------------------------------------
RouteManager::RouteManager(SimulationParameters &simPar, SupportingFunctions &supFun, NetworkScenario *netScen) {

	this->simPar = simPar;
	this->supFun = supFun;
	this->netScen = netScen;

	numOfPrecomputedRoutes = 0;
	numOfCandidateRoutes = 0;
}

void RouteManager::LoadRoutes(string netDir) {

	cout << "- Loading candidate routes" << endl;

	if(simPar.hasDirectedLinks)
		ReadDirectedRoutesFromFile(netDir);
	else
		ReadUndirectedRoutesFromFile(netDir);
	
	cout << "  - " << numOfPrecomputedRoutes << " routes loaded" << endl;
}

int RouteManager::ReadDirectedRoutesFromFile(string netDir) {

	int e, i, j, k, p;
    string line;
    vector<string> tokens;

	Route rt;

	numOfPrecomputedRoutes = 0;

	ostringstream id;
	if(simPar.areRoutesUnique) {
		if(simPar.isLinkOrderMaintained)
			id << netDir << "paths\\linkSeq_unique.txt";
		else
			id << netDir << "paths\\paths_unique.txt";
	} else {
		if(simPar.isLinkOrderMaintained)
			id << netDir << "paths\\linkSeq.txt";
		else
			id << netDir << "paths\\paths.txt";
	}
	string fileName = id.str();
	fstream netFile(fileName.c_str(), ios::in);

	//int hops = 0;
	if(netFile) {
		getline(netFile, line, '\n'); supFun.Tokenize(line, tokens, " ");
		int kRoutesInFile = atoi(tokens[0].c_str());
		tokens.clear();
	
		if(kRoutesInFile < simPar.kSPlimit) {		
			cout << "  Warning: kRoutes too large (kSP <- " << kRoutesInFile << ")" << endl;
			simPar.kSPlimit = kRoutesInFile;
			system("pause");
			exit(0);
		} else {
			precomputedRoutes = vector<vector<vector<Route>>>(netScen->nodes);
			for(i = 0; i < netScen->nodes; i++)
				precomputedRoutes[i] = vector<vector<Route>>(netScen->nodes);

			int iter = 0;
			long len = 0;
			for(i = 0; i < netScen->nodes; i++) {
				for(j = 0; j < netScen->nodes; j++) {
					if(i != j) {					
						k = 0;
						for(p = 0; p < kRoutesInFile; p++) {
							if(k < simPar.kSPlimit) {
								getline(netFile, line, '\n'); supFun.Tokenize(line, tokens, " ");

								bool isLineEmpty = false;
								if(simPar.isLinkOrderMaintained) {
									if(tokens.size() == 0)
										isLineEmpty = true;
								} else {
									if(atoi(tokens[0].c_str()) == -1)
										isLineEmpty = true;
								}

								if(!isLineEmpty) {

									vector<int> linkVector;
									if(simPar.isLinkOrderMaintained) {
										for(int h = 0; h < tokens.size(); h++) {
											e = atoi(tokens[h].c_str());
											linkVector.push_back(e);
										}
									} else {
										for(e = 0; e < netScen->links; e++)
											if(atoi(tokens[e].c_str()) > 0)
												linkVector.push_back(e);
									}

									rt = Route(i, j, linkVector, k, netScen);

									double XTdB = -BIGNUMBER;
									if (netScen->modes > 1 && simPar.isXTconsidered && simPar.XTtype == BESTEFFORT) {
										int Kmax;		// max number of neighbour modes
										if (netScen->modes == 2) { Kmax = 1; }
										else if (netScen->modes == 3) { Kmax = 2; }
										else if (netScen->modes == 7) { Kmax = 6; }
										else if (netScen->modes == 19) { Kmax = 6; }
										else { cout << "Warning: " << netScen->modes << " modes not supported!"; exit(0); }
										
										double XTlinear = rt.getLength() * Kmax * simPar.hXTkm;
										XTdB = 10 * log10(XTlinear);
									}
									
									if (XTdB > simPar.XTlimit) {
										if (k == 0)
											cout << XTdB << endl;
									}
									else {

										precomputedRoutes[i][j].push_back(rt);
										numOfPrecomputedRoutes++;

										//hops += rt.hops;

										if (k == 0) {
											iter++;
											len += rt.getLength();
											//cout << iter << "\t" << rt.getLength() << endl;
										}

									}
								}

								k++;
								tokens.clear();

							} else
								getline(netFile, line, '\n');
						}

						//cout << precomputedRoutes[i][j].size() << endl;
					}
				}
			}

			cout << "  - mean SP path length = " << len / iter << " km" << endl;
		}
	} else {
		cout << endl << "Warning: file " << fileName.c_str() << " does not exist. The Route set not loaded" << endl << endl;	// ###
		exit(0);
	}

	//cout << hops << endl;
	//system("pause");

	return numOfPrecomputedRoutes;
}

int RouteManager::ReadUndirectedRoutesFromFile(string netDir) {

	int e, i, pathID, j, k, p;
    string line;
    vector<string> tokens;

	Route rt;

	numOfPrecomputedRoutes = 0;

	ostringstream id;
	if(simPar.areRoutesUnique) {
		if(simPar.isLinkOrderMaintained) {
			cout << "Warning: link order not supported" << endl; exit(0);
		} else
			id << netDir << "paths\\paths_undir_unique.txt";
	} else {
		if(simPar.isLinkOrderMaintained) {
			cout << "Warning: link order not supported" << endl; exit(0);
		} else
			id << netDir << "paths\\paths_undir.txt";
	}
	string fileName = id.str();
	fstream netFile(fileName.c_str(), ios::in);

	if(netFile) {
		getline(netFile, line, '\n'); supFun.Tokenize(line, tokens, " ");
		int kRoutesInFile = atoi(tokens[0].c_str());
		tokens.clear();
	
		if(kRoutesInFile < simPar.kSPlimit) {		
			cout << "  Warning: kRoutes too large (kSP <- " << kRoutesInFile << ")" << endl;
			simPar.kSPlimit = kRoutesInFile;
			system("pause");
			exit(0);
		} else {
			precomputedRoutes = vector<vector<vector<Route>>>(netScen->nodes);
			for(i = 0; i < netScen->nodes; i++)
				precomputedRoutes[i] = vector<vector<Route>>(netScen->nodes);

			int maxRouteLength = 0;
			for(i = 0; i < netScen->nodes; i++) {
				for(j = 0; j < netScen->nodes; j++) {
					if(i < j) {
						k = 0;
						pathID = 0;
						for(p = 0; p < kRoutesInFile; p++) {
							if(k < simPar.kSPlimit) {
								getline(netFile, line, '\n'); supFun.Tokenize(line, tokens, " ");

								if(atoi(tokens[0].c_str()) != -1) {
									vector<int> linkVector;
									for(e = 0; e < netScen->links; e++)
										if(atoi(tokens[e].c_str()) > 0)
											linkVector.push_back(e);
									rt = Route(i, j, linkVector, pathID, netScen);

									//if(rt.getLength() > netScen->qotModel.getMaxTransmissionDistance())
									//	cout << i << "\t" << j << "\t" << rt.getLength() << endl;

									if(rt.getLength() <= netScen->qotModel.getMaxTransmissionDistance()) {
										if(maxRouteLength < rt.getLength())
											maxRouteLength = rt.getLength();
										//cout << rt.getLength() << "\t";

										precomputedRoutes[i][j].push_back(rt);
										precomputedRoutes[j][i].push_back(rt);
										numOfPrecomputedRoutes++;
										pathID++;
									}
								}
								
								k++;
								tokens.clear();

							} else
								getline(netFile, line, '\n');
						}
						//if(precomputedRoutes[i][j].size() == 0) {
						//	cout << "Warning: no routes loaded for node pair " << i << "-" << j << endl;
						//	//system("pause");
						//	//exit(0);
						//}
					}
				}
			}

			cout << "  - Max. route length = " << maxRouteLength << "km" << endl;
		}
	} else {
		cout << endl << "Warning: file " << fileName.c_str() << " does not exist. The Route set not loaded" << endl << endl;	// ###
		exit(0);
	}

	//system("pause");
	//exit(0);

	return numOfPrecomputedRoutes;
}

void RouteManager::DisplayRoutes() {

	int i, j, k;

	cout << "- Displaying precomputed routes" << endl;

	for(i = 0; i < precomputedRoutes.size(); i++)
		for(j = 0; j < precomputedRoutes[i].size(); j++)
			for(k = 0; k < precomputedRoutes[i][j].size(); k++) {
				cout << "    Route " << k << ": ";
				precomputedRoutes[i][j][k].display();
			}
}

