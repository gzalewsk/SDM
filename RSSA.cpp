#include "StdAfx.h"
#include "RSSA.h"

RSSA::RSSA() : OptimizationProblem() {

}

double RSSA::GetObjValue(int slices, int unservedDemands, SimulationData &simData) {

	//cout << slices << " " << unservedDemands << " " << simData.flexGrid.slices << " ";
	double objective = slices + unservedDemands * simData.flexGrid.slices * 1000;
	//cout << objective << endl;

	return objective;
}

double RSSA::EvaluateSolution(vector<int> &order, NetworkState &netState, SimulationData &simData) {

	//return EvaluateSolutionMK(order, netState, simData);
	//return EvaluateSolutionGZ(order, netState, simData);
	return EvaluateSolutionGZNew(order, netState, simData);
	//	to improve in RSSA:
	//	- use only 2 instead of 3 matrices
}

double RSSA::EvaluateSolutionMK(vector<int> &order, NetworkState &netState, SimulationData &simData) {

	int c, d, e, h, i, j, k, m, n, q, s, t;
	int numOfRts;
	Route rt;
	Channel ch;

	int unservedDemands = 0;

	for (j = 0; j < order.size(); j++) {
		d = order[j];
		numOfRts = simData.traffic.demandSet[d].candidateRoutes.size();
		bool isLpFound = false;

		Route bestRt;
		Channel bestCh;
		int bestMode = -1;

		//cout << "Demand " << d << " is processed" << endl;

		for (k = 0; k < numOfRts; k++) {
			rt = simData.traffic.demandSet[d].candidateRoutes[k];
			n = rt.getNumOfSlices();

			if (n <= simData.flexGrid.slices) {
				for (m = 0; m < simData.net.modes; m++) {
					for (c = 0; c < simData.flexGrid.chs[n].size(); c++) {
						ch = simData.flexGrid.chs[n][c];
						if (ch.size != 0) {
							bool status = isChannelEmpty(rt, ch, m, netState);
							if (status) {
								if (simData.net.modes > 1 && simData.simPar.isXTconsidered && simData.simPar.XTtype == PRECISE)
									status = isXTacceptable(rt, ch, m, netState, simData);
								if (status) {
									updateBestLp(bestRt, bestCh, bestMode, rt, ch, m);
									isLpFound = true;
									break;
								}
							}
						}
					}
				}
			}
		}

		//cout << "- Selected allocation: route = " << bestRt.k << ", mode = " << bestMode << ", channel = [" << bestCh.beginSlice << ", " << bestCh.endSlice << "]" << endl << endl;

		if (!isLpFound) {
			//			cout << "Demand " << d << ": lightpath not found" << endl;
			unservedDemands++;
		}
		else {
			addLpToSolution(d, bestRt, bestCh, bestMode, netState); // Lp = lightpath (œcie¿ka optyczna)
		}
	}
	double objectiveValue = GetObjValue(netState.GetMaxSliceIndex(), unservedDemands, simData);
	return objectiveValue;
}

double RSSA::EvaluateSolutionGZNewV2(vector<int> &order, NetworkState &netState, SimulationData &simData) {
	int c, d, e, h, i, j, k, m, n, q, s, t;
	int numOfRts;
	Route rt;
	Channel ch;
	int unservedDemands = 0;
	for (j = 0; j < order.size(); j++) {
		d = order[j];
		numOfRts = simData.traffic.demandSet[d].candidateRoutes.size();
		bool isLpFound = false;
		Route bestRt;
		int bestMode = -1;
		int bestBeginSlice = -1;
		for (k = 0; k < numOfRts; k++) {
			rt = simData.traffic.demandSet[d].candidateRoutes[k];
			n = rt.getNumOfSlices();		//number of required slices to assign the route
			
			if (n <= simData.flexGrid.slices) {
				for (m = 0; m < simData.net.modes; m++) {
					int beginSlice = 0;
					while (beginSlice < simData.flexGrid.slices) {
						int status = isBeginSliceEmpty(rt, beginSlice, m, netState, n);
						if (status == BIGINT) {
							updateBestLp(bestRt, bestBeginSlice, bestMode, rt, beginSlice, m, n);
							isLpFound = true;
							break;
						}
						else if (status < 0)
							beginSlice += -status;
						else
							beginSlice += status;
					}
				}
			}
		}
		if (!isLpFound) {
			unservedDemands++;
		}
		else {
			addLpToSolution(d, bestRt, bestBeginSlice, bestMode, netState, simData);
		}
	}
	double objectiveValue = GetObjValue(netState.GetMaxSliceIndex(), unservedDemands, simData);
	return objectiveValue;
}

double RSSA::EvaluateSolutionGZ(vector<int> &order, NetworkState &netState, SimulationData &simData) {
	int c, d, e, h, i, j, k, m, n, q, s, t;
	int numOfRts;
	Route rt;
	Channel ch;
	int unservedDemands = 0;
	for (j = 0; j < order.size(); j++) {
		d = order[j];
		numOfRts = simData.traffic.demandSet[d].candidateRoutes.size();
		bool isLpFound = false;
		Route bestRt;
		int bestMode = -1;
		int bestBeginSlice = -1;
		//cout << "Demand " << d << " is processed" << endl;
		for (k = 0; k < numOfRts; k++) {
			rt = simData.traffic.demandSet[d].candidateRoutes[k];
			n = rt.getNumOfSlices();		//number of required slices to assign the route
			if (n <= simData.flexGrid.slices) {
				for (m = 0; m < simData.net.modes; m++) {
					int beginSlice = 0;
					while (beginSlice < simData.flexGrid.slices) {
						int status = isBeginSliceEmpty(rt, beginSlice, m, netState, n);
						if (status == BIGINT) {
							updateBestLp(bestRt, bestBeginSlice, bestMode, rt, beginSlice, m, n);
							isLpFound = true;
							break;
						}
						else if (status < 0)
							beginSlice += -status;
						else
							beginSlice += status;
					}
				}
			}
		}
		if (!isLpFound) {
			unservedDemands++;
		}
		else {
			addLpToSolution(d, bestRt, bestBeginSlice, bestMode, netState, simData);
		}
	}
	double objectiveValue = GetObjValue(netState.GetMaxSliceIndex(), unservedDemands, simData);
	return objectiveValue;
}


double RSSA::EvaluateSolutionGZNew(vector<int> &order, NetworkState &netState, SimulationData &simData) {
	int c, d, e, h, i, j, k, m, n, q, s, t;
	int numOfRts;
	Route rt;
	Channel ch;
	int unservedDemands = 0;
	vector<int> modeVector(simData.net.links);
	vector<int> bestModeVector(simData.net.links);			
	for (j = 0; j < order.size(); j++) {
		d = order[j];
		numOfRts = simData.traffic.demandSet[d].candidateRoutes.size();
		bool isLpFound = false;
		Route bestRt;
		for (e = 0; e < simData.net.links; e++) {
			bestModeVector[e] = -1;
		};
		int bestBeginSlice = -1;
		for (k = 0; k < numOfRts; k++) {
			rt = simData.traffic.demandSet[d].candidateRoutes[k];
			n = rt.getNumOfSlices();		//number of required slices to assign the route
			if (n <= simData.flexGrid.slices) {
				int beginSlice = 0;
				while (beginSlice < simData.flexGrid.slices) {
					int status = BIGINT;
					
					for (int h = 0; h < rt.hops; h++) {// tutaj iterowac po modach i sprawdzic czy istnieje jakis mod na kazdym laczy na ktorym mamy wolne zasoby, kanal
						e = rt.linkVector[h]; // h-ta krawêdŸ œcie¿ki rt
						modeVector[e] = -1;
						if (netState.zes[e][beginSlice] < n) {
							status = netState.zes_min[e][beginSlice];
							//status = abs(netState.zes[e][beginSlice]);
							break;
						}
						for (m=0; m < simData.net.modes; m++){
							if (netState.zems[e][m][beginSlice] < n) {
								//wartosc bezwzgledna !!!
								if (abs(netState.zems[e][m][beginSlice]) < status){status = abs(netState.zems[e][m][beginSlice]);} // przypisanie najmniejszej wartosci beginSlice sposrod wszystkich modow
							}
							if (netState.zems[e][m][beginSlice] >= n){
								modeVector[e] = m;
								status = BIGINT;
								break;
							}
						}
						if (modeVector[e] == -1) {
							//cout << "na krawedzi sciezki nie znaleziono zadnego wolnego modu alokowanego na beginSlice -> (sciezka; przeskok; krawedz; demand; beginSlice) " << k << " " << h << " " << e << " " << d << " " << beginSlice << endl;
							//netState.zes[e][beginSlice] = 1;
							break; // wyjsc z petli iterujacej po hops'ach
						}; 
					};
					if (status == BIGINT){
						updateBestLp(bestRt, bestBeginSlice, bestModeVector, rt, beginSlice, modeVector, n);
						isLpFound = true;
						break;
					}
					else 
						beginSlice += status;
					
				}
			}
		}
		//cout << "- Selected allocation: route = " << bestRt.k << ", mode = " << bestMode << ", channel = [" << bestCh.beginSlice << ", " << bestCh.endSlice << "]" << endl << endl;
		if (!isLpFound) {
			//			cout << "Demand " << d << ": lightpath not found" << endl;
			unservedDemands++;
		}
		else addLpToSolution(d, bestRt, bestBeginSlice, bestModeVector, netState, simData);
	}
	double objectiveValue = GetObjValue(netState.GetMaxSliceIndex(), unservedDemands, simData);
	return objectiveValue;
}




bool RSSA::isChannelEmpty(Route &rt, Channel &ch, int mode, NetworkState &netState) {

	for(int h = 0; h < rt.hops; h++) {
		int e = rt.linkVector[h];
		for(int s = ch.beginSlice; s <= ch.endSlice; s++)
			if(netState.xems[e][mode][s] > 0)
				return false;
	}

	return true;
}

bool RSSA::isXTacceptable(Route &rt, Channel &ch, int mode, NetworkState &netState, SimulationData &simData) {
	int e, h, i, m, s;
	double XTlinear = 0;
	vector<bool> isDemandAffected = vector<bool>(simData.traffic.demands);
	for (int d = 0; d < isDemandAffected.size(); d++)
		isDemandAffected[d] = false;
	// --- evaluate XT for the candidate lightpath (i.e., assuming given route, channel and mode)
	for (h = 0; h < rt.hops; h++) {
		e = rt.linkVector[h];
		int Kmax = 0;
		for (s = ch.beginSlice; s <= ch.endSlice; s++) {
			int K = 0;
			for (i = 0; i < simData.net.neighbourModes[mode].size(); i++) {
				m = simData.net.neighbourModes[mode][i];
				if (netState.xems[e][m][s] > 0) {
					K++;
					int d = netState.yems[e][m][s];
					isDemandAffected[d] = true;			// "true" means that the XT of demand d may be affected by allocation [rt, ch, mode];
														// hence, the XT on the lightpath which was established for demand d has to be verified
				}
			}
			if (Kmax < K)
				Kmax = K;
		}
		XTlinear += simData.net.linkLength[e] * Kmax * simData.simPar.hXTkm;
	}
	double XTdB = 10 * log10(XTlinear);
	//cout << "route = " << rt.k << ", mode = " << mode << ", channel = slices [" << ch.beginSlice << ", " << ch.endSlice << "]: XTdB = " << XTdB << endl;
	if (XTdB > simData.simPar.XTlimit)
		return false;
	//return true;
	// --- evaluate XT for all affected demands/lightpaths (i.e., assuming their route, channel and mode)
	//cout << "\tAffected demands: ";
	for (int d = 0; d < isDemandAffected.size(); d++) {
		if (isDemandAffected[d]) {
			//cout << d << " ";
			double XTlinear_d = 0;			
			Lightpath lp = netState.allocatedLightpaths[d];
			for (h = 0; h < lp.rt.hops; h++) {
				e = lp.rt.linkVector[h];
				int Kmax = 0;
				for (s = lp.ch.beginSlice; s <= lp.ch.endSlice; s++) {
					int K = 0;
					for (i = 0; i < simData.net.neighbourModes[lp.mode].size(); i++) {
						m = simData.net.neighbourModes[lp.mode][i];
						if (netState.xems[e][m][s] > 0)
							K++;
						else if (m == mode && s >= ch.beginSlice && s <= ch.endSlice && rt.doesInclude(e))
							K++;
					}
					if (Kmax < K)
						Kmax = K;
				}
				XTlinear_d += simData.net.linkLength[e] * Kmax * simData.simPar.hXTkm;
			}
			double XTdB_d = 10 * log10(XTlinear_d);
			//cout << "\taffected demand = " << lp.d << " of route = " << rt.k << ", mode = " << lp.mode << ", channel = slices [" << lp.ch.beginSlice << ", " << lp.ch.endSlice << "]: XTdB = " << XTdB_d << endl;
			if (XTdB_d > simData.simPar.XTlimit)
				return false;
		}
	}
	return true;
}



void RSSA::addLpToSolution(int d, Route &bestRt, Channel &bestCh, int bestMode, NetworkState &netState) {
	for (int h = 0; h < bestRt.hops; h++) {
		int e = bestRt.linkVector[h];
		for (int s = bestCh.beginSlice; s <= bestCh.endSlice; s++) {
			netState.xems[e][bestMode][s]++;
			netState.yems[e][bestMode][s] = d;
		}
		if (netState.xe[e] < bestCh.endSlice + 1)
			netState.xe[e] = bestCh.endSlice + 1;
	}
	netState.allocatedLightpaths[d] = Lightpath(d, bestRt, bestMode, bestCh);
}


void RSSA::addLpToSolution(int d, Route &bestRt, int &bestBeginSlice, vector<int> bestModeVector, NetworkState &netState, SimulationData &simData) {
	cout << bestBeginSlice << " <---- bestBeginSlice" << endl;
	int n = bestRt.getNumOfSlices();
	for (int h = 0; h < bestRt.hops; h++) {
		int e = bestRt.linkVector[h];
		for (int s = bestBeginSlice; s <= n - 1 + bestBeginSlice; s++) {
			netState.xems[e][bestModeVector[e]][s]++;
			netState.yems[e][bestModeVector[e]][s] = d;
		}
		if (netState.xe[e] < bestBeginSlice + n)
			netState.xe[e] = bestBeginSlice + n;
		int firstAllocatedSlice = bestBeginSlice;
		for (int s = firstAllocatedSlice - 1; s >= 0; s--) {
			if (netState.zems[e][bestModeVector[e]][s] > 0) {
				firstAllocatedSlice = s + 1;
				break;
			}
			else if (s == 0) {
				firstAllocatedSlice = 0;
				break;
			}
		}
		int lastAllocatedSlice = bestBeginSlice + n - 1;
		for (int s = lastAllocatedSlice + 1; s <= simData.flexGrid.slices - 1; s++) {
			if (netState.zems[e][bestModeVector[e]][s] > 0) {
				lastAllocatedSlice = s - 1;
				break;
			}
			else if (s == simData.flexGrid.slices - 1) {
				lastAllocatedSlice = simData.flexGrid.slices - 1;
				break;
			}
		}
		int numOfSlicesAllocated = -(lastAllocatedSlice - firstAllocatedSlice + 1);
		for (int s = firstAllocatedSlice; s <= lastAllocatedSlice; s++) {
			netState.zems[e][bestModeVector[e]][s] = numOfSlicesAllocated;
			numOfSlicesAllocated++;
		}
		if (firstAllocatedSlice == bestBeginSlice) {
			int nextSliceFree = 1;
			for (int s = bestBeginSlice - 1; s >= 0; s--) {
				if (netState.zems[e][bestModeVector[e]][s] > 0) {
					netState.zems[e][bestModeVector[e]][s] = nextSliceFree;
					nextSliceFree += 1;
					int maxPossibleAllocation = -BIGINT;
					int minPossibleAllocation = BIGINT;
					for (int m=0; m<simData.net.modes; m++){
						if (maxPossibleAllocation < netState.zems[e][m][s]) maxPossibleAllocation = netState.zems[e][m][s];
						if (minPossibleAllocation > abs(netState.zems[e][m][s])) minPossibleAllocation = abs(netState.zems[e][m][s]);
						//if (maxPossibleAllocation == netState.zes[e][s]) break;
					}
					netState.zes[e][s] = maxPossibleAllocation; 
					netState.zes_min[e][s] = minPossibleAllocation; 
				}
				else
					break;
			}
		}
		for (int s = firstAllocatedSlice; s <= lastAllocatedSlice; s++) {
			int maxPossibleAllocation = -BIGINT;
			int minPossibleAllocation = BIGINT;
			for (int m=0; m<simData.net.modes; m++){
				if (maxPossibleAllocation < netState.zems[e][m][s]) maxPossibleAllocation = netState.zems[e][m][s];
				if (minPossibleAllocation > abs(netState.zems[e][m][s])) minPossibleAllocation = abs(netState.zems[e][m][s]);
				//if (maxPossibleAllocation == netState.zes[e][s]) break;
			}
			netState.zes[e][s] = maxPossibleAllocation;
			netState.zes_min[e][s] = minPossibleAllocation;
		}
		
		//if (firstAllocatedSlice == bestBeginSlice) {
		//	int nextSliceFree = 1;
		//	for (int s = bestBeginSlice - 1; s >= 0; s--) {
		//		int maxPossibleAllocation = -BIGINT;
		//		for (int m=0; m<simData.net.modes; m++){
		//			if (maxPossibleAllocation < netState.zems[e][m][s]) maxPossibleAllocation = netState.zems[e][m][s];
		//			if (maxPossibleAllocation == netState.zes[e][s]) break;
		//		}
		//		netState.zes[e][s] = maxPossibleAllocation;
		//	}
		//}
		
		

	}
	Channel bestCh = Channel(Channel(bestBeginSlice, bestBeginSlice - 1 + n, n));
	cout << d << bestRt.hops << bestModeVector[29] << bestCh.beginSlice << bestCh.endSlice << "d, Rt hops, bestModeVector[1], beginSlice, endSlice"<< endl;
	netState.allocatedLightpathsNew[d] = LightpathNew(d, bestRt, bestModeVector, bestCh);
}

void RSSA::addLpToSolution(int d, Route &bestRt, int &bestBeginSlice, int bestMode, NetworkState &netState, SimulationData &simData) {
	
	int channel_width = bestRt.getNumOfSlices();
	
	for (int h = 0; h < bestRt.hops; h++) {
		int e = bestRt.linkVector[h];
		for (int s = bestBeginSlice; s <= channel_width - 1 + bestBeginSlice; s++) {
			netState.xems[e][bestMode][s]++;
			netState.yems[e][bestMode][s] = d;
		}
		if (netState.xe[e] < bestBeginSlice + channel_width)
			netState.xe[e] = bestBeginSlice + channel_width;

		int firstAllocatedSlice = bestBeginSlice;
		for (int s = firstAllocatedSlice - 1; s >= 0; s--) {
			if (netState.zems[e][bestMode][s] > 0) {
				firstAllocatedSlice = s + 1;
				break;
			}
			else if (s == 0) {
				firstAllocatedSlice = 0;
				break;
			}
		}
		int lastAllocatedSlice = bestBeginSlice + channel_width - 1;
		for (int s = lastAllocatedSlice + 1; s <= simData.flexGrid.slices - 1; s++) {
			if (netState.zems[e][bestMode][s] > 0) {
				lastAllocatedSlice = s - 1;
				break;
			}
			else if (s == simData.flexGrid.slices - 1) {
				lastAllocatedSlice = simData.flexGrid.slices - 1;
				break;
			}
		}
		int numOfSlicesAllocated = -(lastAllocatedSlice - firstAllocatedSlice + 1);
		for (int s = firstAllocatedSlice; s <= lastAllocatedSlice; s++) {
			netState.zems[e][bestMode][s] = numOfSlicesAllocated;
			numOfSlicesAllocated++;
		}
		if (firstAllocatedSlice == bestBeginSlice) {
			int nextSliceFree = 1;
			for (int s = bestBeginSlice - 1; s >= 0; s--) {
				if (netState.zems[e][bestMode][s] > 0) {
					netState.zems[e][bestMode][s] = nextSliceFree;
					nextSliceFree += 1;
				}
				else
					break;
			}
		}
	}

	Channel bestCh = Channel(Channel(bestBeginSlice, bestBeginSlice - 1 + channel_width, channel_width));
	netState.allocatedLightpaths[d] = Lightpath(d, bestRt, bestMode, bestCh);
}

void RSSA::removeLpFromSolution(int d, Route &demandRoute, int &bestBeginSlice, vector<int> bestModeVector, NetworkState &netState, SimulationData &simData) {
	int n = demandRoute.getNumOfSlices();
	//cout << n << "-------------------------" << endl;
	for (int h = 0; h < demandRoute.hops; h++) {
		int e = demandRoute.linkVector[h];
		for (int s = bestBeginSlice; s <= n - 1 + bestBeginSlice; s++) {
			netState.xems[e][bestModeVector[e]][s] = 0;
			netState.yems[e][bestModeVector[e]][s] = -1;
		}
		if (netState.xe[e] > bestBeginSlice + n)
			netState.xe[e] = bestBeginSlice + n;
				int firstAllocatedSlice = bestBeginSlice;

				
		for (int s = firstAllocatedSlice - 1; s >= 0; s--) {
			if (netState.zems[e][bestModeVector[e]][s] > 0) {
				firstAllocatedSlice = s + 1;
				break;
			}
			else if (s == 0) {
				firstAllocatedSlice = 0;
				break;
			}
		}
		int lastAllocatedSlice = bestBeginSlice + n - 1;
		for (int s = lastAllocatedSlice + 1; s <= simData.flexGrid.slices - 1; s++) {
			if (netState.zems[e][bestModeVector[e]][s] > 0) {
				lastAllocatedSlice = s - 1;
				break;
			}
			else if (s == simData.flexGrid.slices - 1) {
				lastAllocatedSlice = simData.flexGrid.slices - 1;
				break;
			}
		}
		int numOfSlicesAllocated = (lastAllocatedSlice - firstAllocatedSlice + 1)+netState.zems[e][bestModeVector[e]][lastAllocatedSlice+1];
		for (int s = firstAllocatedSlice; s <= lastAllocatedSlice; s++) {
			netState.zems[e][bestModeVector[e]][s] = numOfSlicesAllocated;
			numOfSlicesAllocated--;
		}
		if (firstAllocatedSlice == bestBeginSlice) {
			int nextSliceFree = 1;
			for (int s = bestBeginSlice - 1; s >= 0; s--) {
				if (netState.zems[e][bestModeVector[e]][s] > 0) {
					netState.zems[e][bestModeVector[e]][s] = nextSliceFree;
					nextSliceFree += 1;
					int maxPossibleAllocation = -BIGINT;
					int minPossibleAllocation = BIGINT;
					for (int m=0; m<simData.net.modes; m++){
						if (maxPossibleAllocation < netState.zems[e][m][s]) maxPossibleAllocation = netState.zems[e][m][s];
						if (minPossibleAllocation > abs(netState.zems[e][m][s])) minPossibleAllocation = abs(netState.zems[e][m][s]);
						//if (maxPossibleAllocation == netState.zes[e][s]) break;
					}
					netState.zes[e][s] = maxPossibleAllocation; 
					netState.zes_min[e][s] = minPossibleAllocation; 
				}
				else
					break;
			}
		}
		for (int s = firstAllocatedSlice; s <= lastAllocatedSlice; s++) {
			int maxPossibleAllocation = -BIGINT;
			int minPossibleAllocation = BIGINT;
			for (int m=0; m<simData.net.modes; m++){
				if (maxPossibleAllocation < netState.zems[e][m][s]) maxPossibleAllocation = netState.zems[e][m][s];
				if (minPossibleAllocation > abs(netState.zems[e][m][s])) minPossibleAllocation = abs(netState.zems[e][m][s]);
				//if (maxPossibleAllocation == netState.zes[e][s]) break;
			}
			netState.zes[e][s] = maxPossibleAllocation;
			netState.zes_min[e][s] = minPossibleAllocation;
		}
	}
	// Channel bestCh = Channel(Channel(bestBeginSlice, bestBeginSlice - 1 + n, n));
	// remove d-th demand from lightpathNew vector 
	// netState.allocatedLightpathsNew = netState.allocatedLightpathsNew.erase (d);
	netState.allocatedLightpathsNew[d] = LightpathNew();
	
}


void RSSA::removeLpFromSolution(int d, Route &demandRoute, int &bestBeginSlice, int bestMode, NetworkState &netState, SimulationData &simData) {
	

	int channel_width = demandRoute.getNumOfSlices();
	

	for (int h = 0; h < demandRoute.hops; h++) {
		int e = demandRoute.linkVector[h];
		for (int s = bestBeginSlice; s <= channel_width - 1 + bestBeginSlice; s++) {
			netState.xems[e][bestMode][s]--;
			netState.yems[e][bestMode][s] = -1;
		}
		//if (netState.xe[e] > bestBeginSlice + channel_width) //Je¿eli xe przechowuje informacje na temat liczby zajêtych slice'ow na e-tym ³uku to trzeba rozpatrzec przypadek kiedy usuwany demand jest ze srodka gdzies
			//netState.xe[e] = bestBeginSlice + channel_width;

		int firstAllocatedSlice = bestBeginSlice;
		for (int s = firstAllocatedSlice - 1; s >= 0; s--) {
			if (netState.zems[e][bestMode][s] > 0) {
				firstAllocatedSlice = s + 1;
				break;
			}
			else if (s == 0) {
				firstAllocatedSlice = 0;
				break;
			}
		}
		int lastAllocatedSlice = bestBeginSlice + channel_width - 1;
		for (int s = lastAllocatedSlice + 1; s <= simData.flexGrid.slices - 1; s++) {
			if (netState.zems[e][bestMode][s] > 0) {
				lastAllocatedSlice = s - 1;
				break;
			}
			else if (s == simData.flexGrid.slices - 1) {
				lastAllocatedSlice = simData.flexGrid.slices - 1;
				break;
			}
		}
		int numOfSlicesAllocated = -(lastAllocatedSlice - firstAllocatedSlice + 1);
		for (int s = firstAllocatedSlice; s <= lastAllocatedSlice; s++) {
			netState.zems[e][bestMode][s] = numOfSlicesAllocated;
			numOfSlicesAllocated++;
		}
		if (firstAllocatedSlice == bestBeginSlice) {
			int nextSliceFree = 1;
			for (int s = bestBeginSlice - 1; s >= 0; s--) {
				if (netState.zems[e][bestMode][s] > 0) {
					netState.zems[e][bestMode][s] = nextSliceFree;
					nextSliceFree += 1;
				}
				else
					break;
			}
		}
	}

	Channel bestCh = Channel(Channel(bestBeginSlice, bestBeginSlice - 1 + channel_width, channel_width));
	netState.allocatedLightpaths[d] = Lightpath(d, demandRoute, bestMode, bestCh);
}


int RSSA::isBeginSliceEmpty(Route &rt, int beginSlice, int mode, NetworkState &netState, int channel_width) {
	
	for (int h = 0; h < rt.hops; h++) {// tutaj iterowac po modach i sprawdzic czy itnieje jakis mod na kazdym laczy na ktorym mamy wolen zasoby, kanal
		int e = rt.linkVector[h];
		//for int m.... 
		if (netState.zems[e][mode][beginSlice] < channel_width)
			return netState.zems[e][mode][beginSlice];
	}

	return BIGINT;
}


int RSSA::isBeginSliceEmpty(Route &rt, int beginSlice, NetworkState &netState, int channel_width, int modes, vector<int> modeVector) {
	int modeAllocationMethod = 1;
	for (int h = 0; h < rt.hops; h++) {// tutaj iterowac po modach i sprawdzic czy itnieje jakis mod na kazdym laczy na ktorym mamy wolen zasoby, kanal
		int e = rt.linkVector[h];
		int m = 0;
		int minBeginSliceOnMode = BIGINT;
		int minChannelWidth = BIGINT;
		int ModeOfMinBeginSlice;
		int chosenMode = -1;
		switch (modeAllocationMethod){
			case 1: { // wybór pierwszego dostêpnego modu i przerwanie pêtli
				for (m=0; m < modes; m++){
					if (netState.zems[e][m][beginSlice] < channel_width) {
						if (netState.zems[e][m][beginSlice] < minBeginSliceOnMode) {
							minBeginSliceOnMode=netState.zems[e][m][beginSlice];
						}
					}
					else if ((netState.zems[e][m][beginSlice] >= channel_width) && (chosenMode == -1)){
						chosenMode = m;
					}
					else break;
				}
				if (chosenMode==-1){
					return minBeginSliceOnMode;
				}
				else {
					modeVector[h] = chosenMode;
					cout<<"assigned" <<modeVector[h]<<h<< endl;
				}
				break;
			}
			case 2: { // wybór modu który posiada najmniejsz¹ mo¿liw¹ szerokoœæ kana³u do alokacji
				for (m=0; m < modes; m++){
					if (netState.zems[e][m][beginSlice] < channel_width) {
						if (netState.zems[e][m][beginSlice] < minBeginSliceOnMode) {
							minBeginSliceOnMode=netState.zems[e][m][beginSlice];
						}
					}
					else if ((netState.zems[e][m][beginSlice] >= channel_width) && (chosenMode == -1)){
						if ( netState.zems[e][m][beginSlice] < minChannelWidth){
							minChannelWidth = netState.zems[e][m][beginSlice];
							chosenMode = m;
						}
					}
					else break;
				}
				if (chosenMode==-1){
					return minBeginSliceOnMode;
				}
				else {
					modeVector[h] = chosenMode;
				}
				break;
			}
		}
	}
	return BIGINT;
}

int RSSA::isBeginSliceEmpty(Route &rt, int beginSlice, vector<int> mode, NetworkState &netState, int channel_width) {
	
	for (int h = 0; h < rt.hops; h++) {
		int e = rt.linkVector[h];
		if (netState.zems[e][mode[h]][beginSlice] < channel_width)
			return netState.zems[e][mode[h]][beginSlice];
	}

	return BIGINT;
}
void RSSA::updateBestLp(Route &bestRt, Channel &bestCh, int &bestMode, Route &rt, Channel &ch, int mode) {
	if (ch.size == 0 || mode == -1){
		cout << "mode value is -1, pause (bestMode)" << endl;
		system("pause");
	}
	if (bestCh.size == 0) {
		bestCh = ch, bestRt = rt, bestMode = mode;
	}
	else if (bestCh.endSlice > ch.endSlice) {
		bestCh = ch, bestRt = rt, bestMode = mode;
	}
	else if (bestCh.endSlice == ch.endSlice && bestRt.getLength() > rt.getLength()) {
		bestCh = ch, bestRt = rt, bestMode = mode;
	}
}
void RSSA::updateBestLp(Route &bestRt, int &bestBeginSlice, int &bestMode, Route &rt, int &beginSlice, int mode, int channel_width) {	
	if (channel_width == 0 || mode == -1){
		cout << "mode value is -1, pause (bestMode, channel_width)" << endl;
		system("pause");
	}
	if (bestBeginSlice == -1) {
		bestBeginSlice = beginSlice, bestRt = rt, bestMode = mode;
	}
	else if (bestBeginSlice > beginSlice) {
		bestBeginSlice = beginSlice, bestRt = rt, bestMode = mode;
	}
	else if (bestBeginSlice == beginSlice && bestRt.getLength() > rt.getLength()) {
		bestBeginSlice = beginSlice, bestRt = rt, bestMode = mode;
	}
}
void RSSA::updateBestLp(Route &bestRt, int &bestBeginSlice, vector<int> &bestModeVector, Route &rt, int &beginSlice, vector<int> modeVector, int n) {

	
	
	//for  (int h=0; h<rt.hops; h++){
		//int e = rt.linkVector[h];
		for  (int h=0; h<rt.hops; h++){
			int e = rt.linkVector[h];
			if (n == 0 || modeVector[e] == -1){cout << "mode value is -1, pause (bestModeVector)" << endl;}
		};
		if (bestBeginSlice == -1) {
			bestBeginSlice = beginSlice;
			bestRt = rt; 
			for  (int h=0; h<rt.hops; h++){
				int e = rt.linkVector[h];
				bestModeVector[e] = modeVector[e];
			};
		}
		else if (bestBeginSlice > beginSlice) {
			bestBeginSlice = beginSlice;
			bestRt = rt;
			for  (int h=0; h<rt.hops; h++){
				int e = rt.linkVector[h];
				bestModeVector[e] = modeVector[e];
			};
		}
		else if (bestBeginSlice == beginSlice && bestRt.getLength() > rt.getLength()) {
			bestBeginSlice = beginSlice; 
			bestRt = rt;
			for  (int h=0; h<rt.hops; h++){
				int e = rt.linkVector[h];
				bestModeVector[e] = modeVector[e];
			};
		};
	//}

}

void RSSA::updateBestLp(Route &bestRt, int &bestBeginSlice, Route &rt, int &beginSlice, int channel_width) {
	for  (int hop=0; hop<rt.hops; hop++){
		if (channel_width == 0){
			cout << "mode value is -1, pause (bestModeVector)" << endl;			
		}
		if (bestBeginSlice == -1) {
			bestBeginSlice = beginSlice, bestRt = rt;
		}
		else if (bestBeginSlice > beginSlice) {
			bestBeginSlice = beginSlice, bestRt = rt;
		}
		else if (bestBeginSlice == beginSlice && bestRt.getLength() > rt.getLength()) {
			bestBeginSlice = beginSlice, bestRt = rt;
		}
	}
}

void RSSA::SetupSingleOrder(int d, NetworkState &netState, SimulationData &simData) { // with mode conversion (MC)
	int c, e, h, i, j, k, m, n, q, s, t;
	int numOfRts;
	Route rt;
	Channel ch;
	int unservedDemands = 0;
	vector<int> modeVector(simData.net.links);
	vector<int> bestModeVector(simData.net.links);			
	numOfRts = simData.traffic.demandSet[d].candidateRoutes.size();
	bool isLpFound = false;
	Route bestRt;
	for (e = 0; e < simData.net.links; e++) {
		bestModeVector[e] = -1;
	};
	int bestBeginSlice = -1;
	for (k = 0; k < numOfRts; k++) {
		rt = simData.traffic.demandSet[d].candidateRoutes[k];
		n = rt.getNumOfSlices();		//number of required slices to assign the route
		if (n <= simData.flexGrid.slices) {
			int beginSlice = 0;
			while (beginSlice < simData.flexGrid.slices) {
				int status = BIGINT;
				for (int h = 0; h < rt.hops; h++) {// tutaj iterowac po modach i sprawdzic czy istnieje jakis mod na kazdym laczy na ktorym mamy wolne zasoby, kanal
					e = rt.linkVector[h]; // h-ta krawêdŸ œcie¿ki rt
					modeVector[e] = -1;
					if (netState.zes[e][beginSlice] < n) {
						status = netState.zes_min[e][beginSlice];
						//status = abs(netState.zes[e][beginSlice]);
						break;
					}
					for (m=0; m < simData.net.modes; m++){
						if (netState.zems[e][m][beginSlice] < n) {
							//wartosc bezwzgledna !!!
							if (abs(netState.zems[e][m][beginSlice]) < status){status = abs(netState.zems[e][m][beginSlice]);} // przypisanie najmniejszej wartosci beginSlice sposrod wszystkich modow
						}
						if (netState.zems[e][m][beginSlice] >= n){
							modeVector[e] = m;
							status = BIGINT;
							break;
						}
					}
					if (modeVector[e] == -1) {
						//cout << "na krawedzi sciezki nie znaleziono zadnego wolnego modu alokowanego na beginSlice -> (sciezka; przeskok; krawedz; demand; beginSlice) " << k << " " << h << " " << e << " " << d << " " << beginSlice << endl;
						//netState.zes[e][beginSlice] = 1;
						break; // wyjsc z petli iterujacej po hops'ach
					}; 
				};
				if (status == BIGINT){
					updateBestLp(bestRt, bestBeginSlice, bestModeVector, rt, beginSlice, modeVector, n);
					isLpFound = true;
					break;
				}
				else 
					beginSlice += status;
			}
		}
	}
	//cout << "- Selected allocation: route = " << bestRt.k << ", mode = " << bestMode << ", channel = [" << bestCh.beginSlice << ", " << bestCh.endSlice << "]" << endl << endl;
	if (!isLpFound) {
		cout << "Demand " << d << ": lightpath not found" << endl;
		unservedDemands++;

		//system("pause");
	}
	else addLpToSolution(d, bestRt, bestBeginSlice, bestModeVector, netState, simData);
	//double objectiveValue = GetObjValue(netState.GetMaxSliceIndex(), unservedDemands, simData);	

}


bool RSSA::SetupSingleOrderFS(int d, NetworkState &netState, SimulationData &simData) { // with mode conversion (MC)
	int c, e, h, i, j, k, m, n, q, s, t;
	int numOfRts;
	Route rt;
	Channel ch;
	int unservedDemands = 0;
	vector<int> modeVector(simData.net.links);
	vector<int> bestModeVector(simData.net.links);			
	numOfRts = simData.traffic.demandSet[d].candidateRoutes.size();
	bool isLpFound = false;
	Route bestRt;
	for (e = 0; e < simData.net.links; e++) {
		bestModeVector[e] = -1;
	};
	int bestBeginSlice = -1;
	for (k = 0; k < numOfRts; k++) {
		rt = simData.traffic.demandSet[d].candidateRoutes[k];
		n = rt.getNumOfSlices();		//number of required slices to assign the route
		if (n <= simData.flexGrid.slices) {
			int beginSlice = 0;
			while (beginSlice < simData.flexGrid.slices) {
				int status = BIGINT;
				for (int h = 0; h < rt.hops; h++) {// tutaj iterowac po modach i sprawdzic czy istnieje jakis mod na kazdym laczy na ktorym mamy wolne zasoby, kanal
					e = rt.linkVector[h]; // h-ta krawêdŸ œcie¿ki rt
					modeVector[e] = -1;
					if (netState.zes[e][beginSlice] < n) {
						status = netState.zes_min[e][beginSlice];
						//status = abs(netState.zes[e][beginSlice]);
						break;
					}
					for (m=0; m < simData.net.modes; m++){
						if (netState.zems[e][m][beginSlice] < n) {
							//wartosc bezwzgledna !!!
							if (abs(netState.zems[e][m][beginSlice]) < status){status = abs(netState.zems[e][m][beginSlice]);} // przypisanie najmniejszej wartosci beginSlice sposrod wszystkich modow
						}
						if (netState.zems[e][m][beginSlice] >= n){
							modeVector[e] = m;
							status = BIGINT;
							break;
						}
					}
					if (modeVector[e] == -1) {
						//cout << "na krawedzi sciezki nie znaleziono zadnego wolnego modu alokowanego na beginSlice -> (sciezka; przeskok; krawedz; demand; beginSlice) " << k << " " << h << " " << e << " " << d << " " << beginSlice << endl;
						//netState.zes[e][beginSlice] = 1;
						break; // wyjsc z petli iterujacej po hops'ach
					}; 
				};
				if (status == BIGINT){
					updateBestLp(bestRt, bestBeginSlice, bestModeVector, rt, beginSlice, modeVector, n);
					isLpFound = true;
					break;
				}
				else 
					beginSlice += status;
			}
		}
	}
	//cout << "- Selected allocation: route = " << bestRt.k << ", mode = " << bestMode << ", channel = [" << bestCh.beginSlice << ", " << bestCh.endSlice << "]" << endl << endl;
	if (!isLpFound) {
		cout << "Demand " << d << ": lightpath not found" << endl;
		unservedDemands++;
		return 0;
		//system("pause");
	}
	else addLpToSolution(d, bestRt, bestBeginSlice, bestModeVector, netState, simData);
	//double objectiveValue = GetObjValue(netState.GetMaxSliceIndex(), unservedDemands, simData);	
	return 1;
}



void RSSA::TearDownSingleOrder(int d, NetworkState &netState, SimulationData &simData) { // with mode conversion (MC)
	int c, e, h, i, j, k, m, n, q, s, t;
	int numOfRts;
	Route rt;
	Channel ch;
	int unservedDemands = 0;
	vector<int> modeVector(simData.net.links);
	vector<int> bestModeVector(simData.net.links);			
	
		numOfRts = simData.traffic.demandSet[d].candidateRoutes.size();
		bool isLpFound = false;
		Route bestRt;
		for (e = 0; e < simData.net.links; e++) {
			bestModeVector[e] = -1;
		};
		int bestBeginSlice = -1;
		for (k = 0; k < numOfRts; k++) {
			rt = simData.traffic.demandSet[d].candidateRoutes[k];
			n = rt.getNumOfSlices();		//number of required slices to assign the route
			if (n <= simData.flexGrid.slices) {
				int beginSlice = 0;
				while (beginSlice < simData.flexGrid.slices) {
					int status = BIGINT;
					
					for (int h = 0; h < rt.hops; h++) {// tutaj iterowac po modach i sprawdzic czy istnieje jakis mod na kazdym laczy na ktorym mamy wolne zasoby, kanal
						e = rt.linkVector[h]; // h-ta krawêdŸ œcie¿ki rt
						modeVector[e] = -1;
						if (netState.zes[e][beginSlice] < n) {
							status = netState.zes_min[e][beginSlice];
							//status = abs(netState.zes[e][beginSlice]);
							break;
						}
						for (m=0; m < simData.net.modes; m++){
							if (netState.zems[e][m][beginSlice] < n) {
								//wartosc bezwzgledna !!!
								if (abs(netState.zems[e][m][beginSlice]) < status){status = abs(netState.zems[e][m][beginSlice]);} // przypisanie najmniejszej wartosci beginSlice sposrod wszystkich modow
							}
							if (netState.zems[e][m][beginSlice] >= n){
								modeVector[e] = m;
								status = BIGINT;
								break;
							}
						}
						if (modeVector[e] == -1) {
							//cout << "na krawedzi sciezki nie znaleziono zadnego wolnego modu alokowanego na beginSlice -> (sciezka; przeskok; krawedz; demand; beginSlice) " << k << " " << h << " " << e << " " << d << " " << beginSlice << endl;
							//netState.zes[e][beginSlice] = 1;
							break; // wyjsc z petli iterujacej po hops'ach
						}; 
					};
					if (status == BIGINT){
						updateBestLp(bestRt, bestBeginSlice, bestModeVector, rt, beginSlice, modeVector, n);
						isLpFound = true;
						break;
					}
					else 
						beginSlice += status;
					
				}
			}
		}
		//cout << "- Selected allocation: route = " << bestRt.k << ", mode = " << bestMode << ", channel = [" << bestCh.beginSlice << ", " << bestCh.endSlice << "]" << endl << endl;
		if (!isLpFound) {
			//			cout << "Demand " << d << ": lightpath not found" << endl;
			unservedDemands++;
		}
		//else addLpToSolution(d, bestRt, bestBeginSlice, bestModeVector, netState, simData);
		else removeLpFromSolution(d, bestRt, bestBeginSlice, bestModeVector, netState, simData);
	double objectiveValue = GetObjValue(netState.GetMaxSliceIndex(), unservedDemands, simData);
	
}