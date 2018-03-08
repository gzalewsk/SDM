#include "StdAfx.h"
#include "Heur_RSSA.h"
#include <random>
#include "cfg.h"
#include <algorithm>


Heur_RSSA::Heur_RSSA(SimulationParameters *simPar, AlgorithmParameters *algPar, SupportingFunctions *supFun, NetworkScenario *netScen, FileWriter *fileWriter) {
	
	//(*this).simPar=simPar;
	this->simPar = simPar;
	this->algPar = algPar;
	this->supFun = supFun;
	this->netScen = netScen;
	this->fileWriter = fileWriter;
	
	maxSliceIndex = netScen->flexGrid->slots;
	bestObjective = IloInfinity;

	srand(algPar->RandGenSeed);

}

void Heur_RSSA::FFkSP() {

	cout << "- Running FFkSP heuristic algorithm" << endl;

	clock_t tStart = clock();

	NetworkState netState = NetworkState(netScen);
	vector<int> order = vector<int>(netScen->traffic->demands);
	for(int d = 0; d < netScen->traffic->demands; d++)
		order[d] = d;

	permuteDemands(order);

	bool status = FFkSP(order, netState);
	netState.Verify();
	
	bestObjective = GetObjValue(netState.GetMaxSliceIndex());
	maxSliceIndex = netState.GetMaxSliceIndex();
	
	double runTime = (double)(clock() - tStart) / CLOCKS_PER_SEC;
	cout << "  - Solution found: objVal = " << bestObjective << ", |S| = " << maxSliceIndex << endl;
	cout << "  - Run time: " << runTime << endl;
	cout << "----------------------------------------" << endl;

	ostringstream scen;
	scen  << "\t" << bestObjective << "\t" << maxSliceIndex;
	fileWriter->Add(0, scen.str());

 //	fileWriter->Add(0, "\t");
	//fileWriter->Add(0, maxSliceIndex);
 //	fileWriter->Add(0, "\t");
	//fileWriter->Add(0, runTime);
}

int Heur_RSSA::FFkSP(vector<int> &order, NetworkState &netState) {
	// konkretnie co ta funkcja robi? czy to jest zach³anne przypisywanie LP (lightpath) œcierzek routingu? <--------------------------------------------------------------------
	int c, d, e, h, i, j, k, n, q, s, t;
	int numOfRts;
	RoutePtr rt, bestRt;
	ChannelPtr ch, bestCh;

	int unservedDemands = 0;

	for(j = 0; j < order.size(); j++) {
		d = order[j];	
		numOfRts = netScen->traffic->demandSet[d].candidateRoutes.size();
		bool isLpFound = false;
	
		bestRt = NULL;
		bestCh = NULL;

		for(k = 0; k < numOfRts; k++) {
			rt = netScen->traffic->demandSet[d].candidateRoutes[k];

			n = rt->getNumOfSlots();

			if(n <= netScen->flexGrid->slots) {
				for(c = 0; c < netScen->flexGrid->chs[n].size(); c++) {
					ch = netScen->flexGrid->chs[n][c];
					if(ch != NULL) {
						if(isChannelEmpty(rt, ch, netState)) {
							updateBestLp(bestRt, bestCh, rt, ch); // tutaj jest update otrzymanego rozwi¹zania
							isLpFound = true;
							break;
						}
					}
				}
			}
		}

		if(!isLpFound) {
			cout << "Demand " << d << ": lightpath not found" << endl;
			unservedDemands++;
		} else
			addLpToSolution(d, bestRt, bestCh, netState);
	}

	return unservedDemands;
}

void Heur_RSSA::SimAn_FFkSP() {

	int d, d1, d2, e, i, j;
	int bestIter = -IloInfinity;
	double runTime;
	cout << "- Running SimAn-FF heuristic algorithm" << endl;
	cout << "  - heurSAiterLimit: " << algPar->heurSAiterLimit << endl;
	cout << "  - heurSAcooling: " << algPar->heurSAcooling << endl;
	cout << "  - heurSAtemperatureCoeff: " << algPar->heurSAtemperatureCoeff << endl;
	cout << "  - heurSAtimeLimit: " << algPar->heurSAtimeLimit << endl;
	clock_t tStart = clock();

	NetworkState netState = NetworkState(netScen);
	NetworkState bestNetState = NetworkState(netScen);
	vector<int> order = vector<int>(netScen->traffic->demands);
	
	//--- ordering demands
	for (int j = 0; j < netScen->traffic->demands; j++)
		order[j] = j;
	permuteDemands(order);
	maxSliceIndex = netScen->flexGrid->slices + 1;
	bestObjective = BIGNUMBER;
	
	FFkSP(order, netState);
	maxSliceIndex = netState.GetMaxSliceIndex();
	bestObjective = GetObjValue(netState.GetMaxSliceIndex());
	DisplayResults(netState); //wypisuje wartosci wyznaczone przez funkcjê FFkSP
	bestIter = 0;

	int tempS = maxSliceIndex * 2;
	//netScen->flexGrid->UpdateNumberOfSlices(tempS);

	ostringstream scen1;
	scen1 << "\t" << bestObjective << "\t" << maxSliceIndex;
	fileWriter->Add(0, scen1.str());

	int currentMaxSliceIndex = maxSliceIndex;
	double currentBestObjVal = bestObjective;

	bestNetState = netState;
	netState.ClearState();

	double initTemperature = bestObjective * algPar->heurSAtemperatureCoeff;
	double temperature = initTemperature;
	double cooling = algPar->heurSAcooling;

	long iter = 0;

	while (iter < algPar->heurSAiterLimit) {

		// generate neighbouring solution by swapping two demands
		d1 = (int)(rand() % netScen->traffic->demands);
		d2 = (int)(rand() % (netScen->traffic->demands - 1));
		if (d1 <= d2) d2++;
		int temp = order[d1]; order[d1] = order[d2]; order[d2] = temp;

		// find RSSA solution
		FFkSP(order, netState);

		int tempA = netState.GetMaxSliceIndex();
		int delta = tempA - currentMaxSliceIndex;
		double tempObjVal = GetObjValue(netState.GetMaxSliceIndex());
		double deltaObjVal = tempObjVal - currentBestObjVal;

		//if(delta <= 0) {
		if (deltaObjVal <= 0) {

			currentMaxSliceIndex = tempA;
			currentBestObjVal = tempObjVal;

			//if(currentMaxSliceIndex < maxSliceIndex) {
			if (currentBestObjVal < bestObjective) {
				maxSliceIndex = currentMaxSliceIndex;
				bestObjective = currentBestObjVal;

				bestNetState.ClearState();
				bestNetState = netState;
				bestIter = iter;

				//DisplayResults(netState, iter);
				runTime = (double)(clock() - tStart) / CLOCKS_PER_SEC;
				DisplayResults(netState, iter, runTime);
			}

		}
		else {

			//if((rand() % 1000) / 1000.0 < exp((double) -delta/temperature)) {
			if ((rand() % 1000) / 1000.0 < exp((double)-deltaObjVal / temperature)) {
				currentMaxSliceIndex = tempA;
				currentBestObjVal = tempObjVal;
			}
			else {
				temp = order[d1]; order[d1] = order[d2]; order[d2] = temp;
			}
		}

		netState.ClearState();

		temperature = temperature * cooling;
		iter++;

		runTime = (double)(clock() - tStart) / CLOCKS_PER_SEC;
		if (runTime > algPar->heurSAtimeLimit) {
			cout << "Time exceeded: " << runTime << " > " << algPar->heurSAtimeLimit << endl;
			break;
		}
	}
	/*
	cout << "  - Stopping criteria: ";
	if (iter >= algPar->heurSAiterLimit)
		cout << "iteration limit exceeded (" << iter << " >= " << algPar->heurSAiterLimit << ")";
	else if (temperature <= algPar->heurSAtempLimit)
		cout << "low temperature (" << temperature << " <= " << algPar->heurSAtempLimit << ")";
	else
		cout << "time limit (" << runTime << " > " << algPar->heurSAtimeLimit << ")";
	*/

	cout << endl;

	runTime = (double)(clock() - tStart) / CLOCKS_PER_SEC;
	cout << "  - Solution found at iter " << bestIter << ": objVal = " << bestObjective << ", |S| = " << maxSliceIndex << endl;
	cout << "  - Run time: " << runTime << endl;
	cout << "----------------------------------------" << endl;

	ostringstream scen;
	scen << "\t" << bestIter << "\t" << bestObjective << "\t" << maxSliceIndex;
	fileWriter->Add(0, scen.str());
}

void Heur_RSSA::permuteDemands(vector<int> &order) {
	unsigned seed = chrono::system_clock::now().time_since_epoch().count();
	auto engine = default_random_engine(seed);
	//random_shuffle(order.begin(), order.end()); // poprzednia funkcja losuj¹ca pseudo losowe liczby. Shuffle+seed pozwala znaleŸæ realnie losowe liczby
	shuffle(order.begin(), order.end(), engine);
}

bool Heur_RSSA::isChannelEmpty(RoutePtr &rt, ChannelPtr &ch, NetworkState &netState) {

	for(int h = 0; h < rt->hops; h++) {
		int e = rt->linkVector[h];
		for(int s = ch->beginSlice; s <= ch->endSlice; s++)
			if(netState.xes[e][s] > 0)
				return false;
	}

	return true;
}

void Heur_RSSA::updateBestLp(RoutePtr &bestRt, ChannelPtr &bestCh, RoutePtr &rt, ChannelPtr &ch) {

	if(bestCh == NULL)
		bestCh = ch, bestRt = rt;
	else if(bestCh->endSlice > ch->endSlice)
			bestCh = ch, bestRt = rt;
		else if(bestCh->endSlice == ch->endSlice && bestRt->getLength() > rt->getLength())
			bestCh = ch, bestRt = rt;
}

void Heur_RSSA::addLpToSolution(int d, RoutePtr &bestRt, ChannelPtr &bestCh, NetworkState &netState) {

	updateNetworkState(bestRt, bestCh->beginSlice, bestCh->endSlice, netState);
}

void Heur_RSSA::updateNetworkState(RoutePtr &bestRt, int beginSlice, int endSlice, NetworkState &netState) {

	for (int h = 0; h < bestRt->hops; h++) {
		int e = bestRt->linkVector[h];
		for (int s = beginSlice; s <= endSlice; s++)
			netState.xes[e][s]++;
		if (netState.xe[e] < endSlice + 1)
			netState.xe[e] = endSlice + 1;
	}
}

int Heur_RSSA::GetMaxSliceIndex() {

	return maxSliceIndex;
}

double Heur_RSSA::GetObjValue(int slices) {

	double objective = slices;

	return objective;
}

void Heur_RSSA::DisplayResults() {
	mtx_cout.lock();
	cout << "    - Max slice index: " << maxSliceIndex << endl << flush;
	mtx_cout.unlock();
}

void Heur_RSSA::DisplayResults(NetworkState &netState) {
	mtx_cout.lock();
	cout << "    - Best solution: objVal = " << bestObjective << ", |S| = " << maxSliceIndex << endl;
	mtx_cout.unlock();
}

void Heur_RSSA::DisplayResults(NetworkState &netState, int iter) {
	mtx_cout.lock();
	cout << "    - Best solution: objVal = " << bestObjective << ", |S| = " << maxSliceIndex << endl;
	mtx_cout.unlock();
}

void Heur_RSSA::DisplayResults(NetworkState &netState, int iter, double time) {
	mtx_cout.lock();
	cout << "    - Best solution: objVal = " << bestObjective << ", |S| = " << maxSliceIndex << endl;
	mtx_cout.unlock();
}

void Heur_RSSA::DisplayResults(string description) {
	mtx_cout.lock();
	cout << "    - Max slice index: " << maxSliceIndex << " (" << description << ")" << endl;
	mtx_cout.unlock();
}

// Ponizej funkcje zmodyfikowane przez Gregor
void Heur_RSSA_gregor::wypisz_wymaluj(string str) {
	cout << str << endl;
} // taka przykladowa funkcja testowa

void Heur_RSSA::SimAn_FFkSP_gr() {

	int d, d1, d2, e, i, j;
	int bestIter = -IloInfinity;
	double runTime;

	/*
	cout << "- Running SimAn-FF heuristic algorithm" << endl;
	cout << "  - heurSAiterLimit: " << algPar->heurSAiterLimit << endl;
	cout << "  - heurSAcooling: " << algPar->heurSAcooling << endl;
	cout << "  - heurSAtemperatureCoeff: " << algPar->heurSAtemperatureCoeff << endl;
	cout << "  - heurSAtimeLimit: " << algPar->heurSAtimeLimit << endl;
	*/
	clock_t tStart = clock();

	NetworkState netState = NetworkState(netScen);
	NetworkState bestNetState = NetworkState(netScen);
	vector<int> order = vector<int>(netScen->traffic->demands);
	
	/*
	for (int h = 0; h < order.size() ; h++ ) {	cout << to_string(order[h]) << "-";	} cout << endl;
	*/
	for (int j = 0; j < netScen->traffic->demands; j++) order[j] = j;
	/*
	for (int h = 0; h < order.size() ; h++ ) {	cout << to_string(order[h]) << "-";	} cout << endl;
	*/
	//--- ordering demands
	permuteDemands(order);
	/*
	for (int h = 0; h < order.size() ; h++ ) {		cout << to_string(order[h]) << "-";	}	cout << endl;
	/*
	for (int vector_value = 0; vector_value < order.size(); vector_value++){
		cout << "---------------> value of order vector " << to_string(vector_value) << " is " << to_string(order[vector_value]) << endl;
	};
	*/
	maxSliceIndex = netScen->flexGrid->slices + 1;
	bestObjective = BIGNUMBER;
	//cout << to_string(maxSliceIndex) << "---------------> value of maxSliceIndex" << endl;

	FFkSP(order, netState);
	maxSliceIndex = netState.GetMaxSliceIndex();
	bestObjective = GetObjValue(netState.GetMaxSliceIndex());
	DisplayResults(netState); //wypisuje wartosci wyznaczone przez funkcjê FFkSP
	bestIter = 0;

	int tempS = maxSliceIndex * 2;
	//netScen->flexGrid->UpdateNumberOfSlices(tempS);

	ostringstream scen1;
	scen1 << "\t" << bestObjective << "\t" << maxSliceIndex;
	fileWriter->Add(0, scen1.str());

	int currentMaxSliceIndex = maxSliceIndex;
	double currentBestObjVal = bestObjective;

	bestNetState = netState;
	netState.ClearState();

	double initTemperature = bestObjective * algPar->heurSAtemperatureCoeff;
	double temperature = initTemperature;
	double cooling = algPar->heurSAcooling;

	long iter = 0;

	//cout << to_string(algPar->heurSAiterLimit) << "---------------> value of heurSAiterLimit" << endl;

	//GLOWNA PETLA KTORA WPLYWA NA CZAS OBLICZEN
	while (iter < algPar->heurSAiterLimit) {
		// generate neighbouring solution by swapping two demands
		// tutaj pocz¹tek 
		d1 = (int)(rand() % netScen->traffic->demands); 
		d2 = (int)(rand() % (netScen->traffic->demands - 1));
		if (d1 <= d2) d2++;
		
		
		
		int temp = order[d1]; order[d1] = order[d2]; order[d2] = temp;
		// zakoñczenie podstawiania rozwi¹zania s¹siaduj¹cego
		// find RSSA solution
		FFkSP_thread(order, netState, 1);
		// znalezienie rozwi¹zania dla wartoœci wektora order - funkcja FFkSP_thread posiada mutexy wiêc powinno byæ dobrze
		int tempA = netState.GetMaxSliceIndex();
		int delta = tempA - currentMaxSliceIndex;
		double tempObjVal = GetObjValue(netState.GetMaxSliceIndex());
		double deltaObjVal = tempObjVal - currentBestObjVal;
		//if(delta <= 0) {
		if (deltaObjVal <= 0) {
			currentMaxSliceIndex = tempA; // ale to ju¿ jest zmienna dla ca³ej pêtli wiêc nale¿y to zabezpieczyæ
			currentBestObjVal = tempObjVal; // to te¿
			//if(currentMaxSliceIndex < maxSliceIndex) {
			if (currentBestObjVal < bestObjective) {
				maxSliceIndex = currentMaxSliceIndex;
				bestObjective = currentBestObjVal;
				bestNetState.ClearState();
				bestNetState = netState;
				bestIter = iter;
				//DisplayResults(netState, iter);
				runTime = (double)(clock() - tStart) / CLOCKS_PER_SEC;			
				DisplayResults(netState, iter, runTime);			
			}
		}
		else {

			//if((rand() % 1000) / 1000.0 < exp((double) -delta/temperature)) {
			if ((rand() % 1000) / 1000.0 < exp((double)-deltaObjVal / temperature)) {
				currentMaxSliceIndex = tempA;
				currentBestObjVal = tempObjVal;
			}
			else {
				temp = order[d1]; order[d1] = order[d2]; order[d2] = temp;
			}
		}
		netState.ClearState();
		temperature = temperature * cooling;
		iter++;
		runTime = (double)(clock() - tStart) / CLOCKS_PER_SEC;
		if (runTime > algPar->heurSAtimeLimit) {
			cout << "Time exceeded: " << runTime << " > " << algPar->heurSAtimeLimit << endl;
			break;
		}
	}
	// KONIEC GLOWNEJ PETLI WPLYWAJACEJ NA CZAS OBLICZEN
		cout << "  - Stopping criteria: ";
		if (iter >= algPar->heurSAiterLimit)
			cout << "iteration limit exceeded (" << iter << " >= " << algPar->heurSAiterLimit << ")" << endl;
		else if (temperature <= algPar->heurSAtempLimit)
			cout << "low temperature (" << temperature << " <= " << algPar->heurSAtempLimit << ")" << endl;
		else
			cout << "time limit (" << runTime << " > " << algPar->heurSAtimeLimit << ")" << endl;

	runTime = (double)(clock() - tStart) / CLOCKS_PER_SEC;
	cout << "  - Solution found at iter " << bestIter << ": objVal = " << bestObjective << ", |S| = " << maxSliceIndex << endl;
	cout << "  - Run time: " << runTime << endl;
	cout << "----------------------------------------" << endl;
	ostringstream scen;
	scen << "\t" << bestIter << "\t" << bestObjective << "\t" << maxSliceIndex;
	fileWriter->Add(0, scen.str());
	bestObjective_public = bestObjective;
	maxSliceIndex_public = maxSliceIndex;
}

int Heur_RSSA::FFkSP_thread(vector<int> &order, NetworkState &netState, int threads) {

	int c, d, e, h, i, j, k, n, q, s, t;
	int numOfRts;
	RoutePtr rt, bestRt;
	ChannelPtr ch, bestCh;
	int unservedDemands = 0;
	for(j = 0; j < order.size(); j++) {
		d = order[j];	
		numOfRts = netScen->traffic->demandSet[d].candidateRoutes.size();
		bool isLpFound = false;
		bestRt = NULL;
		bestCh = NULL;
		for(k = 0; k < numOfRts; k++) {
			rt = netScen->traffic->demandSet[d].candidateRoutes[k];
			n = rt->getNumOfSlots();
			if(n <= netScen->flexGrid->slots) {
				for(c = 0; c < netScen->flexGrid->chs[n].size(); c++) {
					ch = netScen->flexGrid->chs[n][c];
					if(ch != NULL) {
						if(isChannelEmpty(rt, ch, netState)) {
							mtx_FFkSP.lock();
							updateBestLp(bestRt, bestCh, rt, ch);
							mtx_FFkSP.unlock();
							isLpFound = true;
							break;
						}
					}
				}
			}
		}
		if(!isLpFound) {
			cout << "Demand " << d << ": lightpath not found" << endl;
			unservedDemands++;
		} else
			mtx_FFkSP.lock();
			addLpToSolution(d, bestRt, bestCh, netState);
			mtx_FFkSP.unlock();
	}
	return unservedDemands;
}

void Heur_RSSA::SimAn_FFkSP_gr2() {

	int d, e, i, j;
	int bestIter = -IloInfinity;
	double runTime;
	cout << "- Running SimAn-FF heuristic algorithm" << endl;
	cout << "  - heurSAiterLimit: " << algPar->heurSAiterLimit << endl;
	cout << "  - heurSAcooling: " << algPar->heurSAcooling << endl;
	cout << "  - heurSAtemperatureCoeff: " << algPar->heurSAtemperatureCoeff << endl;
	cout << "  - heurSAtimeLimit: " << algPar->heurSAtimeLimit << endl;
	clock_t tStart = clock();

	NetworkState netState = NetworkState(netScen);
	NetworkState bestNetState = NetworkState(netScen);
	order = vector<int>(netScen->traffic->demands);
	
	//--- ordering demands
	for (int j = 0; j < netScen->traffic->demands; j++)
		order[j] = j;
	permuteDemands(order);
	maxSliceIndex = netScen->flexGrid->slices + 1;
	bestObjective = BIGNUMBER;
	

	FFkSP(order, netState);
	maxSliceIndex = netState.GetMaxSliceIndex();
	bestObjective = GetObjValue(netState.GetMaxSliceIndex());
	DisplayResults(netState); //wypisuje wartosci wyznaczone przez funkcjê FFkSP
	bestIter = 0;

	int tempS = maxSliceIndex * 2;
	//netScen->flexGrid->UpdateNumberOfSlices(tempS);

	ostringstream scen1;
	scen1 << "\t" << bestObjective << "\t" << maxSliceIndex;
	fileWriter->Add(0, scen1.str());

	int currentMaxSliceIndex = maxSliceIndex;
	double currentBestObjVal = bestObjective;

	bestNetState = netState;
	netState.ClearState();
	double initTemperature = bestObjective * algPar->heurSAtemperatureCoeff;
	double temperature = initTemperature;
	double cooling = algPar->heurSAcooling;

	// szukanie szeregowe
	/*
	for (int init = 0; init<4 ; init++){
		threadyInit(init, 100);
	};
	*/
	
	// zrównoloeglenie poszukiwana rozwi¹zañ wstêpnych
	const int threads_num = 4;
	thread t[threads_num];

	/*	
	for (i= 0; i<threads_num; i++){
		t[i] = thread (&Heur_RSSA::threadyInit, this, i, 1);
	}
	for (i= 0; i<threads_num; i++){
		t[i].join();	
	} 
	*/
	/*
	for (i= 0; i<threads_num; i++){
		t[i] = thread (&Heur_RSSA::threadyWorker, this, i);
	}
	for (i= 0; i<threads_num; i++){
		t[i].join();	
	} 
	*/
	for (i= 0; i<threads_num; i++){
		t[i] = thread (&Heur_RSSA::pureHeuristic, this, i);
	}
	for (i= 0; i<threads_num; i++){
		t[i].join();	
	} 

	system("pause");




	//- -----------------------------------------------------------------------------------------------------
	/*
	cout << "temp, initTemperature, bestObjective, temperature, cooling " << netState.GetMaxSliceIndex() << " , " << bestObjective << " , " << initTemperature << " , " 
		<< temperature << " , " << cooling << endl;
	system("pause");
	cout << endl;
	for (int thr = 0 ; thr < 4 ; thr++){
		for (int i =0 ; i<initialSolutionsOrders[thr].size() ; i++){
			cout << initialSolutionsOrders[thr][i] << "," ;
		}
		cout << endl;
	}
	*/


	

	/*
	for (int thr = 0 ; thr < 4 ; thr++){
		for (int i =0 ; i<order.size() ; i++){
			if (initialSolutionsOrders[thr].size() > 0)
			cout << thr << "," << i << " " << initialSolutionsOrders[thr][i] << ',' << endl;
			else
				cout << thr << "," << i << " pusty" << endl;
		};
		cout << endl;
	}
	*/
	// teraz tutaj powinna byc lista wektorow z rozwiazaniami inicjalizowanymi
	// initialSolutionsOrders[]
	// i ruszamy z wyzarzaniem
	
	/*
	for (i= 0; i<threads_num; i++){
		t[i] = thread (&Heur_RSSA::threadyWorker, this, i, initialSolutionsOrders[i]);
	}
	for (i= 0; i<threads_num; i++){
		t[i].join();	
	} 
	*/
	/*
	cout << threadyCurrentMaxSliceIndex << ", " << threadyCurrentBestObjVal << endl;
	for (i= 0; i<threadyBestOrder.size(); i++){ cout << threadyBestOrder[i] <<  ","; }
	cout << endl;
	*/
	
}

void Heur_RSSA::neightbourGen( int &d1, int &d2, int orderCount) {
	d1 = (int)(rand() % orderCount);
	d2 = (int)(rand() % (orderCount - 1));
	if (d1 <= d2) d2++;
	temp = order[d1]; order[d1] = order[d2]; order[d2] = temp;
}

void Heur_RSSA::threadyFFkSP(int &d1, int &d2, int orderCount, vector<int> &order, NetworkState &netState) {
	neightbourGen(d1, d2, orderCount);
	FFkSP(order, netState);
}

void Heur_RSSA::threadyInit(int thr, int divide = 1){
	int currentMaxSliceIndex = maxSliceIndex;
	double currentBestObjVal = bestObjective;
	NetworkState netState = NetworkState(netScen);
	NetworkState bestNetState = NetworkState(netScen);
	order = vector<int>(netScen->traffic->demands);
	for (int j = 0; j < netScen->traffic->demands; j++)
		order[j] = j;
	permuteDemands(order);
	maxSliceIndex = netScen->flexGrid->slices + 1;
	threadyCurrentMaxSliceIndex = maxSliceIndex;
	bestObjective = BIGNUMBER;
	threadyCurrentBestObjVal = bestObjective;
	bestNetState = netState;
	netState.ClearState();
	int bestIter = -IloInfinity;
	double initTemperature = bestObjective * algPar->heurSAtemperatureCoeff;
	double temperature = initTemperature;
	double cooling = algPar->heurSAcooling;
	vector<int> bestOrder = order;
	long iter = 0;
	//cout << initTemperature << " <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"<<endl;
	//system("pause");
	while (iter < algPar->heurSAiterLimit/(divide)) {
			// generate neighbouring solution by swapping two demands
			int orderCount = netScen->traffic->demands;
			int d1, d2;
			threadyFFkSP(d1, d2, orderCount, order, netState);
			int tempA = netState.GetMaxSliceIndex();
			int delta = tempA - currentMaxSliceIndex;
			double tempObjVal = GetObjValue(netState.GetMaxSliceIndex());
			double deltaObjVal = tempObjVal - threadyCurrentBestObjVal;

			//if(delta <= 0) {
			if (deltaObjVal <= 0) {
				currentMaxSliceIndex = tempA;
				currentBestObjVal = tempObjVal;

				//if(currentMaxSliceIndex < maxSliceIndex) {
				if (currentBestObjVal < bestObjective) {
					maxSliceIndex = currentMaxSliceIndex;
					bestObjective = currentBestObjVal;

					bestNetState.ClearState();
					bestNetState = netState;
					bestIter = iter;
					bestOrder = order;
					//DisplayResults(netState, iter);
					//runTime = (double)(clock() - tStart) / CLOCKS_PER_SEC;
					//DisplayResults(netState, iter, runTime);

					//if (divide = 1) {
					updateThreadyBestValues(maxSliceIndex, bestObjective, bestOrder);
					//}
					
					cout << "    - Max slice index: " << maxSliceIndex << " in thread" << thr << endl << flush;
				}
		
		}
		else {

			//if((rand() % 1000) / 1000.0 < exp((double) -delta/temperature)) {
			if ((rand() % 1000) / 1000.0 < exp((double)-deltaObjVal / temperature)) {
				currentMaxSliceIndex = tempA;
				currentBestObjVal = tempObjVal;
			}
			else {
				temp = order[d1]; order[d1] = order[d2]; order[d2] = temp;
			}
		}

		netState.ClearState();
		temperature = temperature * cooling;
		iter++;


	}
	initialSolutionsOrders[thr] = bestOrder;
	cout << thr << " " << bestOrder.size() << endl;
	for (int i =0 ; i<bestOrder.size() ; i++){ cout << initialSolutionsOrders[thr][i] << ','; };
	cout << endl;
	//cout << iter << " <--------------------" << endl;
};

void Heur_RSSA::updateThreadyBestValues(int maxSliceIndex, double bestObjective, vector<int> bestOrder){
	mtx_threadyFFkSP.lock();
	threadyCurrentMaxSliceIndex = maxSliceIndex ;
	threadyCurrentBestObjVal = bestObjective;
	threadyBestOrder = bestOrder;
	cout << "new best objetive : " << bestObjective << ", maxSliceIndex : " << maxSliceIndex << endl;
	mtx_threadyFFkSP.unlock();
};

void Heur_RSSA::threadyWorker(int thr){
		int currentMaxSliceIndex = maxSliceIndex;
	double currentBestObjVal = bestObjective;
	NetworkState netState = NetworkState(netScen);
	NetworkState bestNetState = NetworkState(netScen);
	order = vector<int>(netScen->traffic->demands);
	for (int j = 0; j < netScen->traffic->demands; j++)
		order[j] = j;
	permuteDemands(order);
	maxSliceIndex = netScen->flexGrid->slices + 1;
	threadyCurrentMaxSliceIndex = maxSliceIndex;
	bestObjective = BIGNUMBER;
	threadyCurrentBestObjVal = bestObjective;
	bestNetState = netState;
	netState.ClearState();
	int bestIter = -IloInfinity;
	double initTemperature = bestObjective * algPar->heurSAtemperatureCoeff;
	double temperature = initTemperature;
	double cooling = algPar->heurSAcooling;
	vector<int> bestOrder = order;
	long iter = 0;
	int d1, d2;
	//cout << initTemperature << " <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"<<endl;
	//system("pause");
	while (iter < algPar->heurSAiterLimit) {
			// generate neighbouring solution by swapping two demands
			int orderCount = netScen->traffic->demands;
			
			threadyFFkSP(d1, d2, orderCount, order, netState);
			int tempA = netState.GetMaxSliceIndex();
			int delta = tempA - currentMaxSliceIndex;
			double tempObjVal = GetObjValue(netState.GetMaxSliceIndex());
			double deltaObjVal = tempObjVal - threadyCurrentBestObjVal;

			//if(delta <= 0) {
			if (deltaObjVal <= 0) {
				currentMaxSliceIndex = tempA;
				currentBestObjVal = tempObjVal;

				//if(currentMaxSliceIndex < maxSliceIndex) {
				if (currentBestObjVal < bestObjective) {
					maxSliceIndex = currentMaxSliceIndex;
					bestObjective = currentBestObjVal;

					bestNetState.ClearState();
					bestNetState = netState;
					bestIter = iter;
					bestOrder = order;
					//DisplayResults(netState, iter);
					//runTime = (double)(clock() - tStart) / CLOCKS_PER_SEC;
					//DisplayResults(netState, iter, runTime);

					//if (divide = 1) {
					updateThreadyBestValues(maxSliceIndex, bestObjective, bestOrder);
					//}
					
					cout << "    - Max slice index: " << maxSliceIndex << " in thread" << thr << endl << flush;
				}
		
		}
		else {

			//if((rand() % 1000) / 1000.0 < exp((double) -delta/temperature)) {
			if ((rand() % 1000) / 1000.0 < exp((double)-deltaObjVal / temperature)) {
				currentMaxSliceIndex = tempA;
				currentBestObjVal = tempObjVal;
			}
			else {
				permuteDemands(order);
			}
		}

		netState.ClearState();
		temperature = temperature * cooling;
		iter++;


	}
	initialSolutionsOrders[thr] = bestOrder;
	cout << thr << " " << bestOrder.size() << endl;
	for (int i =0 ; i<bestOrder.size() ; i++){ cout << initialSolutionsOrders[thr][i] << ','; };
	cout << endl;
	//cout << iter << " <--------------------" << endl;
};

void Heur_RSSA::pureHeuristic(int thr){
	int currentMaxSliceIndex = maxSliceIndex;
	double currentBestObjVal = bestObjective;
	NetworkState netState = NetworkState(netScen);
	NetworkState bestNetState = NetworkState(netScen);
	order = vector<int>(netScen->traffic->demands);
	for (int j = 0; j < netScen->traffic->demands; j++)
		order[j] = j;
	permuteDemands(order);
	maxSliceIndex = netScen->flexGrid->slices + 1;
	threadyCurrentMaxSliceIndex = maxSliceIndex;
	bestObjective = BIGNUMBER;
	threadyCurrentBestObjVal = bestObjective;
	bestNetState = netState;
	netState.ClearState();
	int bestIter = -IloInfinity;
	double initTemperature = bestObjective * algPar->heurSAtemperatureCoeff;
	double temperature = initTemperature;
	double cooling = algPar->heurSAcooling;
	vector<int> bestOrder = order;
	long iter = 0;
	int d1, d2;
	//cout << initTemperature << " <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"<<endl;
	//system("pause");
	while (iter < algPar->heurSAiterLimit) {
		// generate neighbouring solution by swapping two demands
		int orderCount = netScen->traffic->demands;
		threadyFFkSP(d1, d2, orderCount, order, netState);
		int tempA = netState.GetMaxSliceIndex();
		int delta = tempA - currentMaxSliceIndex;
		double tempObjVal = GetObjValue(netState.GetMaxSliceIndex());
		double deltaObjVal = tempObjVal - threadyCurrentBestObjVal;
		//if(delta <= 0) {
		if (deltaObjVal <= 0) {
			currentMaxSliceIndex = tempA;
			currentBestObjVal = tempObjVal;
			//if(currentMaxSliceIndex < maxSliceIndex) {
			if (currentBestObjVal < bestObjective) {
				maxSliceIndex = currentMaxSliceIndex;
				bestObjective = currentBestObjVal;

				bestNetState.ClearState();
				bestNetState = netState;
				bestIter = iter;
				bestOrder = order;
				//DisplayResults(netState, iter);
				//runTime = (double)(clock() - tStart) / CLOCKS_PER_SEC;
				//DisplayResults(netState, iter, runTime);
				//if (divide = 1) {
				updateThreadyBestValues(maxSliceIndex, bestObjective, bestOrder);
				//}
				cout << "    - Max slice index: " << maxSliceIndex << " in thread" << thr << endl << flush;
			}
		}
		else {
			permuteDemands(order);
		}
		netState.ClearState();
		iter++;
	}
	initialSolutionsOrders[thr] = bestOrder;
	cout << thr << " " << bestOrder.size() << endl;
	for (int i =0 ; i<bestOrder.size() ; i++){ cout << initialSolutionsOrders[thr][i] << ','; };
	cout << endl;
	//cout << iter << " <--------------------" << endl;
};

void Heur_RSSA::neightbourGen2(int iteration) {
	//d1 = (int)(rand() % orderCount);
	//d2 = (int)(rand() % (orderCount - 1));
	//if (d1 <= d2) d2++;
	//temp = order[d1]; order[d1] = order[d2]; order[d2] = temp;
	int orderCount = order.size();
	int d1 = iteration % orderCount;
	int loop_number = iteration / orderCount;
	int d2 = d1 + loop_number;
}