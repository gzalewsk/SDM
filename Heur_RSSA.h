#pragma once

#include "SimulationParameters.h"
#include "AlgorithmParameters.h"
#include "SupportingFunctions.h"
#include "NetworkScenario.h"
#include "FileWriter.h"
#include <thread>
#include <boost/thread.hpp>
#include "NetworkState.h"
#include <ilcplex/ilocplex.h>

ILOSTLBEGIN



class Heur_RSSA {
private:
	int maxSliceIndex;
	double bestObjective;
	boost::mutex mtx_cout;
public:
	Heur_RSSA() {};
	SimulationParameters *simPar;
	AlgorithmParameters *algPar;
	SupportingFunctions *supFun;
	NetworkScenario *netScen;
	FileWriter *fileWriter;
	Heur_RSSA(SimulationParameters *simPar, AlgorithmParameters *algPar, SupportingFunctions *supFun, NetworkScenario *netScen, FileWriter *fileWriter);
	~Heur_RSSA(void) {};

	void FFkSP();
	void SimAn_FFkSP();
	void Heur_RSSA::SimAn_FFkSP_gr();
	void Heur_RSSA::SimAn_FFkSP_gr2();
	int FFkSP(vector<int> &order, NetworkState &netState);
	
	void permuteDemands(vector<int> &order);
	vector<int> order;
	int temp;
	bool isChannelEmpty(RoutePtr &rt, ChannelPtr &ch, NetworkState &netState);
	void updateBestLp(RoutePtr &bestRt, ChannelPtr &bestCh, RoutePtr &rt, ChannelPtr &ch);
	void addLpToSolution(int d, RoutePtr &bestRt, ChannelPtr &bestCh, NetworkState &netState);
	void Heur_RSSA::updateNetworkState(RoutePtr &bestRt, int beginSlice, int endSlice, NetworkState &netState);

	int GetMaxSliceIndex();
	double bestObjective_public;
	double maxSliceIndex_public;
	double GetObjValue(int slices);
	void DisplayResults();
	void DisplayResults(NetworkState &netState);
	void DisplayResults(NetworkState &netState, int iter);
	void DisplayResults(NetworkState &netState, int iter, double time);
	void DisplayResults(string description);
	int myrandom (int i);
	int FFkSP_thread(vector<int> &order, NetworkState &netState, int threads);
	boost::mutex mtx_FFkSP;
	void Heur_RSSA::neightbourGen(  int &d1, int &d2, int orderCount);
	void Heur_RSSA::threadyFFkSP(int &d1, int &d2, int orderCount, vector<int> &order, NetworkState &netstate);
	void Heur_RSSA::threadyInit(int thread, int divide);
	vector<int> initialSolutionsOrders[20];
	
	void Heur_RSSA::updateThreadyBestValues(int maxSliceIndex, double bestObjective, vector<int> bestOrder);
	int threadyCurrentMaxSliceIndex;
	double threadyCurrentBestObjVal;
	vector<int> threadyBestOrder;
	boost::mutex mtx_threadyFFkSP;

	void Heur_RSSA::threadyWorker(int thr);
	void Heur_RSSA::pureHeuristic(int thr);
	void Heur_RSSA::neightbourGen2(int iteration);
};

class Heur_RSSA_gregor : public Heur_RSSA {
	private:
		//boost::mutex mtx_cout;
	public:
		Heur_RSSA_gregor() :  Heur_RSSA() {};
		Heur_RSSA_gregor(SimulationParameters *simPar, AlgorithmParameters *algPar, SupportingFunctions *supFun, NetworkScenario *netScen, FileWriter *fileWriter) : Heur_RSSA(simPar, algPar, supFun, netScen, fileWriter) {};
		//~Heur_RSSA_gregor(void) : ~Heur_RSSA(void) {};
		void wypisz_wymaluj(string);
		boost::mutex mtx;

};