#pragma once

#include "CommonDataSpace.h"
#include "SimulationData.h"
#include "NetworkState.h"
#include "OptimizationProblem.h"
#include "RSSA.h"
#include "Lightpath.h"

#include <random>
#include <thread>
#include <boost\thread.hpp>

class SimulatedAnnealing {
private:
	CommonDataSpace *comDataSpace;

	default_random_engine randGen;
	int mainThreadId;
	boost::mutex mtx;

public:
	SimulatedAnnealing(CommonDataSpace *comDataSpace);
	SimulatedAnnealing(CommonDataSpace *comDataSpace, int threadId);
	~SimulatedAnnealing(void) {};

	vector<Lightpath> GenerateInitialSolution(RSSA &optProblem, SimulationData &simData);
	vector<LightpathNew> GenerateInitialSolutionNew(RSSA &optProblem, SimulationData &simData);
	//void SolveProblem(OptimizationProblem &optProblem, SimulationData &simData);
	void SolveProblem(RSSA &optProblem, SimulationData &simData);
	void SolveProblemGr2(RSSA &optProblem, SimulationData &simData);
	//void SolveProblemGr2(RSSA optProblem, SimulationData simData);

	void permuteDemands(vector<int> &order);
	
	void DisplayResults(NetworkState &netState, double objective, int iter, double time, int threadId, double optGapPerc);

	void addLpToSolution(int d, Route &bestRt, int &bestBeginSlice, int bestMode, NetworkState &netState, SimulationData &simData);
	void addLpToSolution(int d, Route &bestRt, int &bestBeginSlice, vector<int> bestModeVector, NetworkState &netState, SimulationData &simData);
	void addLpToSolution(int d, Route &bestRt, Channel &bestCh, int bestMode, NetworkState &netState);
	
};

class ParallelSimulatedAnnealing {
public:
	ParallelSimulatedAnnealing() {};
	~ParallelSimulatedAnnealing() {};

	void Run(RSSA &optProblem, SimulationData &simData, CommonDataSpace *comDataSpace) {
		//void Run(OptimizationProblem &optProblem, SimulationData &simData, CommonDataSpace *comDataSpace) {
		vector<thread> th;
		unsigned numOfThreads = simData.algPar.CPUnumOfthreads /* >> 1*/;

		for (int t = 0; t < numOfThreads; t++)
		{
			th.push_back(thread(&ParallelSimulatedAnnealing::RunThread, this, t, std::ref(optProblem), std::ref(simData), comDataSpace));
			//th.push_back(thread(&ParallelSimulatedAnnealing::RunThread, this, t, optProblem, std::ref(simData), comDataSpace));
			//th.push_back(thread(&ParallelSimulatedAnnealing::RunThread, this, t, optProblem, simData, comDataSpace));
			//cout << "wid = " << th[t].get_id() << endl;
		}

		//vector<RSSA> rssa = vector<RSSA>(numOfThreads);
		//vector<SimulationData> simDataVect = vector<SimulationData>(numOfThreads);
		//for (int t = 0; t < numOfThreads; t++)
		//{
		//	rssa[t] = optProblem;
		//	simDataVect[t] = simData;
		//}
		//for (int t = 0; t < numOfThreads; t++) 
		//{
		//	//th.push_back(thread(&ParallelSimulatedAnnealing::RunThread, this, t, rssa[t], simDataVect[t], comDataSpace));
		//	th.push_back(thread(&ParallelSimulatedAnnealing::RunThread, this, t, std::ref(rssa[t]), std::ref(simDataVect[t]), comDataSpace));
		//}

		for (auto &t : th)
			t.join();
	}

	void RunThread(int threadId, RSSA &optProblem, SimulationData &simData, CommonDataSpace *comDataSpace) {
	//void RunThread(int threadId, RSSA optProblem, SimulationData &simData, CommonDataSpace *comDataSpace) {

		SimulatedAnnealing simAn = SimulatedAnnealing(comDataSpace, threadId);
		if (simData.algPar.algOption == 0) {
			simAn.SolveProblem(optProblem, simData);
		}
		else if (simData.algPar.algOption == 1)
			simAn.SolveProblemGr2(optProblem, simData);
		else
			exit(0);

		//RSSA *RSSAproblem = dynamic_cast<RSSA*>(&optProblem);
		//simAn.SolveProblem(*RSSAproblem, simData);
	}
};