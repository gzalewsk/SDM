#pragma once

#include "OptimizationProblem.h"
#include "SimulationData.h"
#include "NetworkState.h"

class RSSA : public OptimizationProblem {
private:
	//int maxSliceIndex;
	//double bestObjective;

	double GetObjValue(int slices, int unservedDemands, SimulationData &simData);

public:
	RSSA();
	~RSSA(void) {};

	double EvaluateSolution(vector<int> &order, NetworkState &netState, SimulationData &simData);
	double EvaluateSolutionMK(vector<int> &order, NetworkState &netState, SimulationData &simData);
	double EvaluateSolutionGZ(vector<int> &order, NetworkState &netState, SimulationData &simData);
	double EvaluateSolutionGZNew(vector<int> &order, NetworkState &netState, SimulationData &simData);
	double EvaluateSolutionGZNewV2(vector<int> &order, NetworkState &netState, SimulationData &simData);
	void SetupSingleOrder(int d, NetworkState &netState, SimulationData &simData);
	bool SetupSingleOrderFS(int d, NetworkState &netState, SimulationData &simData);
	void TearDownSingleOrder(int d, NetworkState &netState, SimulationData &simData);
	
	bool isChannelEmpty(Route &rt, Channel &ch, int mode, NetworkState &netState);
	bool isXTacceptable(Route &rt, Channel &ch, int mode, NetworkState &netState, SimulationData &simData);
	void updateBestLp(Route &bestRt, Channel &bestCh, int &bestMode, Route &rt, Channel &ch, int mode);
	bool isBeginSliceEmpty(Route &rt, Channel &ch, int mode, NetworkState &netState);
	int isBeginSliceEmpty(Route &rt, int beginSlice, int mode, NetworkState &netState, int channel_width);
	int isBeginSliceEmpty(Route &rt, int beginSlice, vector<int> mode, NetworkState &netState, int channel_width);
	int isBeginSliceEmpty(Route &rt, int beginSlice, NetworkState &netState, int channel_width, int modes, vector<int> modeVector);
	
	void updateBestLp(Route &bestRt, int &bestBeginSlice, int &bestMode, Route &rt, int &beginSlice, int mode, int channel_width);
	void updateBestLp(Route &bestRt, int &bestBeginSlice, vector<int> &bestModeVector, Route &rt, int &beginSlice, vector<int> modeVector, int n);
	void updateBestLp(Route &bestRt, int &bestBeginSlice, Route &rt, int &beginSlice, int channel_width);
	void addLpToSolution(int d, Route &bestRt, int &bestBeginSlice, int bestMode, NetworkState &netState, SimulationData &simData);
	void addLpToSolution(int d, Route &bestRt, int &bestBeginSlice, vector<int> bestModeVector, NetworkState &netState, SimulationData &simData);
	void addLpToSolution(int d, Route &bestRt, Channel &bestCh, int bestMode, NetworkState &netState);
	void removeLpFromSolution(int d, Route &bestRt, int &bestBeginSlice, int bestMode, NetworkState &netState, SimulationData &simData);
	void removeLpFromSolution(int d, Route &bestRt, int &bestBeginSlice, vector<int> bestModeVector, NetworkState &netState, SimulationData &simData);
	
	//int GetMaxSliceIndex();

	//void test() { cout << "RSSA class" << endl; };

	//int FFkSP_RSA_par(vector<int> &order, NetworkState &netState);
	//void FFSpecAlloc(NetworkState &netState, Route &rt, Channel &foundCh, int &foundMode, bool &isLpFound);
};
