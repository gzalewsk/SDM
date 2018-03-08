#pragma once

#include "CommonDataSpace.h"
#include "SimulationData.h"
#include "NetworkState.h"
#include "OptimizationProblem.h"
#include "RSSA.h"
#include "Lightpath.h"

#include <random>
#include <thread>
#include <boost/thread.hpp>

class DynamicSimulation{
private:
	CommonDataSpace *comDataSpace;
	default_random_engine randGen;
	int mainThreadId;
	boost::mutex mtx;
public:
	DynamicSimulation(CommonDataSpace *comDataSpace);
	DynamicSimulation(CommonDataSpace *comDataSpace, int threadId);
	~DynamicSimulation(void) {};
	
	void SolveProblemDynamic(NetworkState &netState, RSSA &optProblem, SimulationData &simData);
	void SolveProblemDynamicV2(NetworkState &netState, RSSA &optProblem, SimulationData &simData);
	void SolveProblemDynamicV3(NetworkState &netState, RSSA &optProblem, SimulationData &simData);
	void lightpathSetup(int d, NetworkState &netState, RSSA &optProblem, SimulationData &simData);
	void lightpathTearDown(int d, LightpathNew &lightpath, NetworkState &netState, RSSA &optProblem, SimulationData &simData);
	int ConvertBitrateToSlices(double bitrate, double pathLength);
	int generate_exp_v1(int min, int rate, mt19937 &gen);
	int generate_exp_v2(int exp_dist_mean);
};