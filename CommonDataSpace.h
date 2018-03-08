#pragma once

#include "StdAfx.h"
#include "Definitions.h"
#include "NetworkState.h"

#include <thread>
#include <boost/thread.hpp>

class CommonDataSpace {
public:
	double globalLowerBound;
	double initialObjective;
	double bestObjectiveGlobal;
	vector<int> bestOrderGlobal;
	double bestSolutionTime;
	long overallNumOfIterations;
	clock_t algStartTimeGlobal;

	NetworkState bestNetState;

	boost::mutex mtx;

	CommonDataSpace(void) {
		bestObjectiveGlobal = BIGNUMBER;
		globalLowerBound = 0;
		overallNumOfIterations = 0;
	};
	~CommonDataSpace() {};

	void Display() {
	
		cout << "UB = " << bestObjectiveGlobal << ", LB = " << globalLowerBound << endl;
	};
};
