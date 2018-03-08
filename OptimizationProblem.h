#pragma once

#include "NetworkState.h"

class OptimizationProblem {

public:
	OptimizationProblem() {};
	virtual ~OptimizationProblem() {};

	virtual double EvaluateSolution(vector<int> &order, NetworkState &netState) { return -1; };

	virtual void test() { cout << "virtual class" << endl; };
};