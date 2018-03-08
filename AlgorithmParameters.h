#pragma once

#include <thread>
#include <boost/thread.hpp>

class AlgorithmParameters {
public:
	int algOption;

	double MIPrunTimeLimit;
	double MIPmemoryLimit;
	double LBrunTimeLimit;

	unsigned int CPUnumOfthreads;

	double heurSAtemperatureCoeff;
	double heurSAcooling;
	long heurSAiterLimit;
	double heurSAtimeLimit;
	double heurSAtempLimit;

	AlgorithmParameters(void) {
		MIPrunTimeLimit = 1e+75;
		MIPmemoryLimit = 1e+75;
		LBrunTimeLimit = 1e+75;

		CPUnumOfthreads = std::thread::hardware_concurrency();

		heurSAtemperatureCoeff = 0.05;
		heurSAiterLimit = 2147483647;
		heurSAtimeLimit = 1e+75;
		heurSAtempLimit = 0.01;
	};

	~AlgorithmParameters(void) {};
};

