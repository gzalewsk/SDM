#pragma once

#include "StdAfx.h"

#include "SimulationData.h"
#include "SupportingFunctions.h"
#include "FileWriter.h"
#include "CommonDataSpace.h"

#include <thread>
#include <boost/thread.hpp>
//#include <boost/chrono.hpp>

class Simulation {
	CommonDataSpace *comDataSpace;

public:
	SimulationData simData;
	SupportingFunctions supFun;
	FileWriter fileWriter;

	Simulation(void) {};
	Simulation(SimulationData &simData, SupportingFunctions &supFun, FileWriter &fileWriter, CommonDataSpace *comDataSpace);
	~Simulation(void) {};
};

