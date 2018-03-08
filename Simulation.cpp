#include "StdAfx.h"
#include "Simulation.h"

#include "MIP_RSSA.h"
#include "Heuristics.h"

Simulation::Simulation(SimulationData &simData, SupportingFunctions &supFun, FileWriter &fileWriter, CommonDataSpace *comDataSpace) {

	this->simData = simData;
	this->supFun = supFun;
	this->fileWriter = fileWriter;
	this->comDataSpace = comDataSpace;
}
