#pragma once

#include "SimulationData.h"
#include "FileWriter.h"

#include <ilcplex/ilocplex.h>
ILOSTLBEGIN

class MIP_RSSA {
public:
	IloEnv env;
	IloModel model;
	IloCplex cplex;
	IloArray<IloNumVarArray> xdp;
	IloArray<IloArray<IloNumVarArray>> xdpm;
	IloArray<IloArray<IloArray<IloNumVarArray>>> xdpmc;
	IloArray<IloArray<IloArray<IloNumVarArray>>> xdpem;
	IloNumVarArray xs;
	IloArray<IloArray<IloNumVarArray>> xems;
	IloArray<IloNumVarArray> xes;
	IloNumVar zSpec;
	IloObjective objective;
	IloArray<IloArray<IloNumVarArray>> xdpc;
	IloNum bestMIPObj;
	IloNum bestSpec;
	IloArray<IloArray<IloArray<IloNumVarArray>>> xdpemLB;

	double LB;
	double lb;

	SimulationData simData;
	FileWriter fileWriter;

	MIP_RSSA() {};
	MIP_RSSA(SimulationData &simData, FileWriter &fileWriter);
	~MIP_RSSA(void);

	void Preprocessing();
	void InitializeModel();
	void InitializeModel_MC();
	void InitializeModel_ELB();
	void SetCplexParameters();
	void SetMIPParameters();
	void Run();
	double FindLowerBound(int algOption, double &LBoptGap);
	double FindLowerBound_MC(int algOption, double &LBoptGap);

	void DisplayResults(string header);
	void DisplayMIPresults();
	void SaveResults();
};

