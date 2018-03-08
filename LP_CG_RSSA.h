#pragma once

#include "SimulationData.h"
#include "FileWriter.h"
#include "Lightpath.h"

#include <ilcplex/ilocplex.h>
ILOSTLBEGIN

class LP_CG_RSSA {
public:
	IloEnv env;
	IloModel model;
	IloCplex cplex;


	IloArray<IloNumVarArray> xdl;
	IloNumVarArray xs;
	IloArray<IloArray<IloNumVarArray>> xems;
	IloArray<IloNumVarArray> xes;
	IloObjective objective;

	IloRangeArray constrLightpathAssignment;
	IloArray<IloArray<IloRangeArray>> constrSliceCapacity;
	IloArray<IloArray<IloRangeArray>> constrSliceOccupancy;
	IloArray<IloRangeArray> constrSliceCapacity_MC; // wersja dla MC
	IloArray<IloRangeArray> constrSliceOccupancy_MC; // wersja dla MC

	IloNumArray priceLp;
	IloArray<IloArray<IloNumArray>> priceSlice;
	IloArray<IloNumArray> priceSlice_MC;

	vector<vector<Lightpath>> lps;
	vector<vector<LightpathNew>> lpsMC;
	vector<Lightpath> newLps;
	vector<LightpathNew> newLpsMC;

	SimulationData simData;
	FileWriter fileWriter;

	LP_CG_RSSA() {};
	LP_CG_RSSA(SimulationData &simData, FileWriter &fileWriter);
	~LP_CG_RSSA(void);

	void UploadInitialSolution(vector<Lightpath> &initLps);
	void UploadInitialSolution(vector<LightpathNew> &initLpsMC);
	
	void Preprocessing();
	void InitializeModel();
	void InitializeModel_MC();
	IloNumVar AddColumnToModel(Lightpath lp);
	IloNumVar AddColumnToModel(LightpathNew lp);
	
	void ExtractDuals();
	void ExtractDuals_MC();
	void GenerateColumns();
	void FindNewColumnsAmongCandidates();
	void FindNewColumnsAmongCandidates_MC();
	void FindNewColumnsWithSP();
	int AddGeneratedColumns();
	int AddGeneratedColumns_MC();

	void SolveLP();
	void SolveLPwithCG();
	void SolveLPasMIP();
	void SolveLPasMIP_MC();
};

