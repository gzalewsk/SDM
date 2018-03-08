#include "stdafx.h"
#include "iostream"

#include "Network.h"
#include "Traffic.h"
#include "SimulationData.h"
#include "CommonDataSpace.h"

#include "RSSA.h"
#include "Heuristics.h"
#include "DynamicSimulation.h"
#include "MIP_RSSA.h"
#include "LP_CG_RSSA.h"

void ReadInputFiles(int argc, char* argv[], SimulationParameters &simPar, AlgorithmParameters &algPar, SupportingFunctions &supFun);
void InitializeOutputFiles(FileWriter &fileWriter);

void GenerateSimulationScenarios(int argc, char* argv[], vector<SimulationData> &simDataVect) {
	
	SimulationParameters simPar = SimulationParameters();
	AlgorithmParameters algPar = AlgorithmParameters();
	SupportingFunctions supFun = SupportingFunctions();
	ReadInputFiles(argc, argv, simPar, algPar, supFun);
	simPar.hasDirectedLinks = true;
	//simPar.hasDirectedLinks = false;
	NetworkScenario netScen = NetworkScenario(simPar, supFun);
	netScen.InitializeNetwork();
	Network net = Network(netScen.nodes, netScen.links, netScen.modes, netScen.linkLength, netScen.nodeLinks, netScen.neigh, netScen.dijkstraNeigh, netScen.linkIndex);
	netScen.InitializePrecomputedRoutes();
	
	for (int m = 0; m < 1; m++) {
		//for (int m = 0; m < 2; m++) {
		//if (m == 0)
		//net.setModes(19);
		//else
		//net.setModes(7);
		//net.setModes(1);

		//for (int a = 0; a < 2; a++) {
		for (int a = 0; a < 1; a++) {
			if (a == 0)
				algPar.algOption = 0;	// mirek's algorithm
			else if (a == 1)
				algPar.algOption = 1;	// grzesiek's algorithm

			for (int t = 0; t < 1; t++) {
				//for (int t = 0; t < 7; t++) {
				//if (t == 0)
				//algPar.CPUnumOfthreads = 38;
				//algPar.CPUnumOfthreads = 32;
				//algPar.CPUnumOfthreads = 4;
				//else	
				//algPar.CPUnumOfthreads = 1;
				//if (t < 6)
				//	algPar.CPUnumOfthreads = pow(2, t);
				//else
				//	algPar.CPUnumOfthreads = 38;

				for (int i = 0; i < simPar.demandSet.size(); i++) {
					string setID = simPar.demandSet[i];
					Traffic traffic = Traffic(simPar, supFun, netScen);
					traffic.InitializeTraffic(setID, netScen.netDir);
					simDataVect.push_back(SimulationData(algPar, simPar, net, netScen.flexGrid, traffic));
				}
			}
		}
	}

}

void RunOptimizationFramework(vector<SimulationData> &simDataVect) {

	SupportingFunctions supFun = SupportingFunctions();
	FileWriter fileWriter = FileWriter();
	//fileWriter.SetStatus(false);
	//InitializeOutputFiles(fileWriter);

	for (int i = 0; i < simDataVect.size(); i++) {

		SimulationData simData = simDataVect[i];
		CommonDataSpace *comDataSpace = new CommonDataSpace();

		//string setID = simPar.demandSet[i];
		//time_t t = time(0);
		//struct tm * now = localtime(&t);
		//string name = "<" + setID + ">";
		//cout << endl;
		//cout << "======================================================================" << endl;
		//cout << "  Starting simulation for demand set " << name << endl;
		//cout << "  Time: ";
		////cout << (now->tm_year + 1900) << '-' << (now->tm_mon + 1) << '-' <<  now->tm_mday << endl;
		//cout << (now->tm_hour) << 'h' << (now->tm_min) << 'm' << now->tm_sec << 's' << endl;
		//cout << "======================================================================" << endl;

		cout << "- Number of CPU threads = " << simData.algPar.CPUnumOfthreads << endl;
		
		RSSA optProblem = RSSA();

		// --- generate initial solution
		DynamicSimulation simulation = DynamicSimulation(comDataSpace, 1);
		// klasa LightpathNew tyczy siê przypadku z konwersj¹ modu (MC). W przypadku kiedy rozpatrywany jest przypadek bez konwersji (no_MC) nale¿y u¿ywaæ klasy Lightpath. 
		
		//vector<LightpathNew> initLps = simAn.GenerateInitialSolutionNew(optProblem, simData);
		//vector<Lightpath> initLps = simAn.GenerateInitialSolution(optProblem, simData);
		/*for (int l=0; l < initLps.size(); l++){
			initLps[l].display();
		};
		*/
		NetworkState netState = NetworkState(simData);
		//simAn.SolveProblemDynamic(netState, optProblem, simData);
		//simulation.SolveProblemDynamicV2(netState, optProblem, simData);
		simulation.SolveProblemDynamicV3(netState, optProblem, simData);

		// --- update global upper bound and the number of slices in the flexgrid
		int S = comDataSpace->bestNetState.GetMaxSliceIndex();
		// S= 60 i ju¿ bêdzie dzia³a³o to tu jest problem
		comDataSpace->initialObjective = comDataSpace->bestObjectiveGlobal;
		cout << endl << "--- Global upper bound = " << S << endl << endl;
		if (S < simData.traffic.maxSliceVolume)
			simData.flexGrid.UpdateNumberOfSlices(simData.traffic.maxSliceVolume);
		else
			simData.flexGrid.UpdateNumberOfSlices(S);

		


		//system("pause");

		// --- run column generation algorithm and solve MIP assuming the generated columns
		
		//cout << "run column generation algorithm and solve MIP assuming the generated columns" << endl; 
		//LP_CG_RSSA LPmodel = LP_CG_RSSA(simData, fileWriter);
		//LPmodel.UploadInitialSolution(initLps);
		//LPmodel.InitializeModel_MC();
		
		
		//int beginSlice = 0;
		//while (true) {
			//LPmodel.SolveLPwithCG();
			//double objValue = LPmodel.cplex.getObjValue();
			//if (objValue - floor(objValue) > 0) {
				//for (int s = beginSlice; s < ceil(objValue); s++)
					//LPmodel.model.add(LPmodel.xs[s] == 1);
//				beginSlice = ceil(objValue);
			//}
			//else
				//break;
		//}
		
		// LPmodel.SolveLPasMIP();
		//LPmodel.SolveLPasMIP_MC(); // najpierw sprawdzic z tym czy siê nie wysypuje

		//uncoment preprocessing code lines
		//cout << "simple LP solving:" <<endl;
		//LPmodel.SolveLP();
		
		/*
		cout << "test LP_CG" <<endl;
		//system("pause");
		LPmodel.SolveLPwithCG();
		*/


		//exit(0);

		// --- solve RSSA as MIP problem for all possible candidate lightpaths
		//cout << "solve RSSA as MIP problem for all possible candidate lightpaths" << endl;
		//MIP_RSSA MIPmodel = MIP_RSSA(simData, fileWriter);
		
		//MIP model uwzglêdniaj¹cy konwersjê modu z dopiskiem _MC
		//MIPmodel.InitializeModel();
		
		//MIPmodel.InitializeModel_MC();
		
		//MIPmodel.Run();
		//cout << "MIP model done" << endl;
		//system("pause"); 
		//exit(0);
		
		// --- find lower bound
		cout << "find lower bound" << endl;
		double LBoptGap = 1.0;
		//simData.algPar.LBrunTimeLimit = 60;
		
		simData.algPar.LBrunTimeLimit = 10;
		
		//comDataSpace->globalLowerBound = MIPmodel.FindLowerBound(0, LBoptGap);	// find lower bound without cuts applied
		//comDataSpace->globalLowerBound = MIPmodel.FindLowerBound_MC(0, LBoptGap);	// find lower bound without cuts applied
		//comDataSpace->globalLowerBound = MIPmodel.InitializeModel_ELB();	// find lower bound without cuts applied
		
		
		//comDataSpace->globalLowerBound = MIPmodel.FindLowerBound(1, LBoptGap);	// find lower bound with clique cuts applied
		//cout << endl << "--- Global lower bound = " << comDataSpace->globalLowerBound << " (optimality gap = " << LBoptGap << ")" << endl << endl;

		//double LBoptGapWithCuts;
		//int LBwithCuts = MIPmodel.FindLowerBound(1, LBoptGapWithCuts);	// find lower bound with clique cuts applied
		//cout << endl << "--- Global lower bound with cuts = " << LBwithCuts << " (optimality gap = " << LBoptGapWithCuts << ")" << endl << endl;
		//exit(0);
		
		//simData.flexGrid.UpdateNumberOfSlices(S * 2);
		//simData.flexGrid.UpdateNumberOfSlices(S);
		
		

		//comDataSpace->algStartTimeGlobal = clock();
		
		// --- run paraller simulated annealing algorithm
		cout << "--- start parSimAn ---" << endl;
		//system("pause");
		optProblem = RSSA();
		ParallelSimulatedAnnealing parSimAn = ParallelSimulatedAnnealing();
		parSimAn.Run(optProblem, simData, comDataSpace);
		cout << "--- stop parSimAn ---" << endl;
		//system("pause");
		double optGapPerc = (double)supFun.Round(((comDataSpace->bestObjectiveGlobal - comDataSpace->globalLowerBound) / comDataSpace->bestObjectiveGlobal) * 10000) / 100.0;
		double optGap = (double)supFun.Round(((comDataSpace->bestObjectiveGlobal - comDataSpace->globalLowerBound) / comDataSpace->bestObjectiveGlobal) * 10000) / 10000.0;
		cout << endl << "--- Best objective value = " << comDataSpace->bestObjectiveGlobal << " (found after " << comDataSpace->bestSolutionTime << " sec., optimality gap = " << optGapPerc << "%)" << endl;
		double runTime = (double)(clock() - comDataSpace->algStartTimeGlobal) / CLOCKS_PER_SEC;

		
		system("pause");
		exit(0);
		// --- save result to output file
		if (simData.algPar.algOption == 0)
			fileWriter.Add(0, "mk");
		else 
			fileWriter.Add(0, "gz");
		fileWriter.Add(0, "\t");
		fileWriter.Add(0, i + 1);
		fileWriter.Add(0, "\t");
		fileWriter.Add(0, simData.traffic.demands);
		fileWriter.Add(0, "\t");
		if (simData.simPar.hasDirectedLinks == true)
			fileWriter.Add(0, "dir");
		else
			fileWriter.Add(0, "undir");
		fileWriter.Add(0, "\t");
		fileWriter.Add(0, simData.net.modes);
		fileWriter.Add(0, "\t");
		fileWriter.Add(0, (int)simData.traffic.demandSet[0].candidateRoutes.size());
		fileWriter.Add(0, "\t");
		fileWriter.Add(0, (int)simData.algPar.CPUnumOfthreads);
		fileWriter.Add(0, "\t");
		fileWriter.Add(0, comDataSpace->globalLowerBound);
		//fileWriter.Add(0, "\t");
		//fileWriter.Add(0, LBwithCuts);
		fileWriter.Add(0, "\t");		
		fileWriter.Add(0, LBoptGap);
		//fileWriter.Add(0, "\t");
		//fileWriter.Add(0, LBoptGapWithCuts);
		fileWriter.Add(0, "\t");
		fileWriter.Add(0, comDataSpace->initialObjective);
		fileWriter.Add(0, "\t");
		fileWriter.Add(0, comDataSpace->bestObjectiveGlobal);
		fileWriter.Add(0, "\t");
		//fileWriter.Add(0, optGapPerc);
		fileWriter.Add(0, (double) optGap);
		fileWriter.Add(0, "\t");
		fileWriter.Add(0, comDataSpace->overallNumOfIterations);
		fileWriter.Add(0, "\t");
		fileWriter.Add(0, comDataSpace->bestSolutionTime);
		fileWriter.Add(0, "\t");
		fileWriter.Add(0, runTime);
		fileWriter.Add(0, "\n");
		delete comDataSpace;
	}
}

int main(int argc, char* argv[])
{
	clock_t tStart = clock();
	
	vector<SimulationData> simDataVect;
	GenerateSimulationScenarios(argc, argv, simDataVect);
	
	
	RunOptimizationFramework(simDataVect);
	double runTime = (double)(clock() - tStart) / CLOCKS_PER_SEC;
	cout << "--- Overall runtime = " << runTime << endl << endl;
	getchar();
	return 0;
}

void ReadInputFiles(int argc, char* argv[], SimulationParameters &simPar, AlgorithmParameters &algPar, SupportingFunctions &supFun) {

	if (argc >= 2) {

		string line;
		string fileName;
		vector<string> tokens;

		string confFileName = argv[1];

		cout << "- Loading configuration from file " << confFileName << endl;

		ostringstream id1; id1 << ".\\" << confFileName; fileName = id1.str();
		fstream confFile(fileName.c_str(), ios::in);

		if (confFile) {

			getline(confFile, line, '\n'); supFun.Tokenize(line, tokens, " ");
			simPar.dataDir = tokens[0].c_str();
			cout << "  - data directory: " << simPar.dataDir << endl;
			tokens.clear();

			getline(confFile, line, '\n'); supFun.Tokenize(line, tokens, " ");
			simPar.scenFileName = tokens[0].c_str();
			cout << "  - scenario file: " << simPar.scenFileName << endl;
			tokens.clear();

			getline(confFile, line, '\n'); supFun.Tokenize(line, tokens, " ");
			simPar.algFileName = tokens[0].c_str();
			cout << "  - algorithm file: " << simPar.algFileName << endl;
			tokens.clear();

			ostringstream id2; id2 << ".\\" << simPar.algFileName; fileName = id2.str();
			fstream algFile(fileName.c_str(), ios::in);

			if (algFile) {
				while (!algFile.eof()) {

					getline(algFile, line, '\n'); supFun.Tokenize(line, tokens, "\t");
					string text = tokens[0].c_str();
					if (text == "" || text.at(0) == '#') { tokens.clear(); continue; }

					if (text == "algOption") {
						if (tokens.size() < 2) { cout << "Warning: algoritm option defined incorrectly" << endl; getchar(); exit(0); }
						string algorithm = tokens[1].c_str();
						cout << "  - algorithm option: " << algorithm << endl;
						if (algorithm == "SimAn") {
							simPar.algorithm = -1;
							algPar.heurSAiterLimit = atoi(tokens[2].c_str());
							algPar.heurSAcooling = atof(tokens[3].c_str());
							algPar.heurSAtemperatureCoeff = atof(tokens[4].c_str());
							algPar.heurSAtimeLimit = atoi(tokens[5].c_str());
						}
						else if (algorithm == "MIP") {
							simPar.algorithm = 1;
							algPar.MIPmemoryLimit = atoi(tokens[2].c_str());
							cout << "    - memory limit [MB]: " << algPar.MIPmemoryLimit << endl;
							algPar.MIPrunTimeLimit = atoi(tokens[3].c_str());;
							cout << "    - MIP run-time limit [s]: " << algPar.MIPrunTimeLimit << endl;
						}
						else {
							cout << "Warning: unsupported algorithm type (" << algorithm << ")" << endl;
							getchar();
							exit(0);
						}
					}
					else if (text == "CPUthreads") {
						algPar.CPUnumOfthreads = atoi(tokens[1].c_str());
						cout << "  - Max. number of CPU threads: " << algPar.CPUnumOfthreads << endl;
					}
					tokens.clear();
				}
			}
			else {
				cout << endl << "Warning: file " << fileName.c_str() << " does not exist. The algorithm is not loaded" << endl << endl;
				getchar();
				exit(0);
			}
		}
	}

	cout << "- Loading network scenario from file " << simPar.scenFileName << endl;

	string line, line2;
	string fileName;
	vector<string> tokens, tokens2;

	ostringstream id1; id1 << ".\\" << simPar.scenFileName; fileName = id1.str();
	fstream scenFile(fileName.c_str(), ios::in);

	if (scenFile) {

		while (!scenFile.eof()) {

			getline(scenFile, line, '\n'); supFun.Tokenize(line, tokens, "\t");
			string text = tokens[0].c_str();
			if (text == "" || text.at(0) == '#') { tokens.clear(); continue; }

			if (text == "NetworkID") {
				if (tokens.size() < 2) { cout << "Warning: network defined incorrectly" << endl; getchar(); exit(0); }
				string net = tokens[1].c_str();
				if (net == "SIMPLE") simPar.network = SIMPLE;
				else if (net == "INT9") simPar.network = INT9;
				else if (net == "POL12") simPar.network = POL12;
				else if (net == "DT12") simPar.network = DT12;
				else if (net == "DT14") simPar.network = DT14;
				else if (net == "NSF14_GangFeng") simPar.network = NSF14_GangFeng;
				else if (net == "TEL14") simPar.network = TEL14;
				else if (net == "NSF15") simPar.network = NSF15;
				else if (net == "EURO16") simPar.network = EURO16;
				else if (net == "BT22") simPar.network = BT22;
				else if (net == "TEL21N35E") simPar.network = TEL21N35E;
				else if (net == "UBN24") simPar.network = UBN24;
				else if (net == "US26") simPar.network = US26;
				else if (net == "EURO28") simPar.network = EURO28;
				else if (net == "TEL30") simPar.network = TEL30;
				else if (net == "TI44") simPar.network = TI44;
				else if (net == "EURO49") simPar.network = EURO49;
				else { cout << "Warning: wrong network name." << endl; getchar(); exit(0); }
				cout << "  - network: " << net << endl;
			}
			else if (text == "RoutingPaths") {
				if (tokens.size() < 3) { cout << "Warning: candidate routing paths not defined" << endl; getchar(); exit(0); }
				simPar.kSPlimit = atoi(tokens[1].c_str());
				string text1 = tokens[2].c_str();
				if (text1 == "true")
					simPar.areRoutesUnique = true;
				else
					simPar.areRoutesUnique = false;
			}
			else if (text == "SDMmodes") {
				if (tokens.size() < 2) { cout << "Warning: number of modes not defined" << endl; getchar(); exit(0); }
				simPar.modes = atoi(tokens[1].c_str());
				cout << "  - SDM modes: " << simPar.modes << endl;
			}
			else if (text == "Crosstalk") {
				if (tokens.size() < 2) { cout << "Warning: crosstalk not specified" << endl; getchar(); exit(0); }
				string text1 = tokens[1].c_str();
				if (text1 == "yes") {
					simPar.isXTconsidered = true;
					string XTfileName = tokens[2].c_str();
					vector<string> tokens1;
					ostringstream id2; id2 << ".\\" << XTfileName; fileName = id2.str();
					fstream XTfile(fileName.c_str(), ios::in);

					if (XTfile) {
						while (!XTfile.eof()) {

							getline(XTfile, line2, '\n'); supFun.Tokenize(line2, tokens2, "\t");
							string text2 = tokens2[0].c_str();
							if (text2 == "" || text2.at(0) == '#') { tokens2.clear(); continue; }

							if (text2 == "XTestimation") {
								if (tokens2.size() < 2) { cout << "Warning: crosstalk estimation approach not specified" << endl; getchar(); exit(0); }
								string text3 = tokens2[1].c_str();
								cout << "    - XT estimation model: " << text3 << endl;
								if (text3 == "best-effort") {
									simPar.XTtype = BESTEFFORT;
								}
								else if (text3 == "precise") {
									simPar.XTtype = PRECISE;
								}
								else { cout << "Warning: crosstalk estimation approach not supported" << endl; getchar(); exit(0); }
							}
							else if (text2 == "hXT")
							{
								if (tokens2.size() < 2) { cout << "Warning: h parameter not defined" << endl; getchar(); exit(0); }
								simPar.hXTkm = atof(tokens2[1].c_str()) * 1000;
								cout << "    - XT parameter h: " << simPar.hXTkm << endl;
							}
							else if (text2 == "XTlimit") {
								if (tokens2.size() < 2) { cout << "Warning: XT limit not defined" << endl; getchar(); exit(0); }
								simPar.XTlimit = atof(tokens2[1].c_str());
								cout << "    - XT limit [dB]: " << simPar.XTlimit << endl;
							}

							tokens2.clear();
						}
					}
					else {
						cout << endl << "Warning: file " << fileName.c_str() << " does not exist. Crosstalk data not loaded." << endl << endl;
						getchar();
						exit(0);
					}
				}
				else
					simPar.isXTconsidered = false;
			}
			else if (text == "TrafficDemands") {
				if (tokens.size() < 3) { cout << "Warning: traffic scenario not defined" << endl; getchar(); exit(0); }
				string uniDemFileName = tokens[1].c_str();
				simPar.numOfTrafficSets = atoi(tokens[2].c_str());

				vector<string> tokens1;
				ostringstream id2; id2 << ".\\" << uniDemFileName; fileName = id2.str();
				fstream demFile1(fileName.c_str(), ios::in);

				if (demFile1) {
					simPar.demandSet = vector<string>();
					for (int i = 0; i < simPar.numOfTrafficSets; i++) {
						getline(demFile1, line, '\n'); supFun.Tokenize(line, tokens1, "\t");
						string name = tokens1[0].c_str();
						simPar.demandSet.push_back(name);
						tokens1.clear();
					}
				}
				else {
					cout << endl << "Warning: file " << fileName.c_str() << " does not exist. Traffic sets not loaded" << endl << endl;	// ###
					getchar();
					exit(0);
				}
			}

			tokens.clear();
		}
	}
	else {
		cout << endl << "Warning: file " << fileName.c_str() << " does not exist. The network is not loaded" << endl << endl;	// ###
		getchar();
		exit(0);
	}
}

void InitializeOutputFiles(FileWriter &fileWriter) {

	//ostringstream name;
	//name << netScen->netName << "_k" << simPar.kSPlimit;
	//if (simPar.algorithm == -1) {
	//	name << "_SA";
	//	name << "_" << algPar.heurSAcooling;
	//	name << "_" << algPar.heurSAtemperatureCoeff;
	//}
	//else if (simPar.algorithm == -2)
	//	name << "_FF-kSP";

	//else if (simPar.algorithm == 1) name << "_MIP";
	//fileWriter.InitFiles(1, name.str());

	//ostringstream descr;
	//descr << "slice width\t" << simPar.sliceWidth;
	//if (simPar.hasDirectedLinks)
	//	descr << "\ndirected links\ttrue";
	//else
	//	descr << "\ndirected links\tfalse";
	//descr << endl << endl;

	//fileWriter.Add(0, descr.str());
	
	fileWriter.InitFiles(1, "out");
}

