#include "StdAfx.h"
#include "LP_CG_RSSA.h"
#include "Routing.h"

LP_CG_RSSA::LP_CG_RSSA(SimulationData &simData, FileWriter &fileWriter) {

	this->simData = simData;
	this->fileWriter = fileWriter;

	model = IloModel(env);
	cplex = IloCplex(env);

	cplex.setParam(IloCplex::Threads, std::thread::hardware_concurrency());

	lps = vector<vector<Lightpath>>();
	lpsMC = vector<vector<LightpathNew>>();
}

LP_CG_RSSA::~LP_CG_RSSA(void) {

	cplex.end();
	model.end();
	env.end();
}

void LP_CG_RSSA::UploadInitialSolution(vector<Lightpath> &initLps) {

	lps = vector<vector<Lightpath>>(simData.traffic.demands);
	for (int d = 0; d < simData.traffic.demands; d++)
		lps[d].push_back(initLps[d]);
}

void LP_CG_RSSA::UploadInitialSolution(vector<LightpathNew> &initLpsMC) {

	lpsMC = vector<vector<LightpathNew>>(simData.traffic.demands);
	for (int d = 0; d < simData.traffic.demands; d++)
		lpsMC[d].push_back(initLpsMC[d]);
}


void LP_CG_RSSA::Preprocessing() {

	cout << "    - Preprocessing" << endl;
	clock_t tStart = clock();

	// generate all possible candidate lightpaths
	/*lps = vector<vector<Lightpath>>(simData.traffic.demands);
	for (int d = 0; d < simData.traffic.demands; d++) {
		int numOfRts = simData.traffic.demandSet[d].candidateRoutes.size();
		for (int k = 0; k < numOfRts; k++) {
			Route rt = simData.traffic.demandSet[d].candidateRoutes[k];
			int n = rt.getNumOfSlices();
			for (int c = 0; c < simData.flexGrid.chs[n].size(); c++) {
				Channel ch = simData.flexGrid.chs[n][c];
				lps[d].push_back(Lightpath(d, rt, 1, ch));
			}
		}
	}*/
	// generate all possible candidate lightpathNew
	lpsMC = vector<vector<LightpathNew>>(simData.traffic.demands);
	vector<int> zeroModeVector(simData.net.links);
	for (int e = 0; e < simData.net.links; e++) {
			zeroModeVector[e] = 0;
	};
	for (int d = 0; d < simData.traffic.demands; d++) {
		int numOfRts = simData.traffic.demandSet[d].candidateRoutes.size();
		for (int k = 0; k < numOfRts; k++) {
			Route rt = simData.traffic.demandSet[d].candidateRoutes[k];
			int n = rt.getNumOfSlices();
			for (int c = 0; c < simData.flexGrid.chs[n].size(); c++) {
				Channel ch = simData.flexGrid.chs[n][c];
				lpsMC[d].push_back(LightpathNew(d, rt, zeroModeVector, ch));
			}
		}
	};



	double runTime = (double)(clock() - tStart)/CLOCKS_PER_SEC;
	cout << "      - Preprocessing time: " << runTime << endl;
}

void LP_CG_RSSA::InitializeModel() { // to w pierwszej kolejnoœci nale¿y przerobiæ wzoruj¹c siê na MIP (model 2 w opracowaniu)

	int d, e, l, m, s;

	cout << "  - Initializing LP-CG model" << endl;
	Preprocessing();
	// --- variables declaration
	xdl = IloArray<IloNumVarArray>(env, simData.traffic.demands);
	for (d = 0; d < simData.traffic.demands; d++)
		xdl[d] = IloNumVarArray(env);
	xs = IloNumVarArray(env, simData.flexGrid.slices, 0, 1, ILOFLOAT);
	xems = IloArray<IloArray<IloNumVarArray>>(env, simData.net.links);
	for (e = 0; e < simData.net.links; e++) {
		xems[e] = IloArray<IloNumVarArray>(env, simData.net.modes);
		for (m = 0; m < simData.net.modes; m++) {
			xems[e][m] = IloNumVarArray(env, simData.flexGrid.slices, 0, 1, ILOFLOAT);
		}
	}
	// --- objective definition
	objective = IloAdd(model, IloMinimize(env));
	objective.setExpr(IloSum(xs));

	// --- constraints declaration
	constrLightpathAssignment = IloRangeArray(env, simData.traffic.demands);
	for (d = 0; d < simData.traffic.demands; d++) {
		constrLightpathAssignment[d] = IloRange(env, 0.0, 0.0);
		model.add(constrLightpathAssignment[d]);
	}

	constrSliceCapacity = IloArray<IloArray<IloRangeArray>>(env, simData.net.links);
	for (e = 0; e < simData.net.links; e++) {
		constrSliceCapacity[e] = IloArray<IloRangeArray>(env, simData.net.modes);
		for (m = 0; m < simData.net.modes; m++) {
			constrSliceCapacity[e][m] = IloRangeArray(env, simData.flexGrid.slices);
			for (s = 0; s < simData.flexGrid.slices; s++) {
				//constrSliceCapacity[e][m][s] = IloRange(env, 0.0, 0.0);
				constrSliceCapacity[e][m][s] = IloRange(env, 0.0, 1.0);
				model.add(constrSliceCapacity[e][m][s]);
			}
		}
	}

	constrSliceOccupancy = IloArray<IloArray<IloRangeArray>>(env, simData.net.links);
	for (e = 0; e < simData.net.links; e++) {
		constrSliceOccupancy[e] = IloArray<IloRangeArray>(env, simData.net.modes);
		for (m = 0; m < simData.net.modes; m++) {
			constrSliceOccupancy[e][m] = IloRangeArray(env, simData.flexGrid.slices);
			for (s = 0; s < simData.flexGrid.slices; s++) {
				constrSliceOccupancy[e][m][s] = IloRange(env, 0.0, 1.0);
				model.add(constrSliceOccupancy[e][m][s]);
			}
		}
	}

	// --- constraints definition
	for (d = 0; d < simData.traffic.demands; d++)
		constrLightpathAssignment[d].setExpr(IloExpr(env, -1.0));

	for (e = 0; e < simData.net.links; e++)
		for (m = 0; m < simData.net.modes; m++)
			for (s = 0; s < simData.flexGrid.slices; s++)
				constrSliceCapacity[e][m][s].setExpr(xems[e][m][s]);

	for (d = 0; d < simData.traffic.demands; d++)
		for (l = 0; l < lps[d].size(); l++)
			AddColumnToModel(lps[d][l]);

	for (e = 0; e < simData.net.links; e++)
		for (m = 0; m < simData.net.modes; m++)
			for (s = 0; s < simData.flexGrid.slices; s++)
				constrSliceOccupancy[e][m][s].setExpr(xs[s] - xems[e][m][s]);

	for (s = 0; s < simData.flexGrid.slices - 1; s++)
		model.add(xs[s] >= xs[s + 1]);

	for (e = 0; e < simData.net.links; e++)
		for (m = 0; m < simData.net.modes; m++)
			for (s = 0; s < simData.flexGrid.slices - 1; s++)
				model.add(xems[e][m][s] >= xems[e][m][s + 1]);

	//--- declaration of dual variables
	priceLp = IloNumArray(env, simData.traffic.demands);
	priceSlice = IloArray<IloArray<IloNumArray>>(env, simData.net.links);
	for (e = 0; e < simData.net.links; e++) {
		priceSlice[e] = IloArray<IloNumArray>(env, simData.net.modes);
		for (m = 0; m < simData.net.modes; m++)
			priceSlice[e][m] = IloNumArray(env, simData.flexGrid.slices);
	}

	//--- pass model to cplex
	cplex.extract(model);
}

void LP_CG_RSSA::InitializeModel_MC() { // to w pierwszej kolejnoœci nale¿y przerobiæ wzoruj¹c siê na MIP (model 2 w opracowaniu)

	int d, e, l, m, s;

	cout << "  - Initializing LP-CG model" << endl;
	Preprocessing();
	// --- variables declaration
	xdl = IloArray<IloNumVarArray>(env, simData.traffic.demands); // x_dl w modelu 1 i 2
	for (d = 0; d < simData.traffic.demands; d++)
		xdl[d] = IloNumVarArray(env);
	xs = IloNumVarArray(env, simData.flexGrid.slices, 0, simData.net.modes, ILOFLOAT); // y_s w modelu 2
	//xems = IloArray<IloArray<IloNumVarArray>>(env, simData.net.links); //y_ems w modelu 1
	xes = IloArray<IloNumVarArray>(env, simData.net.links); // y_es w modelu 2
	for (e = 0; e < simData.net.links; e++) {
		xes[e] = IloNumVarArray(env, simData.flexGrid.slices, 0, simData.net.modes, ILOFLOAT); // inicjalizacja wartoœci macierzy y_es
	}
	// --- objective definition
	objective = IloAdd(model, IloMinimize(env)); // okreœlenie charakteru optymalizacji jako minimalizacja ,w nag³ówku model jest zadeklarowany jako IloModel
	objective.setExpr(IloSum(xs)); // okreœlenie funkcji kryterium jako sumy sum{s in S} y_s
	// czy zadzia³a coœ takiego: model.add(IloMinimize(env, IloSum(xs)));

	// --- constraints declaration
	constrLightpathAssignment = IloRangeArray(env, simData.traffic.demands); // pytanie 1 ??? - powo³anie obiektu który bêdzie uto¿samiany ze œcie¿k¹ optyczn¹, niezmienione w stosunku do modelu 1
	for (d = 0; d < simData.traffic.demands; d++) {
		constrLightpathAssignment[d] = IloRange(env, 0.0, 0.0); // dla ka¿dego d-tego zapotrzebowania przypisujemy przedzia³ mo¿liwych wartoœci <0.0 ; 0.0> ??? w sumie to wygl¹da jak wektor samych 0, który pewnie zmienia wartoœæ na 1 je¿eli wybierana jest jakaœ œcie¿ka optyczna
		model.add(constrLightpathAssignment[d]); // dodanie ograniczenia do modelu
	}
	constrSliceCapacity_MC = IloArray<IloRangeArray>(env, simData.net.links); // ograniczenie zwi¹zane z pojemnoœci¹ s-tego slica na e-tym ³¹czu
	for (e = 0; e < simData.net.links; e++) {
		constrSliceCapacity_MC[e] = IloRangeArray(env, simData.flexGrid.slices);
		for (s = 0; s < simData.flexGrid.slices; s++) {
			//constrSliceCapacity_MC[e][s] = IloRange(env, 0.0, 1.0); //przypisanie przedzia³u mo¿liwych wartoœci <0.0 ; 1.0> do komórki macierzy odpowiadaj¹cej s-temu plastrowi czêstotliwoœci na e-tym ³¹czu sieci
			constrSliceCapacity_MC[e][s] = IloRange(env, 0.0, simData.net.modes); // na s-tym slice'ie i e-tym laczu mozna zalokowac tyle sciezek ile jest modow
			model.add(constrSliceCapacity_MC[e][s]); // dodanie ograniczenia do modelu
		}
	}
	constrSliceOccupancy_MC = IloArray<IloRangeArray>(env, simData.net.links); // ograniczenie zwi¹zane z zajêtoœci¹ s-tego slica na e-tym ³¹czu
	for (e = 0; e < simData.net.links; e++) {
		constrSliceOccupancy_MC[e] = IloRangeArray(env, simData.flexGrid.slices);
		for (s = 0; s < simData.flexGrid.slices; s++) {
			//constrSliceOccupancy_MC[e][s] = IloRange(env, 0.0, 1.0); //przypisanie przedzia³u mo¿liwych wartoœci <0.0 ; 1.0> do komórki macierzy odpowiadaj¹cej s-temu plastrowi czêstotliwoœci na e-tym ³¹czu sieci
			constrSliceOccupancy_MC[e][s] = IloRange(env, 0.0, 1.0); // na s-tym slice'ie i e-tym laczu mozna zalokowac tyle sciezek ile jest modow
			model.add(constrSliceOccupancy_MC[e][s]); // dodanie kolejnego ograniczenia do modelu
		}
	}

	// --- constraints definition
	for (d = 0; d < simData.traffic.demands; d++) // pytanie 2 ??? - komentarz na ten temat
		constrLightpathAssignment[d].setExpr(IloExpr(env, -1.0)); // dziêki temu sprawiamy ¿e wybrana lightpath przyjmie wartoœæ 1 - 0<= constrlightpathassignment -1 <=0

	for (e = 0; e < simData.net.links; e++)
		for (s = 0; s < simData.flexGrid.slices; s++)
			constrSliceCapacity_MC[e][s].setExpr(simData.net.modes * xes[e][s]);

	for (d = 0; d < simData.traffic.demands; d++)
		for (l = 0; l < lpsMC[d].size(); l++)
			AddColumnToModel(lpsMC[d][l]);

	for (e = 0; e < simData.net.links; e++)
		for (s = 0; s < simData.flexGrid.slices; s++)
			constrSliceOccupancy_MC[e][s].setExpr(xs[s] - xes[e][s]);

	/*for (s = 0; s < simData.flexGrid.slices - 1; s++)
		model.add(xs[s] >= xs[s + 1]);
		*/
	/*for (e = 0; e < simData.net.links; e++)
		for (s = 0; s < simData.flexGrid.slices - 1; s++)
			model.add(xes[e][s] >= xes[e][s + 1]); 
			*/
	//--- declaration of dual variables
	priceLp = IloNumArray(env, simData.traffic.demands);
	priceSlice_MC = IloArray<IloNumArray>(env, simData.net.links);
	for (e = 0; e < simData.net.links; e++) {
		priceSlice_MC[e] = IloNumArray(env, simData.flexGrid.slices);
	}

	//--- pass model to cplex
	cplex.extract(model);
}


IloNumVar LP_CG_RSSA::AddColumnToModel(Lightpath lp) {

	int d = lp.d;
	int mode = lp.mode;

	IloNumColumn col = objective(0);
	col += constrLightpathAssignment[d](1.0);
	for (int h = 0; h < lp.rt.hops; h++) {
		int e = lp.rt.linkVector[h];
		for (int s = lp.ch.beginSlice; s <= lp.ch.endSlice; s++)
			col += constrSliceCapacity[e][mode][s](-1.0);
	}

	IloNumVar var = IloNumVar(col, 0, 1, ILOFLOAT);
	xdl[d].add(var);

	col.end();

	return var;
}

IloNumVar LP_CG_RSSA::AddColumnToModel(LightpathNew lp) {
	int d = lp.d;
	IloNumColumn col = objective(0);
	col += constrLightpathAssignment[d](1.0);
	for (int h = 0; h < lp.rt.hops; h++) {
		int e = lp.rt.linkVector[h];
		for (int s = lp.ch.beginSlice; s <= lp.ch.endSlice; s++)
			col += constrSliceCapacity_MC[e][s](-1.0);
	}
	IloNumVar var = IloNumVar(col, 0, 1, ILOFLOAT);
	xdl[d].add(var);
	col.end();
	return var;
}


void LP_CG_RSSA::ExtractDuals() {

	cplex.getDuals(priceLp, constrLightpathAssignment);
	for (int e = 0; e < simData.net.links; e++)
		for (int m = 0; m < simData.net.modes; m++)
			cplex.getDuals(priceSlice[e][m], constrSliceCapacity[e][m]);
}

void LP_CG_RSSA::ExtractDuals_MC() {

	cplex.getDuals(priceLp, constrLightpathAssignment);
	for (int e = 0; e < simData.net.links; e++)
		cplex.getDuals(priceSlice_MC[e], constrSliceCapacity_MC[e]);
}

void LP_CG_RSSA::GenerateColumns() {

	//FindNewColumnsAmongCandidates();
	FindNewColumnsAmongCandidates_MC();
	//FindNewColumnsWithSP();
}

void LP_CG_RSSA::FindNewColumnsAmongCandidates() {

	newLps = vector<Lightpath>();

	for (int d = 0; d < simData.traffic.demands; d++) {

		Route bestRt;
		Channel bestCh;
		int bestMode;
		IloNum bestReducedCost = -IloInfinity;

		double reducedCost = priceLp[d];
		if (reducedCost <= 0)
			continue;

		int numOfRts = simData.traffic.demandSet[d].candidateRoutes.size();
		for (int k = 0; k < numOfRts; k++) {
			Route rt = simData.traffic.demandSet[d].candidateRoutes[k];
			int n = rt.getNumOfSlices();
			for (int m = 0; m < simData.net.modes; m++) {
				for (int c = 0; c < simData.flexGrid.chs[n].size(); c++) {
					Channel ch = simData.flexGrid.chs[n][c];

					reducedCost = priceLp[d];

					//if (reducedCost <= 0)
					//	continue;

					for (int h = 0; h < rt.hops; h++) {
						int e = rt.linkVector[h];
						for (int s = ch.beginSlice; s <= ch.endSlice; s++) {
							reducedCost = reducedCost - priceSlice[e][m][s];
						}
					}

					if (reducedCost > RC_EPS) {
						//if(reducedCost > 0) {

						if (bestReducedCost < reducedCost) {
							bestReducedCost = reducedCost;
							bestRt = rt;
							bestMode = m;
							bestCh = ch;
						}
					}
				}
			}
		}

		if (bestReducedCost > RC_EPS) {
			//if(bestReducedCost > 0) {
			//cout << "  - Positive reduced cost = " << bestReducedCost << " for demand " << d << endl;
			//cout << "    - a lightpath added" << endl;
			//bestLp->display();
			newLps.push_back(Lightpath(d, bestRt, bestMode, bestCh));
		}
	}
}


void LP_CG_RSSA::FindNewColumnsAmongCandidates_MC() {

	newLpsMC = vector<LightpathNew>();
	for (int d = 0; d < simData.traffic.demands; d++) {
		Route bestRt;
		Channel bestCh;
		vector<int> bestModeVector;

		IloNum bestReducedCost = -IloInfinity;

		double reducedCost = priceLp[d];
		if (reducedCost <= 0)
			continue;

		int numOfRts = simData.traffic.demandSet[d].candidateRoutes.size();
		for (int k = 0; k < numOfRts; k++) {
			Route rt = simData.traffic.demandSet[d].candidateRoutes[k];
			int n = rt.getNumOfSlices();
			//for (int m = 0; m < simData.net.modes; m++) {
				for (int c = 0; c < simData.flexGrid.chs[n].size(); c++) {
					Channel ch = simData.flexGrid.chs[n][c];

					reducedCost = priceLp[d];

					//if (reducedCost <= 0)
					//	continue;
					vector<int> modeVector (simData.net.links);
					for (int h = 0; h < rt.hops; h++) {
						int e = rt.linkVector[h];
						modeVector[e] = 0;
						for (int s = ch.beginSlice; s <= ch.endSlice; s++) {
							reducedCost = reducedCost - priceSlice_MC[e][s];
						}
					}
					if (reducedCost > RC_EPS) {
						//if(reducedCost > 0) {

						if (bestReducedCost < reducedCost) {
							bestReducedCost = reducedCost;
							bestRt = rt;
							bestModeVector = modeVector;
							bestCh = ch;
						}
					}
				}
			//}
		}

		if (bestReducedCost > RC_EPS) {
			//if(bestReducedCost > 0) {
			//cout << "  - Positive reduced cost = " << bestReducedCost << " for demand " << d << endl;
			//cout << "    - a lightpath added" << endl;
			//bestLp->display();
			newLpsMC.push_back(LightpathNew(d, bestRt, bestModeVector, bestCh));
		}
	}
}



void LP_CG_RSSA::FindNewColumnsWithSP() {

	cout << "Warning: the shortest path-based column generation algorithm is not implemented yet!" << endl << endl;
	exit(0);

	newLps = vector<Lightpath>();

	Routing routing = Routing(simData);
	double limit = IloInfinity;

	for (int d = 0; d < simData.traffic.demands; d++) {

		//int src = simData.traffic.demandSet[d].src;
		//int dest = simData.traffic.demandSet[d].dest;

		//Channel bestChannel;
		//vector<int> bestRoute = vector<int>();
		//IloNum bestReducedCost = -IloInfinity;
		//int n = netScen->qotModel->ConvertBitrateToSlots(netScen->traffic->unicastDemandSet[d].hdmin);
	//		for (c = 0; c < netScen->flexGrid->chs[n].size(); c++) {
	//			ch = netScen->flexGrid->chs[n][c];
	//			if (ch != NULL) {

	//				vector<double> linkCost(netScen->links, 0);
	//				for (e = 0; e < netScen->links; e++)
	//					for (s = ch->beginSlice; s <= ch->endSlice; s++)
	//						linkCost[e] += priceSlice[e][s];

	//				vector<int> sp = routing.ShortestPath(src, dest, linkCost, limit);
	//				if (sp.size() > 0) {
	//					double pathCost = 0;
	//					for (e = 0; e < sp.size(); e++)
	//						pathCost += linkCost[sp[e]];

	//					//IloNum reducedCost = priceLp[d] + min((double) netScen->traffic->unicastDemandSet[d].hdmax,netScen->qotModel->ConvertSlotsToBitrate(ch->size));		// newVersion
	//					IloNum reducedCost = priceLp[d]; // only hdmax
	//					reducedCost = reducedCost - pathCost;

	//					//if(simPar->CliqueCutsUsed) {
	//					//	for(int f = 0; f < cliqueCutsLps->cliqueVector.size(); f++) {
	//					//		reducedCost = reducedCost - priceCutClique[f];
	//					//	}
	//					//}

	//					if (reducedCost > RC_EPS) {
	//						if (bestReducedCost < reducedCost) {
	//							bestReducedCost = reducedCost;
	//							bestChannel = ch;
	//							bestRoute = sp;
	//						}
	//					}
	//				}
	//			}
	//		}

	//	//cout << "    Best reduced cost = " << bestReducedCost << endl;
	//	if (bestReducedCost > RC_EPS) {
	//		//cout << "      best channel: "; bestChannel->display();
	//		//cout << "      best route: ";
	//		//for(e = 0; e < bestRoute.size(); e++)
	//		//	cout << bestRoute[e] << " ";
	//		//cout << endl;

	//		//for(e = 0; e < bestRoute.size(); e++)
	//		//	cout << bestRoute[e] << " ";
	//		//cout << endl;

	//		rt = RoutePtr(new Route(src, dest, bestRoute, netScen));

	//		lp = LightpathPtr(new Lightpath(d, rt, bestChannel));
	//		lps.push_back(lp);
	//	}
	}
}

int LP_CG_RSSA::AddGeneratedColumns() {
	if (newLps.size() > 0)
		for (int l = 0; l < newLps.size(); l++)
			AddColumnToModel(newLps[l]);
	return newLps.size();
}

int LP_CG_RSSA::AddGeneratedColumns_MC() {
	if (newLpsMC.size() > 0)
		for (int l = 0; l < newLpsMC.size(); l++)
			AddColumnToModel(newLpsMC[l]);
	return newLpsMC.size();
}

void LP_CG_RSSA::SolveLP() {

	try {
		cplex.solve();
	}
	catch (IloException& e) {
		cerr << "Concert exception caught: " << e << endl;
		system("pause");
	}
	catch (...) {
		cout << "Warning: unexpected error when running cplex" << endl;
		system("pause");
	}

	if (cplex.getStatus() == IloAlgorithm::Infeasible)
		cout << "Warning: infeasible solution" << endl;
	else
		cout << endl << "    - Optimal value: " << cplex.getObjValue() << endl << endl;
}

void LP_CG_RSSA::SolveLPwithCG() {

	int cols = 0, allCols = 0, iterColGen = 0;

	cout << "  - Solving LP problem with CG" << endl;

	cplex.setOut(env.getNullStream());
	cplex.setParam(IloCplex::ParallelMode, 1);
	cplex.setParam(IloCplex::RootAlg, IloCplex::AutoAlg);

	double runTime1 = 0, runTime2 = 0, runTime3 = 0;
	clock_t tStart = clock();

	//--- LP with CG solving loop	
	while (true) {

		iterColGen++;
		cout << "      Iteration " << iterColGen;

		try {
			cplex.solve();
		}
		catch (IloException& e) {
			cerr << "Concert exception caught: " << e << endl;
			system("pause");
		}
		catch (...) {
			cout << "Warning: unexpected error when running cplex" << endl;
			system("pause");
		}

		cout << " (" << cplex.getStatus() << ")";

		if (cplex.getStatus() == IloAlgorithm::Infeasible) {
			cout << "Warning: infeasible solution" << endl;
			exit(0);
		}

		cout << ": objective = " << cplex.getObjValue();

		ExtractDuals_MC();
		GenerateColumns();
		cols = AddGeneratedColumns_MC();

		if (cols == 0)		// no more columns found => the solution is optimal
			break;
		else {
			allCols += cols;
			cout << ", " << cols << " columns added (" << allCols << " added)";
			cout << endl;
		}
	}

	cout << endl;
	cout << "    - Optimal value: " << cplex.getObjValue() << endl;
	cout << "    - Iterations = " << iterColGen << " (" << allCols << " columns added)" << endl;
	double runTime = (double)(clock() - tStart) / CLOCKS_PER_SEC;
	cout << "    - Runtime = " << runTime << endl;
	cout << endl;
}

void LP_CG_RSSA::SolveLPasMIP() {
	// convert LP model to MIP model
	IloModel modelMIP(env);
	modelMIP.add(model);
	for (int d = 0; d < simData.traffic.demands; d++)
		modelMIP.add(IloConversion(env, xdl[d], ILOINT));
	modelMIP.add(IloConversion(env, xs, ILOINT));
	for (int e = 0; e < simData.net.links; e++)
		for (int m = 0; m < simData.net.modes; m++)
			modelMIP.add(IloConversion(env, xems[e][m], ILOINT));
	// solve MIP model
	IloCplex cplexMIP(env);
	cplexMIP.setParam(IloCplex::Threads, std::thread::hardware_concurrency());
	cplexMIP.extract(modelMIP);
	cplexMIP.solve();
	cout << endl;
	cout << "      - Status: " << cplexMIP.getStatus() << endl;
	int objective = -1;
	if (cplexMIP.getStatus() == IloAlgorithm::Optimal || cplexMIP.getStatus() == IloAlgorithm::Feasible) {
		objective = cplexMIP.getObjValue();
		cout << "      - MIP value: " << cplexMIP.getObjValue() << endl << endl;
	}
}

void LP_CG_RSSA::SolveLPasMIP_MC() {
	// convert LP model to MIP model
	IloModel modelMIP(env);
	modelMIP.add(model);
	for (int d = 0; d < simData.traffic.demands; d++)
		modelMIP.add(IloConversion(env, xdl[d], ILOINT));
	modelMIP.add(IloConversion(env, xs, ILOINT));
	for (int e = 0; e < simData.net.links; e++)
		modelMIP.add(IloConversion(env, xes[e], ILOINT));
	// solve MIP model
	IloCplex cplexMIP(env);
	cplexMIP.setParam(IloCplex::Threads, std::thread::hardware_concurrency());
	cplexMIP.extract(modelMIP);
	cplexMIP.solve();
	cout << endl;
	cout << "      - Status: " << cplexMIP.getStatus() << endl;
	int objective = -1;
	if (cplexMIP.getStatus() == IloAlgorithm::Optimal || cplexMIP.getStatus() == IloAlgorithm::Feasible) {
		objective = cplexMIP.getObjValue();
		cout << "      - MIP value: " << cplexMIP.getObjValue() << endl << endl;
	}
}