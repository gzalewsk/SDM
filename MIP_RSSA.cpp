#include "StdAfx.h"
#include "MIP_RSSA.h"
#include "SupportingFunctions.h"

ILOMIPINFOCALLBACK1(StoreBestHeurCallback, MIP_RSSA *, LPmodel) {
	
	if(LPmodel->bestMIPObj > this->getIncumbentObjValue()) {
		LPmodel->bestMIPObj = this->getIncumbentObjValue();
		if(LPmodel->bestMIPObj < 1e+075) {
			cout << "UB improved: " << LPmodel->bestMIPObj << " (" << "max " << this->getIncumbentValue(LPmodel->zSpec) << " slices)" << endl;
		} else
			cout << "UB improved: " << LPmodel->bestMIPObj << endl;
		////LPmodel->tMIPbestUB = (double)(clock() - LPmodel->tMIPStart)/CLOCKS_PER_SEC;
		////cout << "UB improved: " << LPmodel->MIPub << " at time " << LPmodel->tMIPbestUB << endl;
	}
}

MIP_RSSA::MIP_RSSA(SimulationData &simData, FileWriter &fileWriter) {

	this->simData = simData;
	this->fileWriter = fileWriter;

	lb = 0;
	//bestMIPObj = BIGNUMBER;//IloInfinity;
	bestMIPObj = IloInfinity;
	bestSpec = IloInfinity;

	model = IloModel(env);
	cplex = IloCplex(env);

	int CPUnumOfthreads = std::thread::hardware_concurrency();
	cplex.setParam(IloCplex::Threads, CPUnumOfthreads);
}

MIP_RSSA::~MIP_RSSA(void) {

	cplex.end();
	model.end();
	env.end();
}

void MIP_RSSA::Preprocessing() {

	// to be used for additional pre-processing procedures
	// ...

	//cout << "    - Preprocessing" << endl;
	//clock_t tStart = clock();

	//double runTime = (double)(clock() - tStart)/CLOCKS_PER_SEC;
	//cout << "      - Preprocessing time: " << runTime << endl;
}

void MIP_RSSA::InitializeModel() {

	int c, d, e, i, k, l, m, n, p, q, s, t, v, w;
	Route rt;
	Channel ch;

	cout << "  - Initializing MIP model: ";

	Preprocessing();
	
	// --- variables declaration
	xdp = IloArray<IloNumVarArray>(env, simData.traffic.demands);
	xdpm = IloArray<IloArray<IloNumVarArray>>(env, simData.traffic.demands);	
	xdpmc = IloArray<IloArray<IloArray<IloNumVarArray>>>(env, simData.traffic.demands);
	for(d = 0; d < simData.traffic.demands; d++) {
		int numOfRts = simData.traffic.demandSet[d].candidateRoutes.size();
		xdp[d] = IloNumVarArray(env, numOfRts, 0, 1, ILOINT);
		xdpm[d] = IloArray<IloNumVarArray>(env, numOfRts);
		xdpmc[d] = IloArray<IloArray<IloNumVarArray>>(env, numOfRts);
		for(k = 0; k < numOfRts; k++) {
			rt = simData.traffic.demandSet[d].candidateRoutes[k];
			n = rt.getNumOfSlices();
			//cout << d << ", " << k << ": " << n << endl;
			xdpm[d][k] = IloNumVarArray(env, simData.net.modes, 0, 1, ILOINT);
			xdpmc[d][k] = IloArray<IloNumVarArray>(env, simData.net.modes);
			for (m = 0; m < simData.net.modes; m++) {
				xdpmc[d][k][m] = IloNumVarArray(env, simData.flexGrid.chs[n].size(), 0, 1, ILOINT);
			}
		}
	}

	xs = IloNumVarArray(env, simData.flexGrid.slices, 0, 1, ILOINT);
	xems = IloArray<IloArray<IloNumVarArray>>(env, simData.net.links);
	for (e = 0; e < simData.net.links; e++) {
		xems[e] = IloArray<IloNumVarArray>(env, simData.net.modes);
		for (m = 0; m < simData.net.modes; m++) {
			xems[e][m] = IloNumVarArray(env, simData.flexGrid.slices, 0, 1, ILOINT);
		}
	}

	zSpec = IloNumVar(env, 0, simData.flexGrid.slices, ILOINT);

	// --- constraints definition	
	for(d = 0; d < simData.traffic.demands; d++) {
		model.add(IloSum(xdp[d]) == 1);
		int numOfRts = simData.traffic.demandSet[d].candidateRoutes.size();
		for(k = 0; k < numOfRts; k++) {
			//rt = simData.traffic.demandSet[d].candidateRoutes[k];
			//n = rt.getNumOfSlices();
			model.add(IloSum(xdpm[d][k]) == xdp[d][k]);
			for (m = 0; m < simData.net.modes; m++) {
				model.add(IloSum(xdpmc[d][k][m]) == xdpm[d][k][m]); // liczba wykorzystanych kana³ów musi siê zgadzaæ
			}
		}
	}

	for(e = 0; e < simData.net.links; e++) {
		for (m = 0; m < simData.net.modes; m++) {
			for (s = 0; s < simData.flexGrid.slices; s++) {
				//cout << e << ", " << s << ": " << endl;
				IloExpr expr(env);
				for(d = 0; d < simData.traffic.demands; d++) {
					//cout << d << " ";
					int numOfRts = simData.traffic.demandSet[d].candidateRoutes.size();
					for(k = 0; k < numOfRts; k++) {
						//cout << k << endl;
						rt = simData.traffic.demandSet[d].candidateRoutes[k];
						if (rt.doesInclude(e)) {
							n = rt.getNumOfSlices();
							for (c = 0; c < simData.flexGrid.chs[n].size(); c++) {
								ch = simData.flexGrid.chs[n][c];
								if (ch.beginSlice <= s && s <= ch.endSlice && ch.size != 0) {
									//cout << "e = " << e << ", s = " << s << ": ";
									//ch->display();
									expr += xdpmc[d][k][m][c];
								}
							}
						}
					}
				}
				model.add(expr <= xems[e][m][s]);
				expr.end();
			}
		}
	}

	for(s = 0; s < simData.flexGrid.slices; s++)
		for (e = 0; e < simData.net.links; e++)
			for (m = 0; m < simData.net.modes; m++)
				model.add(xems[e][m][s] <= xs[s]);

	for (s = 0; s < simData.flexGrid.slices - 1; s++)
		model.add(xs[s] >= xs[s+1]);

	model.add(zSpec == IloSum(xs));

	// --- objective definition
	objective = IloAdd(model, IloMinimize(env));
	objective.setExpr(zSpec);
	
	//--- pass model to cplex
	cplex.extract(model);

	cout << endl;
	//system("pause");
}


void MIP_RSSA::InitializeModel_MC() {

	int c, d, e, i, k, l, m, n, p, q, s, t, v, w;
	Route rt;
	Channel ch;
	int M = simData.net.modes;
	int E = simData.net.links;
	cout << "  - Initializing MIP model: ";

	Preprocessing();
	
	// --- variables declaration
	xdp = IloArray<IloNumVarArray>(env, simData.traffic.demands); // przyjmuje wartosc 1 jezeli p-sciezka jest aktualnie wykorzystana w d-tym demandzie
	//xdpm = IloArray<IloArray<IloNumVarArray>>(env, simData.traffic.demands);	
	
	//xdpmc = IloArray<IloArray<IloArray<IloNumVarArray>>>(env, simData.traffic.demands);
	xdpc = IloArray<IloArray<IloNumVarArray>>(env, simData.traffic.demands);

	for(d = 0; d < simData.traffic.demands; d++) {
		int numOfRts = simData.traffic.demandSet[d].candidateRoutes.size();
		xdp[d] = IloNumVarArray(env, numOfRts, 0, 1, ILOINT);
		//xdpm[d] = IloArray<IloNumVarArray>(env, numOfRts);
		//xdpmc[d] = IloArray<IloArray<IloNumVarArray>>(env, numOfRts);
		xdpc[d] = IloArray<IloNumVarArray>(env, numOfRts);
		for(k = 0; k < numOfRts; k++) {
			rt = simData.traffic.demandSet[d].candidateRoutes[k];
			n = rt.getNumOfSlices();
			//cout << d << ", " << k << ": " << n << endl;
			//xdpm[d][k] = IloNumVarArray(env, simData.net.modes, 0, 1, ILOINT);
			//xdpmc[d][k] = IloArray<IloNumVarArray>(env, simData.net.modes);
			xdpc[d][k] = IloNumVarArray(env, simData.flexGrid.chs[n].size(), 0, 1, ILOINT);
			//for (m = 0; m < simData.net.modes; m++) {
				//xdpmc[d][k][m] = IloNumVarArray(env, simData.flexGrid.chs[n].size(), 0, 1, ILOINT);
			//}
			
		}
	}

	xs = IloNumVarArray(env, simData.flexGrid.slices, 0, 1, ILOINT);
	//xems = IloArray<IloArray<IloNumVarArray>>(env, simData.net.links);
	//for (e = 0; e < simData.net.links; e++) {
	//	xems[e] = IloArray<IloNumVarArray>(env, simData.net.modes);
	//	for (m = 0; m < simData.net.modes; m++) {
	//		xems[e][m] = IloNumVarArray(env, simData.flexGrid.slices, 0, 1, ILOINT);
	//	}
	//}
	xes = IloArray<IloNumVarArray>(env, simData.net.links); //wektor odpowiadaj¹cy y_es w modelu mat
	for (e = 0; e < simData.net.links; e++) {
		xes[e] = IloNumVarArray(env, simData.flexGrid.slices, 0, 1, ILOINT); // przypisanie dla ka¿dej krawêdzi wartoœci odpowiadaj¹cej liczbie mo¿liwych do alokacji slice'ów/plastrów czêstotliwoœci
	}

	zSpec = IloNumVar(env, 0, simData.flexGrid.slices, ILOINT);

	// --- constraints definition	
	for(d = 0; d < simData.traffic.demands; d++) {
		model.add(IloSum(xdp[d]) == 1); // odpowiada ograniczeniu sum(p in P(d)) x_dp = 1
		int numOfRts = simData.traffic.demandSet[d].candidateRoutes.size();
		for(k = 0; k < numOfRts; k++) {
			//rt = simData.traffic.demandSet[d].candidateRoutes[k];
			//n = rt.getNumOfSlices();
			model.add(IloSum(xdpc[d][k]) == xdp[d][k]); // czyli suma kana³ów wykorzsytywanych na danej sciezce (k) na zapotrzebowaniu (d) jest przypisywana zmiennej xdp. To odpowiada ograniczeniu sum(c in C(d,p))x_dpc = x_dp
		}
	}


	for(e = 0; e < simData.net.links; e++) { // dla ka¿dego e (krawedzi)
		//for (m = 0; m < simData.net.modes; m++) { 
			for (s = 0; s < simData.flexGrid.slices; s++) { // dla kazdego s (plastra czêstotliwoœci)
				//cout << e << ", " << s << ": " << endl;
				IloExpr expr(env);
				for(d = 0; d < simData.traffic.demands; d++) { // dla ka¿dego d (zapotrzebowania, demanda)
					//cout << d << " ";
					int numOfRts = simData.traffic.demandSet[d].candidateRoutes.size();
					for(k = 0; k < numOfRts; k++) {	// dla ka¿dej k (œcie¿ki routingu)
						//cout << k << endl;
						rt = simData.traffic.demandSet[d].candidateRoutes[k];
						if (rt.doesInclude(e)) {
							n = rt.getNumOfSlices();
							for (c = 0; c < simData.flexGrid.chs[n].size(); c++) { // dla ka¿dego c (kana³a czêstotliwoœci
								ch = simData.flexGrid.chs[n][c];
								if (ch.beginSlice <= s && s <= ch.endSlice && ch.size != 0) {
									//cout << "e = " << e << ", s = " << s << ": ";
									//ch->display();
									expr += xdpc[d][k][c]; // obliczamy sumê zajêtych c (kana³ów)
								}
							}
						}
					}
				}
				model.add(expr <= M * xes[e][s]); // dla ka¿dego e oraz s, suma zajêtych kana³ów musi byæ mniejsza ni¿ zmienna xes (jej wartoœci to liczba wszystkich slice'ow na danej krawêdzi) - w modelu y_es. M - to licznoœæ zbioru modów i przez to przemna¿am (|M|y_es)
				expr.end();
			}
		//}
	}

	int KtorePoprawne = 2; // tutaj pytanie, ktore sformulowanie lepiej odpowiada wyrazeniu sum(e in E) y_es <= |E|ys, dla kazdego (s in S)
	switch (KtorePoprawne){
	case 1:
		for(s = 0; s < simData.flexGrid.slices; s++)
			for (e = 0; e < simData.net.links; e++)
				//for (m = 0; m < simData.net.modes; m++)
					//model.add(xems[e][m][s] <= xs[s]);
					model.add(xes[e][s] <= xs[s]); // obliczenie wartoœci xes co odpowiada wyra¿eniu sum(e in E)y_es
		break;
	case 2:
		for(s = 0; s < simData.flexGrid.slices; s++){
			IloExpr expr2(env);
			for (e = 0; e < simData.net.links; e++){
				//for (m = 0; m < simData.net.modes; m++)
					//model.add(xems[e][m][s] <= xs[s]);
					expr2 += xes[e][s];
			}
			model.add(expr2 <= E*xs[s]);
		}
		break;
	}					
	
	for (s = 0; s < simData.flexGrid.slices - 1; s++)
		model.add(xs[s] >= xs[s+1]); // ograniczenie polegaj¹ce na tym ¿e ka¿da nastêpna wartoœæ zmiennej xs musi byæ >= od bie¿¹cej

	model.add(zSpec == IloSum(xs)); // przypisanie sum(xs) do z - funkcji kryterium

	// --- objective definition
	objective = IloAdd(model, IloMinimize(env));
	objective.setExpr(zSpec);
	
	//--- pass model to cplex
	cplex.extract(model);

	cout << endl;
	//system("pause");
}

void MIP_RSSA::InitializeModel_ELB() {

	int c, d, e, i, k, l, m, n, p, q, s, t, v, w;
	Route rt;
	Channel ch;
	int M = simData.net.modes;
	int E = simData.net.links;
	cout << "  - Initializing MIP model: ";

	Preprocessing();
	
	// --- variables declaration
	xdp = IloArray<IloNumVarArray>(env, simData.traffic.demands); // przyjmuje wartosc 1 jezeli p-sciezka jest aktualnie wykorzystana w d-tym demandzie
	//xdpm = IloArray<IloArray<IloNumVarArray>>(env, simData.traffic.demands);	
	xdpem = IloArray<IloArray<IloArray<IloNumVarArray>>>(env, simData.traffic.demands);
	//xdpc = IloArray<IloArray<IloNumVarArray>>(env, simData.traffic.demands);

	for(d = 0; d < simData.traffic.demands; d++) {
		int numOfRts = simData.traffic.demandSet[d].candidateRoutes.size();
		xdp[d] = IloNumVarArray(env, numOfRts, 0, 1, ILOINT);
		//xdpm[d] = IloArray<IloNumVarArray>(env, numOfRts);
		xdpem[d] = IloArray<IloArray<IloNumVarArray>>(env, numOfRts);
		//xdpc[d] = IloArray<IloNumVarArray>(env, numOfRts);
		for(k = 0; k < numOfRts; k++) {
			rt = simData.traffic.demandSet[d].candidateRoutes[k];
			n = rt.getNumOfSlices();
			//xdpm[d][k] = IloNumVarArray(env, simData.net.modes, 0, 1, ILOINT);
			//xdpc[d][k] = IloNumVarArray(env, simData.flexGrid.chs[n].size(), 0, 1, ILOINT);
			xdpem[d][k] = IloArray<IloNumVarArray>(env, simData.net.links );
			for (e = 0; e < simData.net.links; e++) {
				xdpem[d][k][e] = IloNumVarArray(env, simData.net.modes , 0, 1, ILOINT); // przypisanie liczby modów dostêpnych na e-tym ³uku w k-tej œcie¿ce w d-tym demandzie. Ogólnie to jest wszêdzie jedna liczba. bo liczba modów na ³ukach jest sta³a
			}
		}
	}

	xs = IloNumVarArray(env, simData.flexGrid.slices, 0, 1, ILOINT);
	//xems = IloArray<IloArray<IloNumVarArray>>(env, simData.net.links);
	//for (e = 0; e < simData.net.links; e++) {
	//	xems[e] = IloArray<IloNumVarArray>(env, simData.net.modes);
	//	for (m = 0; m < simData.net.modes; m++) {
	//		xems[e][m] = IloNumVarArray(env, simData.flexGrid.slices, 0, 1, ILOINT);
	//	}
	//}
	xes = IloArray<IloNumVarArray>(env, simData.net.links); //wektor odpowiadaj¹cy y_es w modelu mat
	for (e = 0; e < simData.net.links; e++) {
		xes[e] = IloNumVarArray(env, simData.flexGrid.slices, 0, 1, ILOINT); // przypisanie dla ka¿dej krawêdzi wartoœci odpowiadaj¹cej liczbie mo¿liwych do alokacji slice'ów/plastrów czêstotliwoœci
	}

	
	zSpec = IloNumVar(env, 0, simData.flexGrid.slices, ILOINT);

	// --- constraints definition	
	for(d = 0; d < simData.traffic.demands; d++) {
		model.add(IloSum(xdp[d]) == 1); // odpowiada ograniczeniu sum(p in P(d)) x_dp = 1
		int numOfRts = simData.traffic.demandSet[d].candidateRoutes.size();
		for(k = 0; k < numOfRts; k++) {
			rt = simData.traffic.demandSet[d].candidateRoutes[k];
			n = rt.getNumOfSlices();
			IloExpr expr(env);
			for (e=0; e<rt.linkVector.size(); e++){ // dla ka¿dej krawêdzi w rozpatrywanej œcie¿ce
				for (m=0; m<simData.net.modes; m++){
					expr += xdpem[d][k][e][m]; 
				}
			}
			model.add(expr <= xdp[d][k]); // sum (m in M)x_dpem = x_dp; dla ka¿dego d in D, p in P(d), e in p
			expr.end();
		}
	}


	for(e = 0; e < simData.net.links; e++) { // dla ka¿dego e (krawedzi)
		for (m = 0; m < simData.net.modes; m++) { 
			IloExpr expr(env);
			for(d = 0; d < simData.traffic.demands; d++) { // dla ka¿dego d (zapotrzebowania, demanda)
				//cout << d << " ";
				int numOfRts = simData.traffic.demandSet[d].candidateRoutes.size();
				for(k = 0; k < numOfRts; k++) {	// dla ka¿dej k (œcie¿ki routingu)
					//cout << k << endl;
					rt = simData.traffic.demandSet[d].candidateRoutes[k];
					if (rt.doesInclude(e)) {
						n = rt.getNumOfSlices();
						if (ch.beginSlice <= s && s <= ch.endSlice && ch.size != 0) {
							//cout << "e = " << e << ", s = " << s << ": ";
							//ch->display();
							expr += xdpem[d][k][e][m] * n; 
						}
						
					}
				}
			}
			model.add(expr <= zSpec); // dla ka¿dego e oraz s, suma zajêtych kana³ów musi byæ mniejsza ni¿ zmienna xes (jej wartoœci to liczba wszystkich slice'ow na danej krawêdzi) - w modelu y_es. M - to licznoœæ zbioru modów i przez to przemna¿am (|M|y_es)
			expr.end();
		}
	}
	// --- objective definition
	objective = IloAdd(model, IloMinimize(env));
	objective.setExpr(zSpec);
	
	
	// --- objective definition
	objective = IloAdd(model, IloMinimize(env));
	
	
	//--- pass model to cplex
	cplex.extract(model);

	cout << endl;
	//system("pause");
}

void MIP_RSSA::Run() {

	//if(isDisplayed) cplex.setOut(cout);
	//else cplex.setOut(env.getNullStream());

	SetMIPParameters();
	cplex.use(StoreBestHeurCallback(env, this));
	cplex.solve();
}

double MIP_RSSA::FindLowerBound(int algOption, double &LBoptGap) {

	int b, c, d, e, h, i, j, k, l, m, n, q, s, v;
	int numOfRts, routes, size;
	Route rt;

	cout << "  - Initializing LB model" << endl;

	IloModel modelLB(env);
	IloCplex cplexLB = IloCplex(env);

	int CPUnumOfthreads = std::thread::hardware_concurrency();
	cplexLB.setParam(IloCplex::Threads, CPUnumOfthreads);

	// --- variables declaration
	IloArray<IloNumVarArray> xdpLB = IloArray<IloNumVarArray>(env, simData.traffic.demands);
	IloArray<IloArray<IloNumVarArray>> xdpmLB = IloArray<IloArray<IloNumVarArray>>(env, simData.traffic.demands);
	for (d = 0; d < simData.traffic.demands; d++) {
		xdpLB[d] = IloNumVarArray(env, simData.traffic.demandSet[d].candidateRoutes.size(), 0, 1, ILOINT);
		int numOfRts = simData.traffic.demandSet[d].candidateRoutes.size();
		xdpmLB[d] = IloArray<IloNumVarArray>(env, numOfRts);
		for (k = 0; k < numOfRts; k++)
			xdpmLB[d][k] = IloNumVarArray(env, simData.net.modes, 0, 1, ILOINT);
	}
	IloNumVarArray xeLB = IloNumVarArray(env, simData.net.links, 0, simData.flexGrid.slices, ILOINT);
	IloNumVar zLB = IloNumVar(env, 0, simData.flexGrid.slices, ILOINT);

	// --- constraints definition
	// - path and mode selection
	for (d = 0; d < simData.traffic.demands; d++) {
		modelLB.add(IloSum(xdpLB[d]) == 1);
		for (k = 0; k < simData.traffic.demandSet[d].candidateRoutes.size(); k++)
			modelLB.add(IloSum(xdpmLB[d][k]) == xdpLB[d][k]);
	}

	// - link usage
	for (e = 0; e < simData.net.links; e++) {
		for (m = 0; m < simData.net.modes; m++) {
			IloExpr expr(env);
			for (d = 0; d < simData.traffic.demands; d++) {
				for (k = 0; k < simData.traffic.demandSet[d].candidateRoutes.size(); k++) {
					rt = simData.traffic.demandSet[d].candidateRoutes[k];
					if (rt.doesInclude(e)) {
						n = rt.getNumOfSlices();
						expr = expr + n * xdpmLB[d][k][m];
					}
				}
			}
			modelLB.add(expr <= xeLB[e]);
			expr.end();
		}
		modelLB.add(xeLB[e] <= zLB);
	}

	// --- apply clique cuts (if algOption == 1)
	if (algOption == 1) {
		int i1, i2, i3;
		int r = 3;
		vector<vector<vector<int>>> combSet = vector<vector<vector<int>>>(5);
		SupportingFunctions supFun;

		for (int n = 3; n <= 7; n++) {
			i1 = n - 3;
			vector<bool> v(n);
			fill(v.begin() + r, v.end(), true);

			int numOfComb = supFun.fact(n) / (supFun.fact(r) * supFun.fact(n - r));
			combSet[i1] = vector<vector<int>>(numOfComb);

			i2 = 0;
			do {
				vector<int> comb;
				for (int i = 0; i < n; ++i)
					if (!v[i])
						comb.push_back(i);
				combSet[i1][i2] = comb;
				i2++;
			} while (next_permutation(v.begin(), v.end()));
		}
	
		for (v = 0; v < simData.net.nodeLinks.size(); v++) {
			int numOfAdjLinks = simData.net.nodeLinks[v].size();
			if (numOfAdjLinks >= 3 && numOfAdjLinks <= 7) {
				//if(numOfAdjLinks == 3) {
				//if(false) {
				i1 = numOfAdjLinks - 3;
				for (i2 = 0; i2 < combSet[i1].size(); i2++) {
					IloExpr expr(env);
					for (d = 0; d < simData.traffic.demands; d++) {
						numOfRts = simData.traffic.demandSet[d].candidateRoutes.size();
						for (k = 0; k < numOfRts; k++) {
							rt = simData.traffic.demandSet[d].candidateRoutes[k];
							int nd = rt.getNumOfSlices();
							vector<int> &linkVector = rt.linkVector;

							int commonLinks = 0;
							for (i3 = 0; i3 < combSet[i1][i2].size(); i3++) {
								j = combSet[i1][i2][i3];
								for (h = 0; h < linkVector.size(); h++) {
									e = linkVector[h];
									if (e == simData.net.nodeLinks[v][j]) {
										commonLinks++;
										break;
									}
								}
								if (commonLinks == 2)
									break;
							}
							if (commonLinks == 2) {
								expr += nd * xdpLB[d][k];
							}
						}
					}
					modelLB.add(expr <= zLB);
					expr.end();
				}
			}
		}
	}

	// --- objective definition
	IloObjective objective = IloAdd(modelLB, IloMinimize(env));
	objective.setExpr(zLB);

	cplex.setParam(IloCplex::ParallelMode, 1);
	cplexLB.setParam(IloCplex::TiLim, simData.algPar.LBrunTimeLimit);

	cplexLB.extract(modelLB);
	cplexLB.solve();

	//if (cplexLB.getStatus() == IloAlgorithm::Infeasible) {
	//	cout << "    Warning: infeasible solution" << endl;
	//	return IloInfinity;
	//}
	//else {
	//	//cout << "    - Selected routes: " << endl;
	//	foundRtIDs = vector<int>(simData.traffic.demands);
	//	for (d = 0; d < simData.traffic.demands; d++) {
	//		//cout << d << ": ";
	//		for (k = 0; k < simData.traffic.demandSet[d].candidateRoutes.size(); k++) {
	//			double value = cplexLB.getValue(xdpLB[d][k]);
	//			if (value > RC_EPS) {
	//				//cout << k << " ";
	//				foundRtIDs[d] = k;
	//			}
	//		}
	//		//cout << endl;
	//	}
	//}

	//cout << "    - ";
	//if (cplexLB.getStatus() == IloAlgorithm::Optimal)
	//	cout << "Optimal ";
	////cout << "LB value: " << cplexLB.getObjValue() << endl;
	//cout << "LB value: " << cplexLB.getBestObjValue() << endl;

	////cout << cplexLB.getObjValue() << " " << cplexLB.getBestObjValue() << endl;

	//////double lb = cplexLB.getObjValue();
	////double lb = cplexLB.getBestObjValue();
	double lb = (double)ceil(cplexLB.getBestObjValue() - RC_EPS);

	LBoptGap = abs(cplexLB.getObjValue() - cplexLB.getBestObjValue()) / (abs(cplexLB.getObjValue()) + 1e-10);
	LBoptGap = (double)floor(LBoptGap * 100000) / 100000.0;

	//system("pause");

	cplexLB.end();
	modelLB.end();

	return lb;


}


double MIP_RSSA::FindLowerBound_MC(int algOption, double &LBoptGap) {

	int b, c, d, e, h, i, j, k, l, m, n, q, s, v;
	int numOfRts, routes, size;
	Route rt;

	cout << "  - Initializing LB model" << endl;

	IloModel modelLB(env);
	IloCplex cplexLB = IloCplex(env);

	int CPUnumOfthreads = std::thread::hardware_concurrency();
	cplexLB.setParam(IloCplex::Threads, CPUnumOfthreads);

	// --- variables declaration
	IloArray<IloNumVarArray> xdpLB = IloArray<IloNumVarArray>(env, simData.traffic.demands);

	IloArray<IloArray<IloArray<IloNumVarArray>>> xdpemLB = IloArray<IloArray<IloArray<IloNumVarArray>>>(env, simData.traffic.demands);
	for (d = 0; d < simData.traffic.demands; d++) {
		xdpLB[d] = IloNumVarArray(env, simData.traffic.demandSet[d].candidateRoutes.size(), 0, 1, ILOINT);
		int numOfRts = simData.traffic.demandSet[d].candidateRoutes.size();
		xdpemLB[d] = IloArray<IloArray<IloNumVarArray>>(env, numOfRts);
		for (k = 0; k < numOfRts; k++){
			xdpemLB[d][k] = IloArray<IloNumVarArray>(env, simData.net.links);
			for(e = 0; e < simData.net.links; e++){
				xdpemLB[d][k][e] = IloNumVarArray(env, simData.net.modes, 0, 1, ILOINT);
			}
		}
	}
	IloNumVarArray xeLB = IloNumVarArray(env, simData.net.links, 0, simData.flexGrid.slices, ILOINT);
	IloNumVar zLB = IloNumVar(env, 0, simData.flexGrid.slices, ILOINT);

	// --- constraints definition
	// - path and mode selection
	for (d = 0; d < simData.traffic.demands; d++) {
		modelLB.add(IloSum(xdpLB[d]) == 1);
		for (k = 0; k < simData.traffic.demandSet[d].candidateRoutes.size(); k++){
			for (e = 0; e < simData.net.links; e++){
				rt = simData.traffic.demandSet[d].candidateRoutes[k];
				if (rt.doesInclude(e)) {
					modelLB.add(IloSum(xdpemLB[d][k][e]) == xdpLB[d][k]);
				}
			}
		}
	}


	// - link usage for MC
	for (e = 0; e < simData.net.links; e++) {
		for (m = 0; m < simData.net.modes; m++) {
			IloExpr expr(env);
			for (d = 0; d < simData.traffic.demands; d++) {
				for (k = 0; k < simData.traffic.demandSet[d].candidateRoutes.size(); k++) {
					rt = simData.traffic.demandSet[d].candidateRoutes[k];
					if (rt.doesInclude(e)) {
						n = rt.getNumOfSlices();
						expr = expr + n * xdpemLB[d][k][e][m];
					}
				}
			}
			modelLB.add(expr <= xeLB[e]);
			expr.end();
		}
		modelLB.add(xeLB[e] <= zLB);
	}

	// --- apply clique cuts (if algOption == 1)
	if (algOption == 1) {
		int i1, i2, i3;
		int r = 3;
		vector<vector<vector<int>>> combSet = vector<vector<vector<int>>>(5);
		SupportingFunctions supFun;

		for (int n = 3; n <= 7; n++) {
			i1 = n - 3;
			vector<bool> v(n);
			fill(v.begin() + r, v.end(), true);

			int numOfComb = supFun.fact(n) / (supFun.fact(r) * supFun.fact(n - r));
			combSet[i1] = vector<vector<int>>(numOfComb);

			i2 = 0;
			do {
				vector<int> comb;
				for (int i = 0; i < n; ++i)
					if (!v[i])
						comb.push_back(i);
				combSet[i1][i2] = comb;
				i2++;
			} while (next_permutation(v.begin(), v.end()));
		}
	
		for (v = 0; v < simData.net.nodeLinks.size(); v++) {
			int numOfAdjLinks = simData.net.nodeLinks[v].size();
			if (numOfAdjLinks >= 3 && numOfAdjLinks <= 7) {
				//if(numOfAdjLinks == 3) {
				//if(false) {
				i1 = numOfAdjLinks - 3;
				for (i2 = 0; i2 < combSet[i1].size(); i2++) {
					IloExpr expr(env);
					for (d = 0; d < simData.traffic.demands; d++) {
						numOfRts = simData.traffic.demandSet[d].candidateRoutes.size();
						for (k = 0; k < numOfRts; k++) {
							rt = simData.traffic.demandSet[d].candidateRoutes[k];
							int nd = rt.getNumOfSlices();
							vector<int> &linkVector = rt.linkVector;

							int commonLinks = 0;
							for (i3 = 0; i3 < combSet[i1][i2].size(); i3++) {
								j = combSet[i1][i2][i3];
								for (h = 0; h < linkVector.size(); h++) {
									e = linkVector[h];
									if (e == simData.net.nodeLinks[v][j]) {
										commonLinks++;
										break;
									}
								}
								if (commonLinks == 2)
									break;
							}
							if (commonLinks == 2) {
								expr += nd * xdpLB[d][k];
							}
						}
					}
					modelLB.add(expr <= zLB);
					expr.end();
				}
			}
		}
	}

	// --- objective definition
	IloObjective objective = IloAdd(modelLB, IloMinimize(env));
	objective.setExpr(zLB);

	cplex.setParam(IloCplex::ParallelMode, 1);
	cplexLB.setParam(IloCplex::TiLim, simData.algPar.LBrunTimeLimit);

	cplexLB.extract(modelLB);
	cplexLB.solve();


	double lb = (double)ceil(cplexLB.getBestObjValue() - RC_EPS);

	LBoptGap = abs(cplexLB.getObjValue() - cplexLB.getBestObjValue()) / (abs(cplexLB.getObjValue()) + 1e-10);
	LBoptGap = (double)floor(LBoptGap * 100000) / 100000.0;

	//system("pause");

	cplexLB.end();
	modelLB.end();

	return lb;


}


void MIP_RSSA::SetCplexParameters() {

	//cplex.setParam(IloCplex::RootAlg, IloCplex::Primal);
	//cplex.setParam(IloCplex::NumericalEmphasis, true);

	//cplex.setParam(IloCplex::AdvInd, 0);
}

void MIP_RSSA::SetMIPParameters() {

	cplex.setParam(IloCplex::TiLim, simData.algPar.MIPrunTimeLimit);
	cplex.setParam(IloCplex::TreLim, simData.algPar.MIPmemoryLimit);

	cplex.setParam(IloCplex::Threads, simData.algPar.CPUnumOfthreads);
//	//cplex.setParam(IloCplex::ParallelMode,0);
}

void MIP_RSSA::DisplayResults(string header) {

	// --- displaying results
	cout << endl << "    - Optimal value: " << cplex.getObjValue() << " (|S| = " << cplex.getValue(zSpec) << ")" << endl << endl;
}

void MIP_RSSA::SaveResults() {

	IloAlgorithm::Status cplexStatus = cplex.getStatus();
	string status;
	if(cplexStatus == IloAlgorithm::Optimal) status = "Optimal";
	else if(cplexStatus == IloAlgorithm::Feasible) status = "Feasib";
	else if(cplexStatus == IloAlgorithm::Infeasible) status = "Infeas";
	else status = "Unknown";

	fileWriter.Add(0, "\t");
	if(cplexStatus == IloAlgorithm::Optimal || cplexStatus == IloAlgorithm::Feasible) {

		fileWriter.Add(0, bestSpec);
		fileWriter.Add(0, "\t");
		//double objValue = ceil(cplex.getObjValue());
		double objValue = cplex.getObjValue();
		fileWriter.Add(0, objValue);
		fileWriter.Add(0, "\t");
		fileWriter.Add(0, cplex.getBestObjValue());
		double relativeGap = abs(cplex.getObjValue()-cplex.getBestObjValue())/(abs(cplex.getObjValue())+1e-10); 
		relativeGap = (double) floor(relativeGap * 100000) / 100000.0;
		fileWriter.Add(0, "\t");
		fileWriter.Add(0, relativeGap);
		fileWriter.Add(0, "\t");

	} else
		fileWriter.Add(0, "\t\t\t\t\t\t");

	fileWriter.Add(0, status);

	//ostringstream scen;
	//scen  << "\t" << cplex.getObjValue() << "\t" << cplex.getValue(zSpec) << "\t" << cplex.getValue(zTransc) << "\t" << cplex.getValue(zReg);
	//fileWriter.Add(0, scen.str());
}

void MIP_RSSA::DisplayMIPresults() {

	//cout << "    - Lightpath selection: " << endl;
	//for(int d = 0; d < simData.traffic.demands; d++) {
	//	cout << "      d" << d << ": ";
	//	int numOfRts = simData.traffic.demandSet[d].candidateRoutes.size();
	//	for(int k = 0; k < numOfRts; k++) {
	//		Route rt = simData.traffic.demandSet[d].candidateRoutes[k];
	//		if(std::round(cplex.getValue(xdp[d][k])) == 1) {
	//			cout << "k = " << k << ", ";
	//			//rt.display();
	//		}
	//		for (int m = 0; m < simData.net.modes; m++) {				
	//			int n = rt.getNumOfSlices();
	//			for (int c = 0; c < simData.flexGrid.chs[n].size(); c++) {
	//				ChannelPtr ch = simData.flexGrid.chs[n][c];
	//				if (std::round(cplex.getValue(xdpmc[d][k][m][c])) == 1) {
	//					cout << "c = " << c << endl;
	//					//cout << "          n = " << n << ", c = " << c << endl;
	//					//ch->display();
	//				}
	//			}
	//		}
	//	}
	//}

	//for(int e = 0; e < simData.net.links; e++) {
	//	cout << "Link " << e << ": \t";
	//	for(int s = 0; s < simData.flexGrid.slices; s++) {
	//		cout << std::round(cplex.getValue(xes[e][s])) << " ";
	//	}
	//	cout << endl;
	//}

	//for(int s = 0; s < simData.flexGrid.slices; s++) {
	//	cout << cplex.getValue(xs[s]) << " ";
	//}	
	//cout << endl;
}