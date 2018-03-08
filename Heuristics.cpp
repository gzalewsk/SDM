#include "StdAfx.h"
#include "Heuristics.h"
#include <Windows.h> //Sleep function
#include <math.h>
#include <algorithm>
#include <random>
#include "NetworkState.h"
#include "Lightpath.h"
#include <ctime>
#include "NetworkScenario.h"

SimulatedAnnealing::SimulatedAnnealing(CommonDataSpace *comDataSpace) {

	SimulatedAnnealing::SimulatedAnnealing(comDataSpace, 0);
}

SimulatedAnnealing::SimulatedAnnealing(CommonDataSpace *comDataSpace, int threadId) {
	
	this->mainThreadId = threadId;
	this->comDataSpace = comDataSpace;

	randGen = default_random_engine(this->mainThreadId);
}

vector<LightpathNew> SimulatedAnnealing::GenerateInitialSolutionNew(RSSA &optProblem, SimulationData &simData) {
	cout << endl << "----------------------------------------" << endl;
	cout << "- Generating a random solution" << endl;
	clock_t tStart = clock();
	NetworkState netState = NetworkState(simData);
	// generate a random vector of demands
	vector<int> order = vector<int>(simData.traffic.demands);
	for(int d = 0; d < simData.traffic.demands; d++){
		order[d] = d;
	}
	permuteDemands(order);
	
	// evaluate the demand vector
	//double objValue = optProblem.EvaluateSolution(order, netState);
	double objValue = optProblem.EvaluateSolution(order, netState, simData);
	//// verify the network state
	//netState.Verify();
	// save the solution as a global solution
	comDataSpace->bestObjectiveGlobal = objValue;
	comDataSpace->bestOrderGlobal = order;
	comDataSpace->bestSolutionTime = (double)(clock() - comDataSpace->algStartTimeGlobal) / CLOCKS_PER_SEC;
	comDataSpace->bestNetState.ClearState();
	comDataSpace->bestNetState = netState;
	double runTime = (double)(clock() - tStart) / CLOCKS_PER_SEC;
	cout << "  - Solution found: objVal = " << objValue << ", |S| = " << netState.GetMaxSliceIndex() << endl;
	cout << "  - Run time: " << runTime << endl;
	cout << "----------------------------------------" << endl;
	return 	netState.allocatedLightpathsNew;
}

vector<Lightpath> SimulatedAnnealing::GenerateInitialSolution(RSSA &optProblem, SimulationData &simData) {

	cout << endl << "----------------------------------------" << endl;
	cout << "- Generating a random solution" << endl;

	clock_t tStart = clock();
	NetworkState netState = NetworkState(simData);

	// generate a random vector of demands
	vector<int> order = vector<int>(simData.traffic.demands);
	
	for(int d = 0; d < simData.traffic.demands; d++){
		order[d] = d;
	}
	
	// evaluate the demand vector
	//double objValue = optProblem.EvaluateSolution(order, netState);
	double objValue = optProblem.EvaluateSolution(order, netState, simData);

	//// verify the network state
	//netState.Verify();

	// save the solution as a global solution
	comDataSpace->bestObjectiveGlobal = objValue;
	comDataSpace->bestOrderGlobal = order;
	comDataSpace->bestSolutionTime = (double)(clock() - comDataSpace->algStartTimeGlobal) / CLOCKS_PER_SEC;
	comDataSpace->bestNetState.ClearState();
	comDataSpace->bestNetState = netState;

	double runTime = (double)(clock() - tStart) / CLOCKS_PER_SEC;
	cout << "  - Solution found: objVal = " << objValue << ", |S| = " << netState.GetMaxSliceIndex() << endl;
	cout << "  - Run time: " << runTime << endl;
	cout << "----------------------------------------" << endl;

	return 	netState.allocatedLightpaths;
}

void SimulatedAnnealing::SolveProblem(RSSA &optProblem, SimulationData &simData) {
	
	int d, d1, d2, e, i, j;
	double runTime;

	// --- initialize data

	NetworkState netState = NetworkState(simData);

	double bestObjective = comDataSpace->bestObjectiveGlobal;
	double currentBestObjVal = bestObjective;
	int bestIter = 0;

	// generate a random vector of demands
	vector<int> order = vector<int>(simData.traffic.demands);
	for (int j = 0; j < simData.traffic.demands; j++)
		order[j] = j;
	permuteDemands(order);

	// initialize SimAn parameters
	double cooling = simData.algPar.heurSAcooling;
	double initTemperature = bestObjective * simData.algPar.heurSAtemperatureCoeff;
	double temperature = initTemperature;
	long iter = 0;

	// --- run the main algorithm loop
	//while (iter < simData.algPar.heurSAiterLimit) {
	while (true) {

		// terminate whenever lower-bound == upper-bound
		mtx.lock();
		if (comDataSpace->bestObjectiveGlobal == comDataSpace->globalLowerBound) {
			cout << comDataSpace->bestObjectiveGlobal << " == " << comDataSpace->globalLowerBound << " in thread " << this->mainThreadId << endl;
			break;
		}
		mtx.unlock();

		// check and update local solution with the best global solution
		//mtx.lock();
		if (bestObjective > comDataSpace->bestObjectiveGlobal) {
			// update objective function (in all threads)
			bestObjective = comDataSpace->bestObjectiveGlobal;
			// update demand order (only in even threads)
			//if (mainThreadId % 2 == 0) {
			//	//cout << "update thread " << this->mainThreadId << " with best order" << endl;
				order = comDataSpace->bestOrderGlobal;
			//}
		}
		//mtx.unlock();

		// generate neighbouring solution by swapping two demands
		d1 = (int)(randGen() % simData.traffic.demands);
		d2 = (int)(randGen() % (simData.traffic.demands - 1));
		if (d1 <= d2) d2++;
		int temp = order[d1]; order[d1] = order[d2]; order[d2] = temp;

		// evaluate the solution
		//double tempObjVal = optProblem.EvaluateSolution(order, netState);
		double tempObjVal = optProblem.EvaluateSolution(order, netState, simData);

		// calcluate the difference in objective values
		double deltaObjVal = tempObjVal - currentBestObjVal;

		// if there is and improvement in the objective value wrt to the current solution
		if (deltaObjVal <= 0) {

			currentBestObjVal = tempObjVal;

			// if there is and improvement in the objective value wrt to the best local solution
			if (currentBestObjVal < bestObjective) {

				bestObjective = currentBestObjVal;
				bestIter = iter;
				runTime = (double)(clock() - comDataSpace->algStartTimeGlobal) / CLOCKS_PER_SEC;

				// check and update the best global solution with the best local solution
				mtx.lock();
				if (bestObjective < comDataSpace->bestObjectiveGlobal) {
					//cout << bestObjective << " < " << comDataSpace->bestObjectiveGlobal << " in thread " << this->mainThreadId << endl;
					comDataSpace->bestObjectiveGlobal = bestObjective;
					comDataSpace->bestOrderGlobal = order;
					comDataSpace->bestSolutionTime = runTime;
					comDataSpace->bestNetState.ClearState();
					comDataSpace->bestNetState = netState;
					double optGapPerc = (double)floor(((comDataSpace->bestObjectiveGlobal - comDataSpace->globalLowerBound) / comDataSpace->bestObjectiveGlobal) * 10000) / 100.0;
					DisplayResults(netState, bestObjective, iter, runTime, mainThreadId, optGapPerc);

					//// verify the network state
					//if (!netState.Verify()) {
					//	cout << "Warning: wrong netState!" << endl;
					//	exit(0);
					//}
				}
				mtx.unlock();
			}

		}
		else {

			// change current solution with some probability
			if ((randGen() % 1000) / 1000.0 < exp((double)-deltaObjVal / temperature)) {
				currentBestObjVal = tempObjVal;
			}
			else {
				temp = order[d1]; order[d1] = order[d2]; order[d2] = temp;
			}
			//permuteDemands(order);
		}

		// clear network state
		netState.ClearState();

		// update the SimAn parameters
		temperature = temperature * cooling;
		iter++;

		runTime = (double)(clock() - comDataSpace->algStartTimeGlobal) / CLOCKS_PER_SEC;
		if (runTime > simData.algPar.heurSAtimeLimit) {
			//cout << "Time exceeded: " << runTime << " > " << simData.algPar.heurSAtimeLimit << endl;
			break;
		}
		if (iter > simData.algPar.heurSAiterLimit) {
			//cout << "Iterations exceeded: " << iter << " > " << simData.algPar.heurSAiterLimit << endl;
			break;
		}
	}

	comDataSpace->mtx.lock();
	comDataSpace->overallNumOfIterations += iter;
	comDataSpace->mtx.unlock();

	////cout << "ending thread " << this->mainThreadId << endl;

	//comDataSpace->mtx.lock();
	//cout << "  - Thread " << mainThreadId << " finished" << endl;
	//comDataSpace->mtx.unlock();

	//cout << "  - Stopping criteria: ";
	//if (iter >= simData.algPar.heurSAiterLimit)
	//	cout << "iteration limit exceeded (" << iter << " >= " << simData.algPar.heurSAiterLimit << ")";
	//else if (temperature <= simData.algPar.heurSAtempLimit)
	//	cout << "low temperature (" << temperature << " <= " << simData.algPar.heurSAtempLimit << ")";
	//else
	//	cout << "time limit (" << runTime << " > " << simData.algPar.heurSAtimeLimit << ")";
	//cout << endl;

	//runTime = (double)(clock() - tStart) / CLOCKS_PER_SEC;
	//cout << "  - Solution found at iter " << bestIter << ": objVal = " << bestObjective << ", |S| = " << maxSliceIndex << endl;
	//cout << "  - Run time: " << runTime << endl;
	//cout << "----------------------------------------" << endl;

	//ostringstream scen;
	//scen << "\t" << bestIter << "\t" << bestObjective << "\t" << maxSliceIndex;
}

void SimulatedAnnealing::SolveProblemGr2(RSSA &optProblem, SimulationData &simData) {
//void SimulatedAnnealing::SolveProblemGr2(RSSA optProblem, SimulationData simData) {
	
	int d, d1, d2, e, i, j;
	double runTime;

	// --- initialize data

	NetworkState netState = NetworkState(simData);

	double bestObjective = comDataSpace->bestObjectiveGlobal;
	double currentBestObjVal = bestObjective;
	int bestIter = 0;

	// generate a random vector of demands
	vector<int> order = vector<int>(simData.traffic.demands);
	for (int j = 0; j < simData.traffic.demands; j++)
		order[j] = j;
	permuteDemands(order);

	// initialize SimAn parameters
	double cooling = simData.algPar.heurSAcooling;
	double initTemperature = bestObjective * simData.algPar.heurSAtemperatureCoeff;
	double temperature = initTemperature;
	//double temperature0 = initTemperature;
	long iter = 0;

	// --- run the main algorithm loop
	//while (iter < simData.algPar.heurSAiterLimit) {
	while (true) {

		// terminate whenever lower-bound == upper-bound
		//mtx.lock();
		if (comDataSpace->bestObjectiveGlobal == comDataSpace->globalLowerBound) {
			//cout << comDataSpace->bestObjectiveGlobal << " == " << comDataSpace->globalLowerBound << "in thread " << this->mainThreadId << endl;
			break;
		}
		//mtx.unlock();

		// check and update local solution with the best global solution
		////mtx.lock();
		if (bestObjective > comDataSpace->bestObjectiveGlobal) {
			// update objective function (in all threads)
			bestObjective = comDataSpace->bestObjectiveGlobal;
			//// update demand order (only in even threads)
			//if (iter >= 1500 && this->mainThreadId % 2 == 0) {
			if (iter >= 1500) {
					//cout << "update thread " << this->mainThreadId << " with best order" << endl;
				order = comDataSpace->bestOrderGlobal;
			};
		}
		if (iter % (500) == 0 && iter < 1500) {
			order = comDataSpace->bestOrderGlobal;
		};
		////mtx.unlock();
		//if (bestObjective > comDataSpace->bestObjectiveGlobal) {
		//	// update objective function (in all threads)
		//	bestObjective = comDataSpace->bestObjectiveGlobal;
		//	//// update demand order (only in even threads)
		//	//if (iter >= 1500 && this->mainThreadId % 2 == 0) {
		//}
		//if (iter % (500) == 0)
		//	if (bestObjective > comDataSpace->bestObjectiveGlobal)
		//		order = comDataSpace->bestOrderGlobal;

		// generate neighbouring solution by swapping two demands
		d1 = (int)(randGen() % simData.traffic.demands);
		d2 = (int)(randGen() % (simData.traffic.demands - 1));
		if (d1 <= d2) d2++;
		int temp = order[d1]; order[d1] = order[d2]; order[d2] = temp;
		// evaluate the solution
		//double tempObjVal = optProblem.EvaluateSolution(order, netState);
		double tempObjVal = optProblem.EvaluateSolution(order, netState, simData);

		// calcluate the difference in objective values
		double deltaObjVal = tempObjVal - currentBestObjVal;

		// if there is and improvement in the objective value wrt to the current solution
		if (deltaObjVal <= 0) {

			currentBestObjVal = tempObjVal;

			// if there is and improvement in the objective value wrt to the best local solution
			if (currentBestObjVal < bestObjective) {

				bestObjective = currentBestObjVal;
				bestIter = iter;
				runTime = (double)(clock() - comDataSpace->algStartTimeGlobal) / CLOCKS_PER_SEC;

				// check and update the best global solution with the best local solution
				mtx.lock();
				if (bestObjective < comDataSpace->bestObjectiveGlobal) {
					//cout << bestObjective << " < " << comDataSpace->bestObjectiveGlobal << " in thread " << this->mainThreadId << endl;
					comDataSpace->bestObjectiveGlobal = bestObjective;
					comDataSpace->bestOrderGlobal = order;
					comDataSpace->bestSolutionTime = runTime;
					comDataSpace->bestNetState.ClearState();
					comDataSpace->bestNetState = netState;
					double optGapPerc = (double)floor(((comDataSpace->bestObjectiveGlobal - comDataSpace->globalLowerBound) / comDataSpace->bestObjectiveGlobal) * 10000) / 100.0;
					DisplayResults(netState, bestObjective, iter, runTime, mainThreadId, optGapPerc);
				
					//// verify the network state
					//if (!netState.Verify()) {
					//	cout << "Warning: wrong netState!" << endl;
					//	exit(0);
					//}
				}
				mtx.unlock();
			}

		}
		else {

			// change current solution with some probability
			if ((randGen() % 1000) / 1000.0 < exp((double)-deltaObjVal / temperature)) {
				currentBestObjVal = tempObjVal;
			}
			else {
				temp = order[d1]; order[d1] = order[d2]; order[d2] = temp;
			}
		}

		// clear network state
		netState.ClearState();

		// --- ver1
		if (iter < 1500){ temperature = temperature * pow(cooling, (80 / (this->mainThreadId + 1))); }
		else if (iter == 1500) { temperature = temperature * pow(cooling, 5000); }
		else { temperature = temperature * cooling; };
		// --- ver2
		//temperature = temperature * pow(cooling, (80 / (this->mainThreadId + 1)));
		// --- ver3
		//if (iter < 1500){ temperature = temperature * pow(cooling, (80 / (this->mainThreadId + 1))); }
		//else if (iter == 1500) { temperature = initTemperature * pow(cooling, 1500); }
		//else { temperature = temperature * cooling; };

		iter++;

		runTime = (double)(clock() - comDataSpace->algStartTimeGlobal) / CLOCKS_PER_SEC;
		if (runTime > simData.algPar.heurSAtimeLimit) {
			//cout << "Time exceeded: " << runTime << " > " << simData.algPar.heurSAtimeLimit << endl;
			break;
		}
	}

	comDataSpace->mtx.lock();
	comDataSpace->overallNumOfIterations += iter;
	comDataSpace->mtx.unlock();

	//cout << "ending thread " << this->mainThreadId << endl;
}

void SimulatedAnnealing::permuteDemands(vector<int> &order) {

	// permute the vector of demands
	shuffle(order.begin(), order.end(), randGen);
}

void SimulatedAnnealing::DisplayResults(NetworkState &netState, double objective, int iter, double time, int threadId, double optGapPerc)  {

	comDataSpace->mtx.lock();
	cout << "    - Best solution: objVal = " << objective << ", |S| = " << netState.GetMaxSliceIndex() << " (thread " << threadId << ", iter = " << iter << ", time = " << time << ", optimality gap = " << optGapPerc << "%)" << endl;
	comDataSpace->mtx.unlock();
}