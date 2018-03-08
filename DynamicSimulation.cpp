#include "StdAfx.h"
#include "Heuristics.h"
#include "DynamicSimulation.h"
#include <Windows.h> //Sleep function
#include <math.h>
#include <algorithm>
#include <random>
#include "NetworkState.h"
#include "Lightpath.h"
#include <ctime>
#include "NetworkScenario.h"
#include <tuple>


int DynamicSimulation::generate_exp_v1(int min, int rate, mt19937 &gen){
    exponential_distribution<> d(1);
 	double g;
 	g = min+rate*d(gen);
    return g;
};

int DynamicSimulation::generate_exp_v2(int exp_dist_mean){
	double const exp_dist_lambda = 1 / exp_dist_mean;
	std::random_device rd; 
	std::exponential_distribution<> rng (exp_dist_lambda);
	std::mt19937 rnd_gen (rd ());
	int random_nr = int ( rng (rnd_gen) );
	return random_nr;
}

DynamicSimulation::DynamicSimulation(CommonDataSpace *comDataSpace) {
	DynamicSimulation::DynamicSimulation(comDataSpace, 0);
}

DynamicSimulation::DynamicSimulation(CommonDataSpace *comDataSpace, int threadId) {
	this -> mainThreadId;
	this -> comDataSpace;
	randGen = default_random_engine(this -> mainThreadId);
}

void DynamicSimulation::SolveProblemDynamic(NetworkState &netState, RSSA &optProblem, SimulationData &simData){
	list<int>::iterator d;
	double tStart = (double)(clock()) / CLOCKS_PER_SEC;
	//cout << tStart << endl;
	while (true){
		for (int r=0; r<rand()%10; r++){
			netState.orders.push_back(rand()%netState.demands);
		};
		for (d=netState.orders.begin(); d!=netState.orders.end(); ++d){
			cout << ' ' << *d;
			int demand = *d;
			lightpathSetup(demand, netState, optProblem, simData);
		};
		cout << endl;
			
		for (int r=0; r<rand()%10; r++){
			if (netState.orders.size() > 0){
				cout << "tear down order " << netState.orders.front() << endl;
				lightpathTearDown(r, netState.allocatedLightpathsNew[netState.orders.front()], netState, optProblem, simData);
				netState.orders.pop_front();
			}
		}
		
		Sleep(1000);	
	}
}

struct event {
	string type;
	double time;
	int demand;	
};

struct demandStatistic {
	string type;
	double time;
	int demand;	
	int hd;
};

void DynamicSimulation::SolveProblemDynamicV2(NetworkState &netState, RSSA &optProblem, SimulationData &simData){
	netState.allocatedLightpathsNew = vector<LightpathNew>(5000); //max 5000 demands
	
	cout << endl;
	cout << "simulation start" << endl;
	list<event> l;
	int step_id = 0;
	int demand_id = 0;
	
	//vector<Demand> demandSet;
	
	int d, e, k, n, p, q, t, v, src, dest, hd;
	int demandsInFile, numOfSlices, len;
	Route rt;
	int act_demand;
	string line;

	while(true){
		dest=src=0;
		while (src==dest){
			src = rand()%11;
			dest = rand()%11;
		};
		hd=50*(rand()%19+1);
		cout << "step: " << step_id << "------------------------------------------------" << endl;
		//cout << step_id << " "<< src << " " << dest << " " << hd << "<-------------" << endl;
		Demand demand = Demand(src, dest, hd);
		
		if (simData.traffic.netScen.routeManager.precomputedRoutes[src][dest].size() == 0) {
				cout << "Warning: no candidate routes for demand " << d << " (nodes: " << src << "-" << dest << ")" << endl;
				system("pause");
				exit(0);
			}
		else {
			int numOfRts = simData.traffic.netScen.routeManager.precomputedRoutes[src][dest].size();
			cout << numOfRts << "- number of routes" << endl;
			demand.candidateRoutes = vector<Route>(numOfRts);
			for (k = 0; k < numOfRts; k++) {
				rt = simData.traffic.netScen.routeManager.precomputedRoutes[src][dest][k];
				n = ConvertBitrateToSlices(hd, rt.getLength());
				rt.setNumOfSlices(n);
				rt.display();
				demand.candidateRoutes[k] = rt;
			}
			simData.traffic.demandSet.push_back(demand);
			cout << "DemandSet- (";
			for (auto i = simData.traffic.demandSet.begin(); i != simData.traffic.demandSet.end(); ++i){	
				cout << "{src:" << (*i).src << ", dest:" << (*i).dest << ", hd:" << (*i).hd << "},";
			};
			cout << ") " << endl;
			simData.traffic.netScen.routeManager.numOfCandidateRoutes += simData.traffic.netScen.routeManager.precomputedRoutes[src][dest].size();
		}
		double actualTimePoint = 0;
		double eventTime = rand();
		event z1 = {"setup", eventTime, step_id};
		event z2 = {"teardown", eventTime+1+rand(), step_id};
		list<event>::iterator i;
		if (l.size() == 0){
			l.push_front(z1);
			l.push_back(z2);
			actualTimePoint = l.front().time;
			cout << "Aktualna tabela zdarzen: " << endl;			
			for ( i = l.begin() ; i!=l.end() ; ++i){
				(*i).time -= actualTimePoint;
				cout << (*i).type << " | " << (*i).time << " | "<< (*i).demand << endl;		
			}
		}		
		else {
			int flag1 = 0;
			int flag2 = 0;
			for ( i = l.begin() ; i!=l.end() ; ++i){ //zapis setup w odpowiednim miejscu na liscie
				if ((*i).time > z1.time){
				 	l.insert(i,z1);
					flag1 = 1;
					break; 
				};
			}
			for ( i = l.begin() ; i!=l.end() ; ++i){ //zapis teardown w odpowiednim miejscu na liscie
				if ((*i).time > z2.time){
				 	l.insert(i,z2);
				 	flag2 = 1;
					break; 
				}
				if (i==l.end() ) l.push_back(z2);
			}
			if (flag1 ==0) l.push_back(z1); // jezeli przeszlismy cala iteracje i caly czas time bylo wieksze od kazdej poruwnywanej wartosci to nalezy dodac to na koniec
			if (flag2 ==0) l.push_back(z2); // teardown zawsze bedzie za setupem wstawionym przed chwila
			actualTimePoint = l.front().time;
			cout << "Aktualna tabela zdarzen: " << endl;			
			for ( i = l.begin() ; i!=l.end() ; ++i){
				(*i).time -= actualTimePoint;
				cout << (*i).type << " | " << (*i).time << " | "<< (*i).demand << endl;		
			}
		}
		//for ( i = l.begin() ; i!=l.end() ; ++i){ //petla pozwalajaca na aktualizacje time po kolejnym kroku. wartosci atrybutu time sa zmniejszane o actualTimePoint
		//	(*i).time -= actualTimePoint;
		//	cout << (*i).type << " | " << (*i).time << " | "<< (*i).demand << endl;		
		//}
		//cout << "actual time point " << actualTimePoint << endl;
		
		//Sleep(1000);
		
		// zrealizowanie pierwszego zdarzenia z brzegu i odjecie czasu actualTime od wszystkich zdarzen
		
		
		if (l.front().type == "setup"){
			optProblem.SetupSingleOrder(l.front().demand, netState, simData);
			l.pop_front() ;
			netState.allocatedLightpathsNew[l.front().demand].display();
		}
		else {
			lightpathTearDown(l.front().demand, netState.allocatedLightpathsNew[l.front().demand], netState, optProblem, simData);
			l.pop_front();
		}
		//cout << netState.slices << " <---- number of slices" << endl;
		//cout << netState.GetMaxSliceIndex() << " <---- maxSliceIndex" << endl;
		//netState.DisplayState();
		/*
		for (int d = 0 ; d < netState.allocatedLightpathsNew.size() ; d++){
			cout << d << "- demand; ";
			netState.allocatedLightpathsNew[d].display();
			cout << endl;
		}
		*/
		cout << l.front().type << " <-type | " << l.front().time << " <-time | " << l.front().demand << " <-demand --zdarzenie usunieto." << endl;
		cout << endl;
		++step_id;		
	}
}

void DynamicSimulation::lightpathSetup(int d, NetworkState &netState, RSSA &optProblem, SimulationData &simData){
	optProblem.SetupSingleOrder(d, netState, simData);
}
void DynamicSimulation::lightpathTearDown(int d, LightpathNew &lightpath, NetworkState &netState, RSSA &optProblem, SimulationData &simData){
	optProblem.removeLpFromSolution(d, lightpath.rt, lightpath.ch.beginSlice, lightpath.modeVector, netState, simData);
}

int DynamicSimulation::ConvertBitrateToSlices(double bitrate, double pathLength) {
	int slices;
	double sliceWidth = 12.5;
	int maxTrDistinance = 6300;
	//if (simPar.QoTmodelID == JLT2016) {
	// transmission range: BPSK - 6300km, QPSK - 3500km, 8QAM - 1200km, 16QAM - 600km
	int modulationFormat = -1;
	int opticalCarriers;
	if (pathLength > maxTrDistinance) {
		cout << "Warning: too long path in the JLT2016 model" << endl;
		//system("pause"); exit(0);
		opticalCarriers = ceil(bitrate / 50);		// BPSK with regeneration
	}
	else if (pathLength > 3500) {		
		opticalCarriers = ceil(bitrate / 50);		// BPSK
	} 
	else if (pathLength > 1200) {
		opticalCarriers = ceil(bitrate / 100);		// QPSK
	}
	else if (pathLength > 600)
		opticalCarriers = ceil(bitrate / 150);		// 8QAM
	else
		opticalCarriers = ceil(bitrate / 200);		// 16QAM
	double spectrum = 37.5 * opticalCarriers + 12.5;
	slices = ceil(spectrum / sliceWidth);
	if (slices < 1) cout << "Slices less than 1... " << slices << endl;
	//cout << "bit-rate = " << bitrate << ", pathLength = " << pathLength << " -> opticalCarriers = " << opticalCarriers << ", spectrum = " << spectrum << ", slices = " << slices << endl;
	return slices;
}



void DynamicSimulation::SolveProblemDynamicV3(NetworkState &netState, RSSA &optProblem, SimulationData &simData){
	// parametry symulacji
	int IAT = 500; // czas miêdzy kolejnymi zdarzeniami - wartoœæ œrednia // inter arrival time
	int ADT = 2000; // czas trwania alokacji zapotrzebowania - wartoœæ œrednia // average duration time
	int lambda = 1/IAT; // czêstotliwoœæ przychodzenia kolejnych zg³oszeñ
	int mi = 1/ADT; // ile œrednio zdarzeñ mo¿na zmieœciæ w jednostce czasu
	int ro = lambda/mi; // wspó³czynnik otrzymuj¹cy mniejsz¹ wartoœæ je¿eli zdarzenia przychodz¹ rzadziej i trwaj¹ krócej, wtedy mo¿emy spodziewaæ siê ma³ego obci¹¿enia sieci.
	cout << simData.flexGrid.slices << endl;
	random_device rd;
    mt19937 gen(rd());
	netState.allocatedLightpathsNew = vector<LightpathNew>(5000); //max 5000 demands
	vector<demandStatistic> demandsServed;
	vector<demandStatistic> demandsUnserved;
	int demandsServedCounter = 0;
	int demandsUnservedCounter = 0;
	int hdServedCounter = 0;
	int hdUnservedCounter = 0;

	cout << endl;
	cout << "------------- simulation start -------------" << endl;
	list<event> l;
	int step_id = 0;
	int demand_id = 0;
	
	int d, e, k, n, p, q, t, v, src, dest, hd;
	int demandsInFile, numOfSlices, len;
	Route rt;
	int act_demand;
	string line;
	double actualTimePoint = 0;
	while(true){
		
		cout << "generacja zapotrzebowania" << endl;
		dest=src=0;
		while (src==dest){
			src = rand()%11;
			dest = rand()%11;
		};
		hd=50*(rand()%19+1);
		cout << "step: " << step_id << "------------------------------------------------" << endl;
		Demand demand = Demand(src, dest, hd);
		if (simData.traffic.netScen.routeManager.precomputedRoutes[src][dest].size() == 0) {
				cout << "Warning: no candidate routes for demand " << d << " (nodes: " << src << "-" << dest << ")" << endl;
				system("pause");
				exit(0);
			}
		else {
			int numOfRts = simData.traffic.netScen.routeManager.precomputedRoutes[src][dest].size();
			cout << numOfRts << "- number of routes" << endl;
			demand.candidateRoutes = vector<Route>(numOfRts);
			for (k = 0; k < numOfRts; k++) {
				rt = simData.traffic.netScen.routeManager.precomputedRoutes[src][dest][k];
				n = ConvertBitrateToSlices(hd, rt.getLength());
				rt.setNumOfSlices(n);
				rt.display();
				demand.candidateRoutes[k] = rt;
			}
			simData.traffic.demandSet.push_back(demand);
			/*
			cout << "DemandSet- (";
			for (auto i = simData.traffic.demandSet.begin(); i != simData.traffic.demandSet.end(); ++i){	
				cout << "{src:" << (*i).src << ", dest:" << (*i).dest << ", hd:" << (*i).hd << "},";
			};
			cout << ") " << endl;
			*/
			simData.traffic.netScen.routeManager.numOfCandidateRoutes += simData.traffic.netScen.routeManager.precomputedRoutes[src][dest].size();
		}
		double eventTime = generate_exp_v1(int(IAT/1.8),800, gen);
		actualTimePoint += eventTime;
		event z1 = {"setup", actualTimePoint, step_id};
		event z2 = {"teardown", actualTimePoint+generate_exp_v1(int(ADT/1.8),800, gen), step_id};
		list<event>::iterator i;
		cout << "dodawanie wygenerowanego zapotrzebowania do tabeli zdarzen" << endl;
		if (l.size() == 0){
			l.push_front(z1);
			l.push_back(z2);
		}		
		else {
			int flag1 = 0;
			int flag2 = 0;
			for ( i = l.begin() ; i!=l.end() ; ++i){ //zapis setup w odpowiednim miejscu na liscie
				if ((*i).time > z1.time){
				 	l.insert(i,z1);
					flag1 = 1;
					break; 
				};
			}
			for ( i = l.begin() ; i!=l.end() ; ++i){ //zapis teardown w odpowiednim miejscu na liscie
				if ((*i).time > z2.time){
				 	l.insert(i,z2);
				 	flag2 = 1;
					break; 
				}
				if (i==l.end() ) l.push_back(z2);
			}
			if (flag1 ==0) l.push_back(z1); // jezeli przeszlismy cala iteracje i caly czas time bylo wieksze od kazdej poruwnywanej wartosci to nalezy dodac to na koniec
			if (flag2 ==0) l.push_back(z2); // teardown zawsze bedzie za setupem wstawionym przed chwila
		};
		cout << "Aktualna tabela zdarzen: " << endl;			
		for ( i = l.begin() ; i!=l.end() ; ++i){
			cout << (*i).type << " | " << (*i).time << " | "<< (*i).demand << endl;		
		}

		string actualEvent = l.front().type;
		int actualDemand = l.front().demand;
		cout << l.front().type << " <-type | " << l.front().time << " <-time | " << l.front().demand << " <-demand --zaczeto proces usuwania zdarzenia." << endl;
		cout << endl;
		
		if (actualEvent == "setup"){
			cout << "usuwanie zdarzenia setup" << endl;
			bool isDemandAllocated = optProblem.SetupSingleOrderFS(actualDemand, netState, simData);
			if (isDemandAllocated == 1) {
				cout << "alokacja zapotrzebowania udana, alokacja zasobow sieci" << endl;
				demandStatistic e = {actualEvent, l.front().time, actualDemand, hd};
				demandsServed.push_back(e);
				demandsServedCounter++;
				hdServedCounter += hd;
				l.pop_front();
				netState.allocatedLightpathsNew[l.front().demand].display();
			}
			else{
				cout << "alokacja zapotrzebowania nieudana" << endl;
				demandStatistic e = {actualEvent, l.front().time, actualDemand, hd};
				demandsUnserved.push_back(e);
				demandsUnservedCounter++;
				hdUnservedCounter += hd;
				cout << "usuwanie zdarzenia teardown odpowiadajacego niezaalokowanemu zapotrzebowaniu" << endl;
				l.pop_front();
				for ( i = l.begin() ; i!=l.end() ; i++){ //wyrzucenie zdarzenie teardown odpowiadaj¹cego nieprzyjêtemu demandowi
					if ((*i).demand == actualDemand){
						l.erase(i);
						break;
					};
				}
			}
		}
		
		else {
			int tearDownFlag = 1;
			while (tearDownFlag==1){
				cout << "usuwanie zdarzenia teardown, zwalnianie zasobow sieci" << endl;
				lightpathTearDown(l.front().demand, netState.allocatedLightpathsNew[l.front().demand], netState, optProblem, simData);
				l.pop_front();
				string actualEvent = l.front().type;
				int actualDemand = l.front().demand;
				if (actualEvent == "setup"){
					tearDownFlag=0;
				};
			}
		}
		/*
		cout << "Aktualna tabela zdarzen (po procesie usuwania zdarzen): " << endl;			
		for ( i = l.begin() ; i!=l.end() ; ++i){
			cout << (*i).type << " | " << (*i).time << " | "<< (*i).demand << endl;		
		}
		*/

		++step_id;		
		cout << "demands served: " << demandsServedCounter << " demands unserved: " << demandsUnservedCounter << " hd served: " << hdServedCounter << "  hd unserved: " << hdUnservedCounter << endl;
		cout << "% demands served: " << ((double)demandsServedCounter/((double)demandsServedCounter+(double)demandsUnservedCounter))*100 << "%" << endl;
		cout << "% hd served: " << ((double)hdServedCounter/((double)hdServedCounter+(double)hdUnservedCounter))*100 << "%" << endl;
	}
}

