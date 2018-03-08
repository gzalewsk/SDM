#pragma once

#include "AlgorithmParameters.h"
#include "SimulationParameters.h"
#include "Network.h"
#include "FrequencyGrids.h"
#include "Traffic.h"


class SimulationData {

public:
	AlgorithmParameters algPar;
	SimulationParameters simPar;
	Network net;
	FlexGrid flexGrid;
	Traffic traffic;

	SimulationData(void) {};
	SimulationData(AlgorithmParameters algPar, SimulationParameters simPar, Network net, FlexGrid flexGrid, Traffic traffic) {
		this->algPar = algPar;
		this->simPar = simPar;
		this->net = net;
		this->flexGrid = flexGrid;
		this->traffic = traffic;
	};
	~SimulationData(void) {};

	void SetFlexGrid(FlexGrid flexGrid) {
		this->flexGrid = flexGrid;
	};
};

