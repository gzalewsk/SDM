#pragma once

#include "Route.h"
#include "SimulationParameters.h"
#include "SupportingFunctions.h"

//class SimulationParameters;
//class SupportingFunctions;
class NetworkScenario;

using namespace std;

class TranmissionConfig {
public:
	int maxBitRate;
	vector<int> transceivers;
	vector<int> slices;

	TranmissionConfig() {};
	TranmissionConfig(int maxBitRate, vector<int> transceivers, vector<int> slices) {
		this->maxBitRate = maxBitRate;
		this->transceivers = transceivers;
		this->slices = slices;
	};
	~TranmissionConfig() {};
};

class QoTmodel {
public:
	int maxTrDistinance;
	double spectralEfficiency;
	vector<TranmissionConfig> transConf;
	int bitRateGranularity;
	int minModLevel;
	int maxModLevel;

	SimulationParameters simPar;
	SupportingFunctions supFun;
	NetworkScenario *netScen;

	QoTmodel() {};
	QoTmodel(SimulationParameters &simPar, SupportingFunctions &supFun, NetworkScenario *netScen);
	~QoTmodel(void) {};

	int ConvertBitrateToSlices(double bitrate, double pathLength);
	int getMaxTransmissionDistance();
};