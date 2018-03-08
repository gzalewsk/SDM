#include "StdAfx.h"
#include "QoTmodel.h"

#include "SimulationParameters.h"
#include "SupportingFunctions.h"
#include "NetworkScenario.h"

QoTmodel::QoTmodel(SimulationParameters &simPar, SupportingFunctions &supFun, NetworkScenario *netScen) {

	this->simPar = simPar;
	this->supFun = supFun;
	this->netScen = netScen;

	if(simPar.QoTmodelID == JLT2016) {
		maxTrDistinance = 6300;
	} else {
		cout << endl << "Warning: other QoT models not supported" << endl << endl;
		exit(0);
	}
}

int QoTmodel::ConvertBitrateToSlices(double bitrate, double pathLength) {

	int slices;

	if (simPar.QoTmodelID == JLT2016) {

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

		slices = ceil(spectrum / netScen->flexGrid.sliceWidth);
		cout << slices << "- slices; " << bitrate << "- bitrate; " << pathLength << "- path length; " << spectrum << "- spectrum; " << netScen->flexGrid.sliceWidth << "- sliceWidth; " << endl;
		//cout << "bit-rate = " << bitrate << ", pathLength = " << pathLength << " -> opticalCarriers = " << opticalCarriers << ", spectrum = " << spectrum << ", slices = " << slices << endl;

	} else {

		cout << "Warning: other QoT models not supported" << endl;
		system("pause");
		exit(0);
	}

	return slices;
}

int QoTmodel::getMaxTransmissionDistance() {

	if(simPar.QoTmodelID == JLT2016)
		return maxTrDistinance;
	else {
		cout << "Warning: other QoT models not supported" << endl;
		system("pause");
		exit(0);
	}
}
