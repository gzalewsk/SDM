#pragma once

#include "Definitions.h"

class SimulationParameters {
public:
	bool detailsDisplayed;

	string problemName;
	int problemCase;
	int algorithm;
	int algOption;
	int config;
	string dataDir;

	string scenFileName;
	string algFileName;

	int numOfTrafficSets;
	vector<string> demandSet;

	NetworkTopology network;
	bool hasDirectedLinks;
	bool areRoutesUnique;
	bool isLinkOrderMaintained;
	int kSPlimit;
	double bigNumber;

	double spectrum;
	double sliceWidth;

	int linkLengthMultiplier;

	TransmissionEONmodel QoTmodelID;

	int modes;
	bool isXTconsidered;			// is crosstalk considered?
	XTestimationType XTtype;
	double hXTkm;
	double XTlimit;

	SimulationParameters() {
		problemName = "RSSA_UNCAP";

		hasDirectedLinks = true;
		//hasDirectedLinks = false;

		//isLinkOrderMaintained = true;		// link indexes in path->linkVector maintain their order on the path
		isLinkOrderMaintained = false;		// link indexes in path->linkVector in increasing order

		linkLengthMultiplier = 1;

		//spectrum = 30000;
		spectrum = 250;
		sliceWidth = 12.5;					// generic flexGrid of 12.5GHz granularity

		QoTmodelID = JLT2016;
	};

	~SimulationParameters(void) {};
};

