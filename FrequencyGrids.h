#pragma once

#include <list>
#include "Channel.h"

class FlexGrid {
public:
	double spectrum;
	double sliceWidth;
	int slices;

	vector<vector<Channel>> chs;

	FlexGrid() {};
	FlexGrid(double spectrum, double sliceWidth);
	~FlexGrid(void) {};

	void UpdateNumberOfSlices(int newNumOfSlices);
	void UpdateSpectrum(double spectrum);

	void GenerateChannels();
	void DeleteChannels();
	void DisplayChannels();
	
	double ConvertSliceToSpectrum(int slices);
};

