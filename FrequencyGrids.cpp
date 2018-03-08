#include "StdAfx.h"
#include "FrequencyGrids.h"

FlexGrid::FlexGrid(double spectrum, double sliceWidth) {

	this->spectrum = spectrum;
	this->sliceWidth = sliceWidth;
	slices = floor((double) spectrum / sliceWidth);

	//chs = vector<vector<Channel>>();
	GenerateChannels();
}

void FlexGrid::UpdateNumberOfSlices(int newNumOfSlices) {

	DeleteChannels();

	slices = newNumOfSlices;
	spectrum = slices * sliceWidth;

	GenerateChannels();

	cout << "- Spectrum updated: " << spectrum << "GHz, " << slices << " slices" << endl;
	cout << "----------------------------------------" << endl;	
}

void FlexGrid::UpdateSpectrum(double spectrum) {

	DeleteChannels();

	this->spectrum = spectrum;
	slices = floor((double) spectrum / sliceWidth);
	
	GenerateChannels();

	cout << "- Spectrum updated: " << spectrum << "GHz, " << slices << " slices" << endl;
	cout << "----------------------------------------" << endl;
}

void FlexGrid::GenerateChannels() {

	int j, n, s;

	DeleteChannels();

	chs = vector<vector<Channel>>(slices + 1);
	Channel ch;

	//cout << "--- " << slices << " --- " << chs.size() << " --- " << chs[32].size() << " --- " << endl;

	for(n = 1; n <= slices; n++) {
		for(j = 0; j < slices; j++) {
			if(j + 1 >= n) {				
				//ch = ChannelPtr(new Channel(slices));
				//ch->size = n;
				//for(s = j + 1 - ns; s < j + 1; s++)
				//	ch->sliceAdj[s] = 1;
				//chs[n].add(ch);
				ch = Channel(Channel(j + 1 - n, j, n));
				chs[n].push_back(ch);
			}
		}
	}

	//cout << "--- " << slices << " --- " << chs.size() << " --- " << chs[32].size() << " --- " << endl;
}

void FlexGrid::DeleteChannels(void) {

	if(chs.size() > 0)
		for(int n = 1; n <= slices; n++)
		//for(int n = 1; n < chs.size(); n++)
			chs[n].clear();
	chs.clear();
	
	chs = vector<vector<Channel>>();
}

void FlexGrid::DisplayChannels() {
	
	cout << "- Displaying channels" << endl;

	for (int n = 1; n <= slices; n++) {
		cout << "  - Channel width = " << n << " slices" << endl;
		for(int j = 0; j < chs[n].size(); j++) {
			cout << "    Element " << j << ": ";
			chs[n][j].display();
		}
	}
}

double FlexGrid::ConvertSliceToSpectrum(int numOfSlices) {

	return numOfSlices * sliceWidth;
}