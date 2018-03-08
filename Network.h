#pragma once

class Network {
public:
	int nodes;
	int links;
	vector<int> linkLength;
	vector<vector<int>> nodeLinks;
	vector<vector<int>> neigh;
	vector<vector<int>> dijkstraNeigh;
	vector<vector<int>> linkIndex;
	int modes;
	vector<vector<int>> neighbourModes;

	Network() {};
	Network(int nodes, int links, int modes, vector<int> &linkLength, vector<vector<int>> &nodeLinks, vector<vector<int>> &neigh, vector<vector<int>> &dijkstraNeigh, vector<vector<int>> &linkIndex) {
		this->nodes = nodes;
		this->links = links;
		this->linkLength = linkLength;
		this->nodeLinks = nodeLinks;
		this->neigh = neigh;
		this->dijkstraNeigh = dijkstraNeigh;
		this->linkIndex = linkIndex;

		setModes(modes);
	};

	~Network() {};

	void setModes(int modes) {

		if (modes != 1 && modes != 2 && modes != 3 && modes != 7 && modes != 19) {
			cout << "Warning: " << modes << " modes not supported!";
			getchar();
			exit(0);
		}
		
		this->modes = modes;
		neighbourModes = vector<vector<int>>(modes);
		for (int m = 0; m < neighbourModes.size(); m++) {
			neighbourModes[m] = vector<int>();

			//cout << "Mode " << m << ": ";
			//for (int i = 0; i < neighbourModes[m].size(); i++)
			//	cout << neighbourModes[m][i] << " ";
			//cout << endl;
		}

		if (modes == 2) {
			neighbourModes[0].push_back(1);
			neighbourModes[1].push_back(0);
			// core (inaczej "modes") layout:
			//				  0   1	
			//
		}
		else if (modes == 3) {
			neighbourModes[0].push_back(1); neighbourModes[0].push_back(2);
			neighbourModes[1].push_back(0); neighbourModes[1].push_back(2);
			neighbourModes[2].push_back(0); neighbourModes[2].push_back(1);
			// core layout:
			//					0	
			//				  2   1	
			//
		}
		else if (modes == 7) {
			neighbourModes[0].push_back(1); neighbourModes[0].push_back(5); neighbourModes[0].push_back(6);
			neighbourModes[1].push_back(2); neighbourModes[1].push_back(0); neighbourModes[1].push_back(6);
			neighbourModes[2].push_back(3); neighbourModes[2].push_back(1); neighbourModes[2].push_back(6);
			neighbourModes[3].push_back(4); neighbourModes[3].push_back(2); neighbourModes[3].push_back(6);
			neighbourModes[4].push_back(5); neighbourModes[4].push_back(3); neighbourModes[4].push_back(6);
			neighbourModes[5].push_back(0); neighbourModes[5].push_back(4); neighbourModes[5].push_back(6);
			neighbourModes[6].push_back(0); neighbourModes[6].push_back(1); neighbourModes[6].push_back(2);
			neighbourModes[6].push_back(3); neighbourModes[6].push_back(4); neighbourModes[6].push_back(5);
			// core layout:
			//				  0   1 	
			//			    5   6   2	
			//				  4   3 	
		}
		else if (modes == 19) {
			neighbourModes[0].push_back(1); neighbourModes[0].push_back(11); neighbourModes[0].push_back(12);
			neighbourModes[1].push_back(0); neighbourModes[1].push_back(2); neighbourModes[1].push_back(12); neighbourModes[1].push_back(13);
			neighbourModes[2].push_back(1); neighbourModes[2].push_back(3); neighbourModes[2].push_back(13);
			neighbourModes[3].push_back(2); neighbourModes[3].push_back(4); neighbourModes[3].push_back(13); neighbourModes[1].push_back(14);
			neighbourModes[4].push_back(3); neighbourModes[4].push_back(5); neighbourModes[4].push_back(14);
			neighbourModes[5].push_back(4); neighbourModes[5].push_back(6); neighbourModes[5].push_back(14); neighbourModes[5].push_back(15);
			neighbourModes[6].push_back(5); neighbourModes[6].push_back(7); neighbourModes[6].push_back(15);
			neighbourModes[7].push_back(6); neighbourModes[7].push_back(8); neighbourModes[7].push_back(15); neighbourModes[7].push_back(16);
			neighbourModes[8].push_back(7); neighbourModes[8].push_back(9); neighbourModes[8].push_back(16);
			neighbourModes[9].push_back(8); neighbourModes[9].push_back(10); neighbourModes[9].push_back(16); neighbourModes[9].push_back(17);
			neighbourModes[10].push_back(9); neighbourModes[10].push_back(11); neighbourModes[10].push_back(17);
			neighbourModes[11].push_back(10); neighbourModes[11].push_back(12); neighbourModes[11].push_back(17); neighbourModes[11].push_back(0);
			neighbourModes[12].push_back(0); neighbourModes[12].push_back(1); neighbourModes[12].push_back(11);
			neighbourModes[12].push_back(13); neighbourModes[12].push_back(17); neighbourModes[12].push_back(18);
			neighbourModes[13].push_back(1); neighbourModes[13].push_back(2); neighbourModes[13].push_back(3);
			neighbourModes[13].push_back(12); neighbourModes[13].push_back(14); neighbourModes[13].push_back(18);
			// core layout:
			//				  0   1   2
			//			    11  12  13  3
			//			  10  17  18  14  4
			//			    9   16  15  5
			//				  8   7   6
			neighbourModes[14].push_back(3); neighbourModes[14].push_back(4); neighbourModes[14].push_back(5);
			neighbourModes[14].push_back(13); neighbourModes[14].push_back(15); neighbourModes[14].push_back(18);
			neighbourModes[15].push_back(5); neighbourModes[15].push_back(6); neighbourModes[15].push_back(7);
			neighbourModes[15].push_back(14); neighbourModes[15].push_back(16); neighbourModes[15].push_back(18);
			neighbourModes[16].push_back(7); neighbourModes[16].push_back(8); neighbourModes[16].push_back(9);
			neighbourModes[16].push_back(15); neighbourModes[16].push_back(17); neighbourModes[16].push_back(18);
			neighbourModes[17].push_back(9); neighbourModes[17].push_back(10); neighbourModes[17].push_back(11);
			neighbourModes[17].push_back(12); neighbourModes[17].push_back(16); neighbourModes[17].push_back(18);
			neighbourModes[18].push_back(12); neighbourModes[18].push_back(13); neighbourModes[18].push_back(14);
			neighbourModes[18].push_back(15); neighbourModes[18].push_back(16); neighbourModes[18].push_back(17);
		}
	}
};
