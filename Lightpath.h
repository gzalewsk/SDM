#pragma once

#include "StdAfx.h"
#include "Route.h"
#include "Channel.h"

class Lightpath {
public:
	int d;
	Route rt;
	int mode;
	Channel ch;

	Lightpath(void) {
		this->d = -1;
	};
	Lightpath(int d, Route rt, int mode, Channel ch) {
		this->d = d, this->rt = rt, this->mode = mode, this->ch = ch;
	};
	~Lightpath(void) {};

	void display() {
		if (d == -1) {
			cout << endl << "Warning: the lightpath is not defined" << endl;
		}
		else {
			cout << endl << "\tThe lightpath of demand " << d << ":" << endl;
			cout << "\t\tchannel: ";
			ch.display();
			cout << "\t\troute: ";
			rt.display();
		}
	};
};

class LightpathNew {
public:
	int d;
	Route rt;
	vector<int> modeVector;
	Channel ch;

	LightpathNew(void) {
		this->d = -1;
	};
	LightpathNew(int d, Route rt, vector<int> modeVector, Channel ch) {
		this->d = d, this->rt = rt, this->modeVector = modeVector, this->ch = ch;
	};
	~LightpathNew(void) {};

	void display() {
		if (d == -1) {
			cout << endl << "Warning: the lightpath is not defined" << endl;
		}
		else {
			cout << endl << "\tThe lightpath of demand " << d << ":" << endl;
			cout << "\t\tchannel: ";
			ch.display();
			cout << "\t\troute: ";
			rt.display();
		}
	};
};