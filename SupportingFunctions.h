#pragma once

class SupportingFunctions {
public:
	SupportingFunctions(void) {};
	~SupportingFunctions(void) {};

	int Round(double value);
	int fact(int n);
	double lin2dB(double value);
	double dB2lin(double value);
	void Tokenize(const string& str, vector<string>& tokens, const string& delimiters = " ");	

	void displayVector(vector<int> vect);
	void displayVector(vector<double> vect);
};
