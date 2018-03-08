#include "StdAfx.h"
#include "SupportingFunctions.h"

int SupportingFunctions::Round(double value) { 	return (int) (0.5 + value); }

int SupportingFunctions::fact(int n) { return (n == 1 || n == 0) ? 1 : fact(n - 1) * n; }

double SupportingFunctions::lin2dB(double value) { return 10 * log10(value); }

double SupportingFunctions::dB2lin(double value) { return pow(10, value / 10.0); }

void SupportingFunctions::Tokenize(const string& str, vector<string>& tokens, const string& delimiters) {

    // Skip delimiters at beginning.
    string::size_type lastPos = str.find_first_not_of(delimiters, 0);
    // Find first "non-delimiter".
    string::size_type pos = str.find_first_of(delimiters, lastPos);

    while (string::npos != pos || string::npos != lastPos) {
		// Found a token, add it to the vector.
        tokens.push_back(str.substr(lastPos, pos - lastPos));
        // Skip delimiters.  Note the "not_of"
        lastPos = str.find_first_not_of(delimiters, pos);
        // Find next "non-delimiter"
        pos = str.find_first_of(delimiters, lastPos);
    }
}

void SupportingFunctions::displayVector(vector<int> vect) {

	for(int i = 0; i < vect.size(); i++)
		cout << vect[i] << " ";
}

void SupportingFunctions::displayVector(vector<double> vect) {

	for(int i = 0; i < vect.size(); i++)
		cout << vect[i] << " ";
}
