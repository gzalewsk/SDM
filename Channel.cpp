#include "StdAfx.h"
#include "Channel.h"

//-----------------------------------------------------------------------------------------
Channel::Channel() {

	beginSlice = -1; endSlice = -1; size = 0;
}

Channel::Channel(int begin, int end, int size) {
	
	if(begin <= end && size > 0) {
		beginSlice = begin; endSlice = end; this->size = size;
	} else {
		beginSlice = -1; endSlice = -1; size = 0;
	}
}

Channel::~Channel() {}

bool Channel::operator==(Channel &ch) {

	if(size == ch.size && beginSlice == ch.beginSlice && endSlice == ch.endSlice)
		return true;
	else
		return false;
}

bool Channel::doesCoincide(Channel &ch) {

	int S1lps, s2, e1, e2;

	if(beginSlice == ch.beginSlice || endSlice == ch.endSlice)
		return true;

	if(beginSlice < ch.beginSlice) {
		S1lps = beginSlice;
		e1 = endSlice;
		s2 = ch.beginSlice;
		e2 = ch.endSlice;
	} else {
		S1lps = ch.beginSlice;
		e1 = ch.endSlice;
		s2 = beginSlice;
		e2 = endSlice;
	}

	if(e1 >= s2)
		return true;

	return false;
}

void Channel::display() {
	
	//cout << "[" << beginSlice << ", " << endSlice << "], slices: " << size << endl;
	cout << "[" << beginSlice << "..." << endSlice << "] of " << size << " slices" << endl;
}