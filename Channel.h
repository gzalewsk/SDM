#pragma once

class Channel {
public:
	int size;			// in slices
	int beginSlice;
	int endSlice;

	Channel();
	Channel(int begin, int end, int size);
	virtual ~Channel();

	bool operator==(Channel &ch);
	bool doesCoincide(Channel &ch);
	void display();
};
