#pragma once

class FileWriter {
private:
	int numOfFiles;
	vector<string> fileName;
	bool isEnabled;

public:
	FileWriter();
	~FileWriter(void) {};

	void SetStatus(bool isEnabled);

	void InitFiles(int numOfFiles, string outFileName);
	void ClearFile(int fileNum);
	void Add(int fileNum, string text);
	void Add(int fileNum, double num);
	void Add(int fileNum, int num);
};

