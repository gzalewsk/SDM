#include "StdAfx.h"
#include "FileWriter.h"
#include <sstream>
#include <fstream>

FileWriter::FileWriter() {

	isEnabled = true;
}

void FileWriter::SetStatus(bool isEnabled) {

	this->isEnabled = isEnabled;
}

void FileWriter::InitFiles(int numOfFiles, string outFileName) {

	if (isEnabled) {
		if (numOfFiles >= 0 && numOfFiles < 10) {

			this->numOfFiles = numOfFiles;
			fileName = vector<string>(numOfFiles);

			for (int i = 0; i < numOfFiles; i++) {
				ostringstream name;
				name << "Out" << i << "_" << outFileName << ".txt";
				//name << outFileName << ".txt"; 
				fileName[i] = name.str();

				ClearFile(i);
			}

		}
		else {
			cout << "Warning: wrong number of files" << endl;
			system("pause");
			exit(0);
		}
	}
}

void FileWriter::ClearFile(int fileNum) {

	if (isEnabled) {
		if (fileNum >= 0 && fileNum < numOfFiles) {
			ofstream file;
			file.open(fileName[fileNum]);
			file.close();
		}
	}
}

void FileWriter::Add(int fileNum, string text) {

	if (isEnabled) {
		if (fileNum >= 0 && fileNum < numOfFiles) {
			ofstream file;
			file.open(fileName[fileNum], ios::app);
			file << text;
			file.close();
		}
	}
}

void FileWriter::Add(int fileNum, double num) {

	if (isEnabled) {
		if (fileNum >= 0 && fileNum < numOfFiles) {
			ofstream file;
			file.open(fileName[fileNum], ios::app);
			file << num;
			file.close();
		}
	}
}


void FileWriter::Add(int fileNum, int num) {

	if (isEnabled) {
		if (fileNum >= 0 && fileNum < numOfFiles) {
			ofstream file;
			file.open(fileName[fileNum], ios::app);
			file << num;
			file.close();
		}
	}
}