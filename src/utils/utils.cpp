#include "utils.hpp"

std::string fileName;
std::ofstream outputFile;

// Open the file which name is "data+time"
void openFile() {

	// get current time
	auto currentTime = std::chrono::system_clock::now();
	std::time_t time = std::chrono::system_clock::to_time_t(currentTime);

	// Format the time as a string to be used as a file name
	std::stringstream ss;
	ss << std::put_time(std::localtime(&time), "%Y-%m-%d_%H-%M-%S");
	fileName = ss.str() + ".txt";

	outputFile.open(fileName, std::ios::app);
	if (!outputFile.is_open()) {
		std::cerr << "creat oyutputFile FAILED! " << std::endl;
	}


}

void saveDataTofile(std::vector<double>& data, const std::string& dataName, bool closeFlag ) {
	static unsigned int iteration = 1;
	
	// open outputFile
	outputFile.open(fileName, std::ios::app);

	// Check whether the file is opened
	if (outputFile.is_open()) {
		std::string str = dataName;
		outputFile << iteration << " " << dataName << ": " ;
		for (int i = 0; i < data.size(); i++) {
			outputFile << data.at(i) << "  ";
		}
	}
	else {
		std::cerr << " Can not open the output File£¡" << std::endl;
	}

	if (closeFlag) {
		iteration += 1;
	}
	outputFile << " " << std::endl;
	outputFile.close();

}



