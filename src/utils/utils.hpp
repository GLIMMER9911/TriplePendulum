#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include <fstream>
#include <vector>

// 
#include <chrono>
//
#include <iomanip>

// 
#include <sstream>

// Open the file which name is "data+time"
void openFile();

// Save data to .txt file
void saveDataTofile(std::vector<double>& data, const std::string& dataName, bool closeFlag);

#endif // !UTILS_H
