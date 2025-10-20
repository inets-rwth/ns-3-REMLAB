#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include "ns3/core-module.h"
#include <ns3/ornstein-uhlenbeck-process.h>

using namespace ns3;
using namespace std;

Ptr<OrnsteinUhlenbeckErrorModel> latitudeErrorModel
    = CreateObject<OrnsteinUhlenbeckErrorModel>(6.2, 1.5, 3.8);
Ptr<OrnsteinUhlenbeckErrorModel> longitudeErrorModel
    = CreateObject<OrnsteinUhlenbeckErrorModel>(3.9, 2.4, 4.2);

// Function to read a CSV file and apply an offset to x values
void ReadCsvAndApplyOffset(std::string filename, double offsetX) {
    std::ifstream file(filename);
    std::string line;
    std::vector<std::tuple<double, double, double>> positions;

    // Read and ignore the header line
    std::getline(file, line);

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string xStr, yStr, zStr;
        double x, y, z;

        // Read x, y, z values from CSV
        if (std::getline(ss, xStr, ',') &&
            std::getline(ss, yStr, ',') &&
            std::getline(ss, zStr, ',')) {
            
            // Convert strings to double
            x = std::stod(xStr);
            y = std::stod(yStr);
            z = std::stod(zStr);

            // Apply the offset to x value
            double xErr = latitudeErrorModel->GetNextValue();
            double yErr = longitudeErrorModel->GetNextValue();
            std::cout << xErr << "," << yErr << std::endl;
            x += xErr;
            y += yErr;

            // Store the modified coordinates
            positions.push_back(std::make_tuple(x, y, z));
        }
    }

    file.close();

    // Print modified coordinates
    std::cout << "Updated Coordinates:\n";
    for (const auto& pos : positions) {
        std::cout << std::get<0>(pos)
                  << "," << std::get<1>(pos)
                  << "," << std::get<2>(pos) << std::endl;
    }
}

int main(int argc, char *argv[]) {
    CommandLine cmd;
    cmd.Parse(argc, argv);

    std::string input_raytracing_folder = "src/nr/model/Raytracing_UE_set/2000_cords.txt";
    double offsetX = 10.0; // Fixed offset value for x

    ReadCsvAndApplyOffset(input_raytracing_folder, offsetX);

    return 0;
}
