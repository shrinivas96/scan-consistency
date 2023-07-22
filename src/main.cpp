#include <vector>
#include <fstream>
#include <iostream>
#include <consist/visibility.h>
#include <g2o/types/slam2d/se2.h>
#include <boost/algorithm/string.hpp>
#include <g2o/types/data/raw_laser.h>

std::vector<std::vector<double>> scan_as_vector()
{
    // config to read file and get scans out
    std::string filePath = "../data/scan_sim_real.dat";
    std::ifstream infile(filePath);
    if (!infile)
    {
        std::cerr << "Error opening file." << std::endl;
        exit(-1);
    }

    // store the two lines of scans as vectors
    std::vector<std::vector<double>> scansInf; // include inf values
    std::vector<std::vector<double>> scans;    // discard inf values

    std::string line;
    while (std::getline(infile, line))
    {
        std::vector<double> row;
        std::vector<double> rowOI; // ohne inf
        std::istringstream iss(line);
        std::string value;

        while (std::getline(iss, value, ','))
        {
            boost::trim(value);
            if (value == "inf")
            {
                row.emplace_back(std::numeric_limits<double>::infinity());
            }
            else
            {
                row.emplace_back(std::stod(value));
                rowOI.emplace_back(std::stod(value));
            }
        }

        scansInf.emplace_back(row);
        scans.emplace_back(rowOI);
    }
    infile.close();

    return scansInf;
}

int main()
{
    std::vector<std::vector<double>> scans = scan_as_vector();

    g2o::RawLaser realScan, simScan;
    realScan.setRanges(scans[1]);
    simScan.setRanges(scans[0]);

    double range_min = 0.11999999731779099, range_max = 3.5;
    double x = 6.297623001392478;
    double y = 0.5992175406224158;
    double theta = 1.5355101;

    g2o::SE2 robPose(x, y, theta);

    consist::Visibility polyReal(&realScan, robPose, range_max);
    consist::Visibility polySim(&simScan, robPose, range_max);
    std::cout << "Compiled!!" << std::endl;

    return 0;
}