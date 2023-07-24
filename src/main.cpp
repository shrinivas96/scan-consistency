#define _USE_MATH_DEFINES

#include <cmath>
#include <vector>
#include <fstream>
#include <iostream>
#include <consist/visibility.h>
#include <g2o/types/slam2d/se2.h>
#include <boost/algorithm/string.hpp>
#include <g2o/types/data/raw_laser.h>
#include <g2o/types/data/laser_parameters.h>


// function to return the range values as a single vector
std::vector<std::vector<double>> scan_as_vector()
{
    // config to read file and get scans out
    // std::string filePath = "../data/scan_sim_real.dat";                 // 1st line sim scan, 2nd line actual scan
    std::string filePath = "../data/scans_6444_70.dat";                 // both lines are actual scans at different time stamps
    std::ifstream infile(filePath);
    if (!infile)
    {
        std::cerr << "Error opening file." << std::endl;
        exit(-1);
    }

    // store the two lines of scans as vectors
    std::vector<std::vector<double>> scans; // include inf values

    // to replace inf values with 0.0
    // to check if that was causing the problem in polyOcclusions()
    const double kZero = 0.0;

    std::string line;
    while (std::getline(infile, line))
    {
        std::vector<double> row;
        std::istringstream iss(line);
        std::string value;

        while (std::getline(iss, value, ','))
        {
            boost::trim(value);         // value always contained a leading space
            if (value == "inf")
            {
                // uncomment one of the two lines depending on if you want to include infinity values
                // row.emplace_back(std::numeric_limits<double>::infinity());
                row.emplace_back(kZero);
            }
            else
            {
                row.emplace_back(std::stod(value));
            }
        }

        scans.emplace_back(row);
    }
    infile.close();

    return scans;
}

int main()
{
    std::vector<std::vector<double>> scans = scan_as_vector();

    // some scan parameters
    int kNumBeams = 360;
    double angle_min = 0.0;
    double range_min = 0.11999999731779099;
    double range_max = 3.5;
    double angle_increment = 0.017501922324299812;
    double x = 6.297623001392478;
    double y = 0.5992175406224158;
    double theta = 1.5355101;

    // pose variable
    g2o::SE2 robPose(x, y, theta);

    // scan parameters varaible
    g2o::LaserParameters scanParams(kNumBeams, angle_min, angle_increment, range_max, range_min);
    scanParams.fov = 2*M_PI;
    scanParams.laserPose = robPose;
    
    // raw laser variable
    g2o::RawLaser realScan, simScan;
    realScan.setRanges(scans[1]);
    realScan.setLaserParams(scanParams);
    simScan.setRanges(scans[0]);
    simScan.setLaserParams(scanParams);

    // std::cout << "Beam: " << realScan.laserParams().firstBeamAngle << std::endl;

    consist::Visibility polyReal(&realScan, robPose, range_max);
    consist::Visibility polySim(&simScan, robPose, range_max);

    std::vector<double> realInconsistDist, simInconsistDist;
    realInconsistDist = polyReal.polyOcclusions(polySim);           // distances of when real is inside sim
    simInconsistDist = polySim.polyOcclusions(polyReal);           // distances of when sim is inside real

    double inconsistMeasure = 0;                    // the inconsistency measure is the sum of both arrays
    for(double& elem:realInconsistDist)
        inconsistMeasure += elem;
    for(double& elem:simInconsistDist)
        inconsistMeasure += elem;
    
    std::cout << "The inconsistency measure is: " << inconsistMeasure << std::endl;

    return 0;
}