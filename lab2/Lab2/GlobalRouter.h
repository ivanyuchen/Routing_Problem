#pragma once

#include "ispdData.h"
#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <chrono>

class GlobalRouter {
public:
    GlobalRouter(ISPDParser::ispdData* data);
    ~GlobalRouter();

    void route();

private:
    ISPDParser::ispdData* ispdData;
    int numX, numY;
    
    std::chrono::steady_clock::time_point startTime;
    double timeLimitSeconds = 1600.0; // Increased to ~26 minutes

    // 2D Capacity and Demand
    std::vector<std::vector<double>> horCapacity;
    std::vector<std::vector<double>> verCapacity;
    std::vector<std::vector<double>> horDemand;
    std::vector<std::vector<double>> verDemand;
    std::vector<std::vector<double>> horHistory;
    std::vector<std::vector<double>> verHistory;

    // Reuse vectors for A* to avoid frequent allocations
    std::vector<double> dist;
    std::vector<int> parent;
    std::vector<bool> horParent;
    std::vector<int> visitTag;
    int currentTag = 0;

    // Best solution tracking
    struct BestSolution {
        std::vector<std::vector<ISPDParser::RPoint>> paths;
        double tof = 1e18;
        double wl = 1e18;
    } bestSol;

    // Parameters for cost function
    double k_factor = 1.0;     // Start smaller
    double p_exponent = 2.0;   // Less aggressive initial exponent
    double h_factor = 0.01;    // Small initial history impact

    void initializeGrid();
    void decomposeNets();
    void routeTwoPinNet(ISPDParser::TwoPin& twoPin);
    double getEdgeCost(int x, int y, bool hori);
    void routeTwoPinNetMaze(ISPDParser::TwoPin& tp);
    void updateDemand(const ISPDParser::TwoPin& twoPin, double delta);
    
    void saveBestSolution();
    void restoreBestSolution();
    double calculateTOF();
    double calculateWL();
};
