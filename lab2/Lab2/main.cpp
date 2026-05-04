#include "ispdData.h"
#include "LayerAssignment.h"
#include "GlobalRouter.h"

#include <iostream>
#include <fstream>
#include <algorithm>
#include <utility>
#include <string>
#include <cassert>

int main(int argc, char **argv) {
    if (argc < 3) {
        std::cerr << "Usage: ./router <inputFile> <outputFile>" << std::endl;
        return 1;
    }
    std::ifstream fp(argv[1]);
    if (!fp.is_open()) {
        std::cerr << "Failed to open input file: " << argv[1] << std::endl;
        return 1;
    }
    ISPDParser::ispdData *ispdData = ISPDParser::parse(fp);
    fp.close();

    // Filtering and Grid Mapping
    ispdData->nets.erase(std::remove_if(ispdData->nets.begin(), ispdData->nets.end(), [&](ISPDParser::Net *net) {
        for (auto &pin : net->pins) {
            int x = (std::get<0>(pin) - ispdData->lowerLeftX) / ispdData->tileWidth;
            int y = (std::get<1>(pin) - ispdData->lowerLeftY) / ispdData->tileHeight;
            int z = std::get<2>(pin) - 1;
            if (std::any_of(net->pin3D.begin(), net->pin3D.end(), [x, y, z](const auto &p) { return p.x == x && p.y == y && p.z == z; })) continue;
            net->pin3D.emplace_back(x, y, z);
            if (std::any_of(net->pin2D.begin(), net->pin2D.end(), [x, y](const auto &p) { return p.x == x && p.y == y; })) continue;
            net->pin2D.emplace_back(x, y);
        }
        return net->pin3D.size() > 1000 || net->pin2D.size() <= 1;
    }), ispdData->nets.end());
    ispdData->numNet = ispdData->nets.size();

    // Global Routing
    std::cout << "Starting Global Routing..." << std::endl;
    GlobalRouter router(ispdData);
    router.route();

    // Layer Assignment
    std::cout << "Starting Layer Assignment..." << std::endl;
    LayerAssignment::Graph graph;
    graph.initialLA(*ispdData, 1);
    graph.convertGRtoLA(*ispdData, true);
    graph.COLA(true);

    // Final Output
    graph.output3Dresult(argv[2]);

    delete ispdData;
    return 0;
}
