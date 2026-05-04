#include "GlobalRouter.h"
#include <queue>
#include <numeric>
#include <iostream>
#include <algorithm>
#include <random>

GlobalRouter::GlobalRouter(ISPDParser::ispdData* data) : ispdData(data) {
    numX = data->numXGrid;
    numY = data->numYGrid;
    startTime = std::chrono::steady_clock::now();
    initializeGrid();
}

GlobalRouter::~GlobalRouter() {}

void GlobalRouter::initializeGrid() {
    horCapacity.assign(numX - 1, std::vector<double>(numY, 0));
    verCapacity.assign(numX, std::vector<double>(numY - 1, 0));
    horDemand.assign(numX - 1, std::vector<double>(numY, 0));
    verDemand.assign(numX, std::vector<double>(numY - 1, 0));
    horHistory.assign(numX - 1, std::vector<double>(numY, 0));
    verHistory.assign(numX, std::vector<double>(numY - 1, 0));

    dist.assign(numX * numY, 1e18);
    parent.assign(numX * numY, -1);
    horParent.assign(numX * numY, false);
    visitTag.assign(numX * numY, 0);
    currentTag = 0;

    std::vector<int> wireSize(ispdData->numLayer);
    for (int i = 0; i < ispdData->numLayer; i++) {
        wireSize[i] = ispdData->minimumWidth[i] + ispdData->minimumSpacing[i];
    }

    for (int i = 0; i < numX - 1; i++) {
        for (int j = 0; j < numY; j++) {
            for (int k = 0; k < ispdData->numLayer; k++) {
                horCapacity[i][j] += ispdData->horizontalCapacity[k] / (double)wireSize[k];
            }
        }
    }

    for (int i = 0; i < numX; i++) {
        for (int j = 0; j < numY - 1; j++) {
            for (int k = 0; k < ispdData->numLayer; k++) {
                verCapacity[i][j] += ispdData->verticalCapacity[k] / (double)wireSize[k];
            }
        }
    }

    for (auto* adj : ispdData->capacityAdjs) {
        int x1 = std::get<0>(adj->grid1), y1 = std::get<1>(adj->grid1), z1 = std::get<2>(adj->grid1) - 1;
        int x2 = std::get<0>(adj->grid2), y2 = std::get<1>(adj->grid2), z2 = std::get<2>(adj->grid2) - 1;
        bool isH = (y1 == y2);
        int x = std::min(x1, x2), y = std::min(y1, y2);
        int zSize = wireSize[z1];
        int reduced = (int)adj->reducedCapacityLevel / zSize;
        int original = (isH ? ispdData->horizontalCapacity[z1] : ispdData->verticalCapacity[z1]) / zSize;
        if (isH) horCapacity[x][y] -= (original - reduced);
        else verCapacity[x][y] -= (original - reduced);
    }
}

struct Edge {
    int u, v;
    double weight;
    bool operator>(const Edge& other) const { return weight > other.weight; }
};

void GlobalRouter::decomposeNets() {
    for (auto* net : ispdData->nets) {
        if (net->pin2D.size() <= 1) continue;
        int n = net->pin2D.size();
        std::vector<Edge> edges;
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                double d = std::abs(net->pin2D[i].x - net->pin2D[j].x) + std::abs(net->pin2D[i].y - net->pin2D[j].y);
                edges.push_back({i, j, d});
            }
        }
        std::sort(edges.begin(), edges.end(), [](const Edge& a, const Edge& b) { return a.weight < b.weight; });
        std::vector<int> p(n);
        std::iota(p.begin(), p.end(), 0);
        auto find = [&](auto self, int i) -> int { return (p[i] == i) ? i : (p[i] = self(self, p[i])); };
        auto unite = [&](int i, int j) {
            int rootI = find(find, i), rootJ = find(find, j);
            if (rootI != rootJ) { p[rootI] = rootJ; return true; }
            return false;
        };
        int count = 0;
        for (const auto& e : edges) {
            if (unite(e.u, e.v)) {
                ISPDParser::TwoPin tp;
                tp.from = net->pin2D[e.u]; tp.to = net->pin2D[e.v]; tp.parNet = net;
                net->twopin.push_back(tp);
                count++; if (count == n - 1) break;
            }
        }
    }
}

double GlobalRouter::getEdgeCost(int x, int y, bool hori) {
    double cap = hori ? horCapacity[x][y] : verCapacity[x][y];
    double dem = hori ? horDemand[x][y] : verDemand[x][y];
    double his = hori ? horHistory[x][y] : verHistory[x][y];
    
    if (cap <= 1e-6) {
        return 1000000.0 + dem * 10000.0; // Higher penalty for hard blocks
    }
    
    double congestion = dem / cap;
    double base_cost = 1.0;
    
    // PathFinder-like cost function: (base + history) * (1 + congestion_penalty)
    double history_term = 1.0 + his * h_factor;
    double congestion_term = 1.0;
    if (congestion >= 1.0) {
        congestion_term += k_factor * std::pow(congestion, p_exponent);
    } else if (congestion > 0.8) {
        // Soft congestion penalty for near-capacity edges
        congestion_term += k_factor * 0.1 * std::pow(congestion, p_exponent);
    }
    
    return base_cost * history_term * congestion_term;
}

void GlobalRouter::routeTwoPinNet(ISPDParser::TwoPin& tp) {
    int x1 = tp.from.x, y1 = tp.from.y, x2 = tp.to.x, y2 = tp.to.y;
    int dx = (x1 <= x2) ? 1 : -1, dy = (y1 <= y2) ? 1 : -1;
    int nx = std::abs(x1 - x2), ny = std::abs(y1 - y2);
    std::vector<std::vector<double>> dp_table(nx + 1, std::vector<double>(ny + 1, 1e18));
    std::vector<std::vector<bool>> fromLeft(nx + 1, std::vector<bool>(ny + 1, false));
    dp_table[0][0] = 0;
    for (int i = 0; i <= nx; i++) {
        for (int j = 0; j <= ny; j++) {
            if (i < nx) {
                int cx = x1 + i * dx;
                int edgeX = (dx == 1) ? cx : cx - 1;
                double cost = getEdgeCost(edgeX, y1 + j * dy, true);
                if (dp_table[i + 1][j] > dp_table[i][j] + cost) { dp_table[i + 1][j] = dp_table[i][j] + cost; fromLeft[i + 1][j] = true; }
            }
            if (j < ny) {
                int cy = y1 + j * dy;
                int edgeY = (dy == 1) ? cy : cy - 1;
                double cost = getEdgeCost(x1 + i * dx, edgeY, false);
                if (dp_table[i][j + 1] > dp_table[i][j] + cost) { dp_table[i][j + 1] = dp_table[i][j] + cost; fromLeft[i][j + 1] = false; }
            }
        }
    }
    tp.path.clear();
    int ci = nx, cj = ny;
    while (ci > 0 || cj > 0) {
        if (ci > 0 && (cj == 0 || fromLeft[ci][cj])) {
            int cx = x1 + ci * dx;
            int edgeX = (dx == 1) ? cx - 1 : cx;
            tp.path.push_back(ISPDParser::RPoint(edgeX, y1 + cj * dy, true));
            ci--;
        } else {
            int cy = y1 + cj * dy;
            int edgeY = (dy == 1) ? cy - 1 : cy;
            tp.path.push_back(ISPDParser::RPoint(x1 + ci * dx, edgeY, false));
            cj--;
        }
    }
    std::reverse(tp.path.begin(), tp.path.end());
}

struct MazeNode {
    int x, y;
    double g, f;
    bool operator>(const MazeNode& other) const { return f > other.f; }
};

void GlobalRouter::routeTwoPinNetMaze(ISPDParser::TwoPin& tp) {
    int x1 = tp.from.x, y1 = tp.from.y, x2 = tp.to.x, y2 = tp.to.y;
    if (x1 == x2 && y1 == y2) { tp.path.clear(); return; }

    currentTag++;
    auto getIdx = [&](int x, int y) { return y * numX + x; };
    auto getH = [&](int x, int y) { return (double)(std::abs(x - x2) + std::abs(y - y2)); };

    int startIdx = getIdx(x1, y1);
    dist[startIdx] = 0;
    visitTag[startIdx] = currentTag;
    parent[startIdx] = -1;

    std::priority_queue<MazeNode, std::vector<MazeNode>, std::greater<MazeNode>> pq;
    pq.push({x1, y1, 0, getH(x1, y1)});

    int dx[] = {1, -1, 0, 0};
    int dy[] = {0, 0, 1, -1};

    // Bounding box expansion strategy - Increased margin for better detours
    int margin = 20; 
    int minX = std::max(0, std::min(x1, x2) - margin);
    int maxX = std::min(numX - 1, std::max(x1, x2) + margin);
    int minY = std::max(0, std::min(y1, y2) - margin);
    int maxY = std::min(numY - 1, std::max(y1, y2) + margin);

    bool reached = false;
    while (!pq.empty()) {
        MazeNode curr = pq.top(); pq.pop();
        int currIdx = getIdx(curr.x, curr.y);
        if (curr.g > dist[currIdx]) continue;
        if (curr.x == x2 && curr.y == y2) { reached = true; break; }

        for (int i = 0; i < 4; ++i) {
            int nx = curr.x + dx[i];
            int ny = curr.y + dy[i];
            if (nx < minX || nx > maxX || ny < minY || ny > maxY) continue;

            bool hori = (dx[i] != 0);
            int edgeX = hori ? std::min(curr.x, nx) : nx;
            int edgeY = hori ? ny : std::min(curr.y, ny);
            
            double edgeCost = getEdgeCost(edgeX, edgeY, hori);
            double nextG = curr.g + edgeCost;
            int nIdx = getIdx(nx, ny);
            if (visitTag[nIdx] != currentTag || nextG < dist[nIdx]) {
                dist[nIdx] = nextG;
                visitTag[nIdx] = currentTag;
                parent[nIdx] = currIdx;
                horParent[nIdx] = hori;
                pq.push({nx, ny, nextG, nextG + getH(nx, ny)});
            }
        }
    }

    if (!reached) {
        routeTwoPinNet(tp);
        return;
    }

    tp.path.clear();
    int currIdx = getIdx(x2, y2);
    while (parent[currIdx] != -1) {
        int pIdx = parent[currIdx];
        int px = pIdx % numX;
        int py = pIdx / numX;
        int cx = currIdx % numX;
        int cy = currIdx / numX;
        bool hori = horParent[currIdx];
        int edgeX = hori ? std::min(px, cx) : cx;
        int edgeY = hori ? cy : std::min(py, cy);
        tp.path.push_back(ISPDParser::RPoint(edgeX, edgeY, hori));
        currIdx = pIdx;
    }
    std::reverse(tp.path.begin(), tp.path.end());
}

void GlobalRouter::updateDemand(const ISPDParser::TwoPin& tp, double delta) {
    for (const auto& rp : tp.path) {
        if (rp.hori) horDemand[rp.x][rp.y] += delta;
        else verDemand[rp.x][rp.y] += delta;
    }
}

double GlobalRouter::calculateTOF() {
    double tof = 0;
    for (int i = 0; i < numX - 1; i++) for (int j = 0; j < numY; j++) if (horDemand[i][j] > horCapacity[i][j]) tof += (horDemand[i][j] - horCapacity[i][j]);
    for (int i = 0; i < numX; i++) for (int j = 0; j < numY - 1; j++) if (verDemand[i][j] > verCapacity[i][j]) tof += (verDemand[i][j] - verCapacity[i][j]);
    return tof;
}

double GlobalRouter::calculateWL() {
    double wl = 0;
    for (auto* net : ispdData->nets) for (auto& tp : net->twopin) wl += tp.path.size();
    return wl;
}

void GlobalRouter::saveBestSolution() {
    bestSol.paths.clear();
    bestSol.tof = calculateTOF();
    bestSol.wl = calculateWL();
    for (auto* net : ispdData->nets) {
        for (auto& tp : net->twopin) {
            bestSol.paths.push_back(tp.path);
        }
    }
}

void GlobalRouter::restoreBestSolution() {
    int idx = 0;
    for (auto* net : ispdData->nets) {
        for (auto& tp : net->twopin) {
            tp.path = bestSol.paths[idx++];
        }
    }
}

void GlobalRouter::route() {
    decomposeNets();
    std::vector<ISPDParser::TwoPin*> allTwoPins;
    for (auto* net : ispdData->nets) for (auto& tp : net->twopin) allTwoPins.push_back(&tp);
    
    // Sort by HPWL to route smaller nets first in initial phase
    std::sort(allTwoPins.begin(), allTwoPins.end(), [](ISPDParser::TwoPin* a, ISPDParser::TwoPin* b) { return a->HPWL() < b->HPWL(); });

    std::cout << "  Initial monotonic routing..." << std::endl;
    for (auto* tp : allTwoPins) { routeTwoPinNet(*tp); updateDemand(*tp, 1.0); }
    saveBestSolution();

    std::cout << "  Rip-up and Reroute (A* maze routing)..." << std::endl;
    int iter = 0;
    
    while (true) {
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - startTime).count();
        if (elapsed > timeLimitSeconds) break;

        std::vector<ISPDParser::TwoPin*> targetNets;
        for (auto* tp : allTwoPins) {
            bool hasOverflow = false;
            for (const auto& rp : tp->path) {
                if (rp.hori) { if (horDemand[rp.x][rp.y] > horCapacity[rp.x][rp.y]) { hasOverflow = true; break; } }
                else { if (verDemand[rp.x][rp.y] > verCapacity[rp.x][rp.y]) { hasOverflow = true; break; } }
            }
            if (hasOverflow) {
                targetNets.push_back(tp);
            }
        }

        int overflowedNets = targetNets.size();
        if (overflowedNets == 0) break;

        // Shuffle target nets to avoid bias
        static std::random_device rd;
        static std::mt19937 g(rd());
        std::shuffle(targetNets.begin(), targetNets.end(), g);

        // Limit the number of nets to re-route per iteration if it's too many?
        // Actually routing all overflowed nets is standard RRR.
        for (auto* tp : targetNets) {
            updateDemand(*tp, -1.0);
            routeTwoPinNetMaze(*tp);
            updateDemand(*tp, 1.0);
        }
        
        // Update history more moderately
        for (int i = 0; i < numX - 1; i++) {
            for (int j = 0; j < numY; j++) {
                if (horDemand[i][j] > horCapacity[i][j]) horHistory[i][j] += 1.0;
            }
        }
        for (int i = 0; i < numX; i++) {
            for (int j = 0; j < numY - 1; j++) {
                if (verDemand[i][j] > verCapacity[i][j]) verHistory[i][j] += 1.0;
            }
        }

        double currentTOF = calculateTOF();
        double currentWL = calculateWL();
        if (currentTOF < bestSol.tof || (currentTOF == bestSol.tof && currentWL < bestSol.wl)) saveBestSolution();

        std::cout << "    Iter " << ++iter << ": TOF=" << currentTOF << ", WL=" << currentWL << ", OF_Nets=" << overflowedNets << ", Elapsed=" << (int)elapsed << "s" << std::endl;
        
        if (currentTOF == 0) break;
        
        // Slower, more controlled parameter updates
        k_factor = std::min(500.0, k_factor + 5.0); 
        h_factor = std::min(100.0, h_factor + 0.1);
        p_exponent = std::min(8.0, p_exponent + 0.05);
    }
    restoreBestSolution();
}
