#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <map>
#include <algorithm>

using namespace std;

struct Segment {
    char type;
    int x1, y1, x2, y2;
    int net_id;
};

bool check_shorts(const string& filename) {
    ifstream fin(filename);
    if (!fin) return false;
    string line;
    vector<Segment> segments;
    int current_net = -1;
    while (getline(fin, line)) {
        stringstream ss(line);
        string cmd;
        ss >> cmd;
        if (cmd == ".begin") ss >> current_net;
        else if (cmd == ".H") {
            int x1, y, x2;
            ss >> x1 >> y >> x2;
            segments.push_back({'H', min(x1, x2), y, max(x1, x2), y, current_net});
        } else if (cmd == ".V") {
            int x, y1, y2;
            ss >> x >> y1 >> y2;
            segments.push_back({'V', x, min(y1, y2), x, max(y1, y2), current_net});
        }
    }

    bool found_short = false;
    // Check horizontal shorts (same track, different nets, overlapping X)
    map<int, vector<Segment>> h_by_track;
    for (auto& s : segments) if (s.type == 'H') h_by_track[s.y1].push_back(s);
    for (auto& [y, segs] : h_by_track) {
        for (size_t i = 0; i < segs.size(); ++i) {
            for (size_t j = i + 1; j < segs.size(); ++j) {
                if (segs[i].net_id != segs[j].net_id) {
                    if (max(segs[i].x1, segs[j].x1) < min(segs[i].x2, segs[j].x2)) {
                        cout << "H-Short on Track " << y << ": Net " << segs[i].net_id << " and Net " << segs[j].net_id << endl;
                        found_short = true;
                    }
                }
            }
        }
    }

    // Check vertical shorts (same column, different nets, overlapping Y)
    map<int, vector<Segment>> v_by_col;
    for (auto& s : segments) if (s.type == 'V') v_by_col[s.x1].push_back(s);
    for (auto& [x, segs] : v_by_col) {
        for (size_t i = 0; i < segs.size(); ++i) {
            for (size_t j = i + 1; j < segs.size(); ++j) {
                if (segs[i].net_id != segs[j].net_id) {
                    if (max(segs[i].y1, segs[j].y1) < min(segs[i].y2, segs[j].y2)) {
                        cout << "V-Short on Column " << x << ": Net " << segs[i].net_id << " and Net " << segs[j].net_id << endl;
                        found_short = true;
                    }
                }
            }
        }
    }
    return !found_short;
}

int main(int argc, char* argv[]) {
    if (argc < 2) return 1;
    if (check_shorts(argv[1])) cout << "No shorts found in " << argv[1] << endl;
    else cout << "Shorts found in " << argv[1] << endl;
    return 0;
}
