#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <map>

using namespace std;

struct Segment {
    char type;
    int x1, y1, x2, y2;
};

int main(int argc, char* argv[]) {
    if (argc < 3) return 1;
    // argv[1]: testcase.txt, argv[2]: output.txt
    ifstream f_tc(argv[1]);
    string line;
    getline(f_tc, line); // cost
    
    vector<int> top, bot;
    if (getline(f_tc, line)) { stringstream ss(line); int p; while(ss >> p) top.push_back(p); }
    if (getline(f_tc, line)) { stringstream ss(line); int p; while(ss >> p) bot.push_back(p); }
    
    // Read output.txt
    ifstream f_out(argv[2]);
    map<int, vector<Segment>> net_segs;
    int current_net = -1;
    while (getline(f_out, line)) {
        if (line.substr(0, 6) == ".begin") {
            stringstream ss(line.substr(7)); ss >> current_net;
        } else if (line.substr(0, 2) == ".V") {
            stringstream ss(line.substr(3));
            int x, y1, y2; ss >> x >> y1 >> y2;
            net_segs[current_net].push_back({'V', x, y1, x, y2});
        }
    }
    
    int num_tracks = 0;
    for (auto& [id, segs] : net_segs) for (auto& s : segs) {
        num_tracks = max(num_tracks, s.y1);
        num_tracks = max(num_tracks, s.y2);
    }
    // Assume num_tracks is the largest y-coordinate seen.
    // If output reports success, it usually has the baseline tracks.
    // In TC2, top pin boundary is num_tracks. Let's find it correctly.
    // Usually top pins are at y = tracks + 1.
    int top_y = num_tracks;
    
    int leaks_top = 0, leaks_bot = 0;
    int redundant_top = 0, redundant_bot = 0;
    
    map<int, int> top_count, bot_count;
    for (auto& [id, segs] : net_segs) {
        for (auto& s : segs) {
            if (s.y2 == top_y) {
                if (s.x1 >= top.size() || top[s.x1] != id) {
                    leaks_top++;
                    cout << "Leak Top: Net " << id << " Col " << s.x1 << " reaches " << top_y << endl;
                } else {
                    top_count[s.x1]++;
                }
            }
            if (s.y1 == 0) {
                if (s.x1 >= bot.size() || bot[s.x1] != id) {
                    leaks_bot++;
                    cout << "Leak Bot: Net " << id << " Col " << s.x1 << " reaches 0" << endl;
                } else {
                    bot_count[s.x1]++;
                }
            }
        }
    }
    
    for (auto& [x, c] : top_count) if (c > 1) { redundant_top += c - 1; cout << "Redundant Top: Col " << x << " has " << c << " segments" << endl; }
    for (auto& [x, c] : bot_count) if (c > 1) { redundant_bot += c - 1; cout << "Redundant Bot: Col " << x << " has " << c << " segments" << endl; }
    
    cout << "Total Leaks Top: " << leaks_top << endl;
    cout << "Total Leaks Bot: " << leaks_bot << endl;
    cout << "Total Redundant Top: " << redundant_top << endl;
    cout << "Total Redundant Bot: " << redundant_bot << endl;
    
    return 0;
}
