#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <algorithm>
#include <set>
#include <cmath>

using namespace std;

struct Segment {
    char type; // 'H' 或 'V'
    int x1, y1, x2, y2;
};

struct Net {
    int id;
    int rightmost_col; // 紀錄此 Net 最右邊的 Pin 位置
    vector<Segment> segments;
};

struct Constraint {
    int from_net;
    int to_net;
};

class ChannelRouter {
private:
    double alpha, beta, gamma, delta;
    vector<int> top_pins;
    vector<int> bottom_pins;
    vector<Constraint> constraints;
    unordered_map<int, Net> nets;
    
    int num_tracks; 
    vector<int> tracks; // tracks[i] 記錄第 i 層軌道目前屬於哪個 Net，0 代表空

public:
    ChannelRouter(int num_tracks) : num_tracks(num_tracks) {
        tracks.resize(num_tracks + 2, 0); 
    }

    bool parseInput(const string& filename) {
        ifstream fin(filename);
        if (!fin.is_open()) {
            cerr << "Error: Cannot open input file " << filename << endl;
            return false;
        }

        string line;
        if (getline(fin, line)) {
            stringstream ss(line);
            ss >> alpha >> beta >> gamma >> delta;
        }

        if (getline(fin, line)) {
            stringstream ss(line);
            int pin;
            while (ss >> pin) top_pins.push_back(pin);
        }

        if (getline(fin, line)) {
            stringstream ss(line);
            int pin;
            while (ss >> pin) bottom_pins.push_back(pin);
        }

        int constraint_num;
        if (getline(fin, line)) {
            stringstream ss(line);
            ss >> constraint_num;
            for (int i = 0; i < constraint_num; ++i) {
                if (getline(fin, line)) {
                    stringstream css(line);
                    int n1, n2;
                    css >> n1 >> n2;
                    constraints.push_back({n1, n2});
                }
            }
        }
        fin.close();

        // 預處理：找出每個 Net 的 Rightmost Column
        for (int i = 0; i < top_pins.size(); ++i) {
            if (top_pins[i] > 0) nets[top_pins[i]].rightmost_col = max(nets[top_pins[i]].rightmost_col, i);
        }
        for (int i = 0; i < bottom_pins.size(); ++i) {
            if (bottom_pins[i] > 0) nets[bottom_pins[i]].rightmost_col = max(nets[bottom_pins[i]].rightmost_col, i);
        }

        return true;
    }

    bool check_v(int y1, int y2, int net_id, const vector<bool>& v_occupied) {
        int mn = min(y1, y2), mx = max(y1, y2);
        for (int i = mn; i <= mx; ++i) {
            if (v_occupied[i]) return false; 
        }
        return true;
    }

    void add_v(int col, int y1, int y2, int net_id, vector<bool>& v_occupied) {
        int mn = min(y1, y2), mx = max(y1, y2);
        for (int i = mn; i <= mx; ++i) v_occupied[i] = true;
        nets[net_id].segments.push_back({'V', col, mn, col, mx});
    }

    bool route() {
        int top_row_y = num_tracks + 1;
        int col = 0;
        int mid = num_tracks / 2;

        // --- 預處理 Constraint 快速查詢表 ---
        set<pair<int, int>> bad_pairs;
        for (const auto& c : constraints) {
            bad_pairs.insert({min(c.from_net, c.to_net), max(c.from_net, c.to_net)});
        }

        auto is_conflict = [&](int n1, int n2) {
            if (n1 <= 0 || n2 <= 0 || n1 == n2) return false;
            return bad_pairs.count({min(n1, n2), max(n1, n2)}) > 0;
        };

        while (true) {
            bool has_pins = (col < (int)top_pins.size() || col < (int)bottom_pins.size());
            bool has_active_tracks = false;
            for (int i = 1; i <= num_tracks; ++i) if (tracks[i] > 0) has_active_tracks = true;

            if (!has_pins && !has_active_tracks) break; 

            int T = (col < (int)top_pins.size()) ? top_pins[col] : 0;
            int B = (col < (int)bottom_pins.size()) ? bottom_pins[col] : 0;
            vector<bool> v_occupied(num_tracks + 2, false); 

            // --- Step 1: Top Pin ---
            if (T > 0) {
                bool connected = false;
                auto try_assign = [&](bool check_neighbors) {
                    for (int i = mid; i <= num_tracks; ++i) {
                        if ((tracks[i] == T || tracks[i] == 0) && check_v(i, top_row_y, T, v_occupied)) {
                            if (!check_neighbors || (!is_conflict(T, tracks[i-1]) && !is_conflict(T, tracks[i+1]))) {
                                tracks[i] = T; add_v(col, i, top_row_y, T, v_occupied); return true;
                            }
                        }
                    }
                    for (int i = mid - 1; i >= 1; --i) {
                        if ((tracks[i] == T || tracks[i] == 0) && check_v(i, top_row_y, T, v_occupied)) {
                            if (!check_neighbors || (!is_conflict(T, tracks[i-1]) && !is_conflict(T, tracks[i+1]))) {
                                tracks[i] = T; add_v(col, i, top_row_y, T, v_occupied); return true;
                            }
                        }
                    }
                    return false;
                };
                connected = try_assign(true);  // 優先避嫌
                if (!connected) connected = try_assign(false); // 避不開就保命
                if (!connected) return false;
            }

            // --- Step 2: Bottom Pin ---
            if (B > 0) {
                bool connected = false;
                auto try_assign_b = [&](bool check_neighbors) {
                    for (int i = mid + 1; i >= 1; --i) {
                        if ((tracks[i] == B || tracks[i] == 0) && check_v(0, i, B, v_occupied)) {
                            if (!check_neighbors || (!is_conflict(B, tracks[i-1]) && !is_conflict(B, tracks[i+1]))) {
                                tracks[i] = B; add_v(col, 0, i, B, v_occupied); return true;
                            }
                        }
                    }
                    for (int i = mid + 2; i <= num_tracks; ++i) {
                        if ((tracks[i] == B || tracks[i] == 0) && check_v(0, i, B, v_occupied)) {
                            if (!check_neighbors || (!is_conflict(B, tracks[i-1]) && !is_conflict(B, tracks[i+1]))) {
                                tracks[i] = B; add_v(col, 0, i, B, v_occupied); return true;
                            }
                        }
                    }
                    return false;
                };
                connected = try_assign_b(true); 
                if (!connected) connected = try_assign_b(false);
                if (!connected) return false;
            }

            // --- Step 3: Collapse ---
            for (auto const& pair : nets) {
                int n = pair.first;
                vector<int> my_tracks;
                for (int i = 1; i <= num_tracks; ++i) if (tracks[i] == n) my_tracks.push_back(i);
                
                if (my_tracks.size() > 1) {
                    int low = my_tracks.front(), high = my_tracks.back();
                    if (check_v(low, high, n, v_occupied)) {
                        add_v(col, low, high, n, v_occupied);
                        int keep = low;
                        int min_dist = abs(low - mid);
                        for (int idx : my_tracks) {
                            if (abs(idx - mid) < min_dist) { min_dist = abs(idx - mid); keep = idx; }
                        }
                        for (int idx : my_tracks) if (idx != keep) tracks[idx] = 0; 
                    }
                }
            }

            // --- Step 3.5: Dogleg (保命優先版) ---
            for (int i = 1; i <= num_tracks; ++i) {
                int n = tracks[i]; if (n <= 0) continue;
                
                if (i > mid + 1 && tracks[i-1] == 0 && check_v(i, i-1, n, v_occupied)) {
                    // 最頂層(大門口)直接跳，否則檢查是否會撞冤家
                    if (i == num_tracks || !is_conflict(n, tracks[i-2])) { 
                        add_v(col, i, i-1, n, v_occupied);
                        tracks[i-1] = n; tracks[i] = 0;
                    }
                } 
                else if (i < mid - 1 && tracks[i+1] == 0 && check_v(i, i+1, n, v_occupied)) {
                    // 最底層(大門口)直接跳，否則檢查是否會撞冤家
                    if (i == 1 || !is_conflict(n, tracks[i+2])) {
                        add_v(col, i, i+1, n, v_occupied);
                        tracks[i+1] = n; tracks[i] = 0;
                    }
                }
            }

            // --- Step 4: 釋放完成的 Net ---
            for (int i = 1; i <= num_tracks; ++i) {
                int n = tracks[i];
                if (n > 0 && col >= nets[n].rightmost_col) {
                    int count = 0;
                    for (int j = 1; j <= num_tracks; ++j) if (tracks[j] == n) count++;
                    if (count == 1) tracks[i] = 0; 
                }
            }

            // --- Step 5: 向右延伸 ---
            for (int i = 1; i <= num_tracks; ++i) {
                if (tracks[i] > 0) nets[tracks[i]].segments.push_back({'H', col, i, col + 1, i});
            }
            col++;
        }
        return true;
    }

    // --- 計算繞線成本 ---
    double calculateCost() {
        double total_length = 0;
        int total_vias = 0;
        int max_col = 0;

        // 1. 算總線長與 Vias
        for (const auto& pair : nets) {
            for (const auto& seg : pair.second.segments) {
                if (seg.type == 'H') {
                    total_length += abs(seg.x2 - seg.x1);
                    max_col = max(max_col, max(seg.x1, seg.x2));
                } else if (seg.type == 'V') {
                    total_length += abs(seg.y2 - seg.y1);
                    total_vias++; 
                }
            }
        }

        // 2. 算 Coupling Penalty
        int coupling_length = 0;
        set<pair<int, int>> bad_pairs;
        for (const auto& c : constraints) {
            bad_pairs.insert({min(c.from_net, c.to_net), max(c.from_net, c.to_net)});
        }

        vector<vector<int>> grid(max_col + 1, vector<int>(num_tracks + 2, 0));
        for (const auto& pair : nets) {
            int n = pair.first;
            for (const auto& seg : pair.second.segments) {
                if (seg.type == 'H') {
                    for (int c = seg.x1; c < seg.x2; ++c) grid[c][seg.y1] = n;
                }
            }
        }

        for (int c = 0; c < max_col; ++c) {
            for (int t = 1; t < num_tracks; ++t) {
                int n1 = grid[c][t];
                int n2 = grid[c][t+1]; 
                if (n1 > 0 && n2 > 0 && n1 != n2) {
                    if (bad_pairs.count({min(n1, n2), max(n1, n2)})) coupling_length++;
                }
            }
        }

        // 3. 結算總分
        return alpha * num_tracks + beta * total_length + gamma * total_vias + delta * coupling_length;
    }

    void mergeAndWriteOutput(const string& filename) {
        ofstream fout(filename);
        if (!fout.is_open()) return;

        for (auto& pair : nets) {
            int net_id = pair.first;
            if (net_id == 0 || pair.second.segments.empty()) continue;
            
            fout << ".begin " << net_id << "\n";
            
            vector<Segment> h_segs;
            for (const auto& seg : pair.second.segments) {
                if (seg.type == 'V') {
                    fout << ".V " << seg.x1 << " " << seg.y1 << " " << seg.y2 << "\n";
                } else {
                    h_segs.push_back(seg);
                }
            }

            sort(h_segs.begin(), h_segs.end(), [](const Segment& a, const Segment& b){
                if (a.y1 != b.y1) return a.y1 < b.y1;
                return a.x1 < b.x1;
            });

            vector<Segment> merged_h;
            for (const auto& s : h_segs) {
                if (merged_h.empty()) {
                    merged_h.push_back(s);
                } else {
                    auto& last = merged_h.back();
                    if (last.y1 == s.y1 && last.x2 == s.x1) {
                        last.x2 = s.x2; 
                    } else {
                        merged_h.push_back(s);
                    }
                }
            }

            for (const auto& s : merged_h) {
                fout << ".H " << s.x1 << " " << s.y1 << " " << s.x2 << "\n";
            }
            fout << ".end\n";
        }
        fout.close();
    }
};