#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <random>
#include <chrono>
#include <map>
#include <set>

using namespace std;

// Structure to store a wire segment
struct Segment {
    int net_id;
    bool is_horizontal;
    int x1, y1, x2, y2;
};

// Structure for Net information
struct Net {
    int id;
    vector<int> top_cols;
    vector<int> bot_cols;
    int min_col = 1e9, max_col = -1;
    
    void add_col(int col) {
        min_col = min(min_col, col);
        max_col = max(max_col, col);
    }
};

// Cost coefficients
double alpha, beta_val, gamma_val, delta;

struct Solution {
    int track_num;
    vector<Segment> segments;
    int total_vias = 0;
    long long total_wl = 0;
    int spill_over = 0;
    double cost = 1e18;
    bool valid = false;

    void calculate_cost(const map<int, Net>& nets, const vector<pair<int, int>>& constraints) {
        if (!valid) return;
        
        int max_x = 0;
        for (const auto& s : segments) max_x = max(max_x, s.x2);
        
        // Optimize coupling calculation: O(X * W) instead of O(Constraints * X * W)
        // track_at[x][y] = net_id
        vector<vector<int>> track_at(max_x + 1, vector<int>(track_num + 2, 0));
        for (const auto& s : segments) {
            if (s.is_horizontal) {
                for (int x = s.x1; x <= s.x2; ++x) track_at[x][s.y1] = s.net_id;
            }
        }

        map<pair<int, int>, long long> coupling_lens;
        for (int x = 1; x <= max_x; ++x) {
            for (int y = 1; y < track_num; ++y) {
                int n1 = track_at[x][y];
                int n2 = track_at[x][y+1];
                if (n1 != 0 && n2 != 0 && n1 != n2) {
                    if (n1 > n2) swap(n1, n2);
                    coupling_lens[{n1, n2}]++;
                }
            }
        }

        double max_coupling_ratio = 0;
        for (const auto& cp : constraints) {
            int n1 = cp.first;
            int n2 = cp.second;
            if (n1 > n2) swap(n1, n2);
            long long clen = coupling_lens[{n1, n2}];
            
            double span1 = nets.at(cp.first).max_col - nets.at(cp.first).min_col;
            double span2 = nets.at(cp.second).max_col - nets.at(cp.second).min_col;
            if (span1 > 0) max_coupling_ratio = max(max_coupling_ratio, (double)clen / span1);
            if (span2 > 0) max_coupling_ratio = max(max_coupling_ratio, (double)clen / span2);
        }

        cost = alpha * exp((double)track_num / delta) + beta_val * (total_wl + 5.0 * total_vias) + gamma_val * (double)spill_over;
        // Optionally penalize high coupling ratio if you want the search to avoid it
        // cost += some_penalty * max_coupling_ratio; 
    }
};

// Global variables
map<int, Net> all_nets;
vector<int> top_pins, bot_pins;
vector<pair<int, int>> coupling_constraints;
int max_col_orig = 0;

void merge_horizontal_segments(vector<Segment>& segs) {
    if (segs.empty()) return;
    map<pair<int, int>, vector<pair<int, int>>> h_lines;
    vector<Segment> others;
    for (const auto& s : segs) {
        if (s.is_horizontal && s.x1 != s.x2) h_lines[{s.net_id, s.y1}].push_back({s.x1, s.x2});
        else if (!s.is_horizontal && s.y1 != s.y2) others.push_back(s);
    }
    vector<Segment> merged;
    for (auto& entry : h_lines) {
        auto& intervals = entry.second;
        sort(intervals.begin(), intervals.end());
        int start = intervals[0].first, end = intervals[0].second;
        for (size_t i = 1; i < intervals.size(); ++i) {
            if (intervals[i].first <= end) end = max(end, intervals[i].second);
            else { merged.push_back({entry.first.first, true, start, entry.first.second, end, entry.first.second});
                   start = intervals[i].first; end = intervals[i].second; }
        }
        merged.push_back({entry.first.first, true, start, entry.first.second, end, entry.first.second});
    }
    merged.insert(merged.end(), others.begin(), others.end());
    segs = merged;
}

class RandomizedRouter {
    int W;
    mt19937& rng;
public:
    RandomizedRouter(int tracks, mt19937& gen) : W(tracks), rng(gen) {}

    Solution solve() {
        Solution sol; sol.track_num = W;
        vector<int> track_occ(W + 2, 0); // 1..W are valid tracks
        map<int, vector<int>> net_to_tracks;
        int max_x = max_col_orig;
        
        auto add_h = [&](int nid, int x1, int y, int x2) {
            if (x1 == x2) return;
            sol.segments.push_back({nid, true, min(x1, x2), y, max(x1, x2), y});
            sol.total_wl += abs(x1 - x2);
        };

        for (int col = 0; col < max_x + W + 100; ++col) {
            int tp = (col < (int)top_pins.size()) ? top_pins[col] : 0;
            int bp = (col < (int)bot_pins.size()) ? bot_pins[col] : 0;
            
            vector<int> col_v_occ(W + 2, 0);
            
            auto check_v_safe = [&](int nid, int x, int y1, int y2) {
                int m_y1 = min(y1, y2), m_y2 = max(y1, y2);
                for (int y = m_y1; y <= m_y2; ++y) {
                    if (col_v_occ[y] != 0 && col_v_occ[y] != nid) return false;
                }
                return true;
            };

            auto add_v_safe = [&](int nid, int x, int y1, int y2) {
                if (y1 == y2) return true;
                if (!check_v_safe(nid, x, y1, y2)) return false;
                int m_y1 = min(y1, y2), m_y2 = max(y1, y2);
                for (int y = m_y1; y <= m_y2; ++y) col_v_occ[y] = nid;
                sol.segments.push_back({nid, false, x, m_y1, x, m_y2});
                sol.total_wl += m_y2 - m_y1; sol.total_vias += 2;
                return true;
            };

            // 1. Connect pins to tracks
            if (tp == bp && tp > 0) {
                int target = -1;
                if (net_to_tracks.count(tp)) target = net_to_tracks[tp][0];
                else {
                    for (int y = 1; y <= W; ++y) if (track_occ[y] == 0) { target = y; break; }
                }
                if (target == -1 || !add_v_safe(tp, col, 0, W + 1)) {
                    cerr << "Fail S1 target=" << target << " col=" << col << endl;
                    return sol;
                }
                if (track_occ[target] == 0) { track_occ[target] = tp; net_to_tracks[tp].push_back(target); }
            } else {
                int tp_target = -1;
                if (tp > 0) {
                    if (net_to_tracks.count(tp)) {
                        for (int y : net_to_tracks[tp]) {
                            if (bp > 0 && y <= 1) continue;
                            tp_target = max(tp_target, y);
                        }
                    }
                }

                int bp_target = -1;
                if (bp > 0) {
                    if (net_to_tracks.count(bp)) {
                        int lowest = 1e9;
                        for (int y : net_to_tracks[bp]) {
                            if (tp > 0 && tp_target != -1 && y >= tp_target) continue;
                            lowest = min(lowest, y);
                        }
                        if (lowest != 1e9) bp_target = lowest;
                    }
                }

                if (bp > 0 && bp_target == -1) {
                    for (int y = 1; y <= W; ++y) {
                        if (track_occ[y] == 0 && (tp == 0 || tp_target == -1 || y < tp_target)) { bp_target = y; break; }
                    }
                    if (bp_target == -1 && tp > 0 && tp_target != -1) {
                        tp_target = -1; 
                        for (int y = 1; y <= W; ++y) {
                            if (track_occ[y] == 0) { bp_target = y; break; }
                        }
                    }
                }

                if (tp > 0 && tp_target == -1) {
                    for (int y = W; y >= 1; --y) {
                        if (track_occ[y] == 0 && (bp == 0 || bp_target == -1 || y > bp_target)) { tp_target = y; break; }
                    }
                    if (tp_target == -1 && bp > 0 && bp_target != -1) {
                        // Extreme density edge case fallback
                        bp_target = -1;
                        for (int y = W; y >= 1; --y) if (track_occ[y] == 0) { tp_target = y; break; }
                        for (int y = 1; y <= W; ++y) if (track_occ[y] == 0 && y < tp_target) { bp_target = y; break; }
                    }
                }

                if (tp > 0) {
                    if (tp_target == -1 || !add_v_safe(tp, col, tp_target, W + 1)) return sol;
                    if (track_occ[tp_target] == 0) { track_occ[tp_target] = tp; net_to_tracks[tp].push_back(tp_target); }
                }
                if (bp > 0) {
                    if (bp_target == -1 || !add_v_safe(bp, col, 0, bp_target)) return sol;
                    if (track_occ[bp_target] == 0) { track_occ[bp_target] = bp; net_to_tracks[bp].push_back(bp_target); }
                }
            }

            // 2. Net Collapse (Partial subsets allowed, smartly pick closest to next pin)
            vector<int> active;
            for (auto const& [nid, ts] : net_to_tracks) if (ts.size() > 1) active.push_back(nid);
            shuffle(active.begin(), active.end(), rng);
            for (int nid : active) {
                auto& ts = net_to_tracks[nid]; sort(ts.begin(), ts.end());
                
                int next_pin_idx = -1;
                for (int c = col + 1; c < (int)max_col_orig; ++c) {
                    if ((c < (int)top_pins.size() && top_pins[c] == nid) || (c < (int)bot_pins.size() && bot_pins[c] == nid)) {
                        next_pin_idx = c; break;
                    }
                }
                bool to_top = (next_pin_idx != -1 && next_pin_idx < (int)top_pins.size() && top_pins[next_pin_idx] == nid);
                bool to_bot = (next_pin_idx != -1 && next_pin_idx < (int)bot_pins.size() && bot_pins[next_pin_idx] == nid);
                
                vector<int> new_ts;
                for (size_t i = 0; i < ts.size(); ) {
                    size_t best_j = i;
                    for (size_t k = i + 1; k < ts.size(); ++k) {
                        bool clr = true;
                        for (int y = ts[i]; y <= ts[k]; ++y) {
                            if (col_v_occ[y] != 0 && col_v_occ[y] != nid) { clr = false; break; }
                        }
                        if (clr) best_j = k;
                        else break;
                    }
                    if (best_j > i) {
                        if (add_v_safe(nid, col, ts[i], ts[best_j])) {
                            for (int y = ts[i]; y <= ts[best_j]; ++y) {
                                if (track_occ[y] == nid) track_occ[y] = 0;
                            }
                            int final_y = ts[i];
                            if (to_top && !to_bot) final_y = ts[best_j]; // Upper is higher y value
                            else if (to_bot && !to_top) final_y = ts[i]; // Lower is lower y value
                            else final_y = (rng() % 2) ? ts[i] : ts[best_j];
                            
                            track_occ[final_y] = nid;
                            new_ts.push_back(final_y);
                        } else {
                            new_ts.push_back(ts[i]);
                        }
                        i = best_j + 1;
                    } else {
                        new_ts.push_back(ts[i]);
                        i++;
                    }
                }
                net_to_tracks[nid] = new_ts;
            }

            // 3. Smart Jogging (Aggressive 100% Target next pin, Sorted Bidirectional)
            vector<pair<int, int>> jog_up;
            vector<pair<int, int>> jog_dn;
            for (auto const& [nid, ts] : net_to_tracks) {
                if (ts.size() == 1) {
                    int cur_y = ts[0];
                    int next_pin_idx = -1;
                    for (int c = col + 1; c < (int)max_col_orig; ++c) {
                        if ((c < (int)top_pins.size() && top_pins[c] == nid) || (c < (int)bot_pins.size() && bot_pins[c] == nid)) {
                            next_pin_idx = c; break;
                        }
                    }
                    if (next_pin_idx != -1) {
                        bool to_top = (next_pin_idx < (int)top_pins.size() && top_pins[next_pin_idx] == nid);
                        bool to_bot = (next_pin_idx < (int)bot_pins.size() && bot_pins[next_pin_idx] == nid);
                        if (to_top && !to_bot) jog_up.push_back({cur_y, nid});
                        else if (to_bot && !to_top) jog_dn.push_back({cur_y, nid});
                        else if (rng() % 100 < 5) { // Random spread if both
                            if (rng() % 2) jog_up.push_back({cur_y, nid});
                            else jog_dn.push_back({cur_y, nid});
                        }
                    } else if (rng() % 100 < 5) {
                        if (cur_y > W / 2) jog_up.push_back({cur_y, nid});
                        else jog_dn.push_back({cur_y, nid});
                    }
                }
            }
            
            sort(jog_up.begin(), jog_up.end(), [](auto a, auto b){ return a.first > b.first; });
            for (auto& p : jog_up) {
                int nid = p.second; int cur_y = p.first;
                int next_y = cur_y + 1;
                if (next_y <= W && track_occ[next_y] == 0) {
                    if (add_v_safe(nid, col, cur_y, next_y)) {
                        track_occ[cur_y] = 0; track_occ[next_y] = nid;
                        net_to_tracks[nid] = {next_y};
                    }
                }
            }
            
            sort(jog_dn.begin(), jog_dn.end(), [](auto a, auto b){ return a.first < b.first; });
            for (auto& p : jog_dn) {
                int nid = p.second; int cur_y = p.first;
                int next_y = cur_y - 1;
                if (next_y >= 1 && track_occ[next_y] == 0) {
                    if (add_v_safe(nid, col, cur_y, next_y)) {
                        track_occ[cur_y] = 0; track_occ[next_y] = nid;
                        net_to_tracks[nid] = {next_y};
                    }
                }
            }

            // 4. Extension
            bool none_active = true;
            for (int y = 1; y <= W; ++y) {
                int nid = track_occ[y];
                if (nid == 0) continue;
                if (col >= all_nets[nid].max_col && net_to_tracks[nid].size() == 1) {
                    track_occ[y] = 0;
                    net_to_tracks.erase(nid);
                } else {
                    add_h(nid, col, y, col + 1);
                    none_active = false;
                }
            }
            if (none_active && col >= max_x - 1) {
                sol.valid = true; sol.spill_over = max(0, col - ((int)max_col_orig - 1));
                break;
            }
            if (col > max_x + 500) return sol;
        }
        return sol;
    }
};

#include <cstdio>

int main(int argc, char* argv[]) {
    if (argc > 1) {
        if (!freopen(argv[1], "r", stdin)) {
            cerr << "Failed to open input file: " << argv[1] << endl;
            return 1;
        }
    }
    if (argc > 2) {
        if (!freopen(argv[2], "w", stdout)) {
            cerr << "Failed to open output file: " << argv[2] << endl;
            return 1;
        }
    }
    
    ios::sync_with_stdio(false);
    cin.tie(NULL);
    
    if (!(cin >> alpha >> beta_val >> gamma_val >> delta)) return 0;
    
    string line;
    getline(cin, line); // consume remainder of first line
    
    auto read_pins = [&](vector<int>& pins) {
        if (!getline(cin, line) || line.empty()) getline(cin, line);
        stringstream ss(line);
        int p;
        while (ss >> p) {
            int col_idx = pins.size(); // 0-based
            pins.push_back(p);
            if (p > 0) {
                all_nets[p].id = p;
                all_nets[p].add_col(col_idx);
            }
        }
    };
    
    read_pins(top_pins);
    read_pins(bot_pins);
    
    max_col_orig = max(top_pins.size(), bot_pins.size());
    
    int num_constraints;
    if (!(cin >> num_constraints)) num_constraints = 0;
    for (int i = 0; i < num_constraints; ++i) {
        int n1, n2;
        cin >> n1 >> n2;
        coupling_constraints.push_back({n1, n2});
    }
    
    auto start_time = chrono::steady_clock::now();
    mt19937 rng(1337);
    
    Solution best_sol;
    best_sol.cost = 1e30;
    
    // Determine track limit based on problem scale or user hint
    int track_limit = (max_col_orig > 150 || all_nets.size() > 50) ? 80 : 45;
    
    for (int t = track_limit; t >= 1; --t) {
        for (int iter = 0; iter < 10000; ++iter) {
            if (iter % 50 == 0) {
                auto now = chrono::steady_clock::now();
                if (chrono::duration_cast<chrono::seconds>(now - start_time).count() > 285) goto stop_search;
            }
            
            RandomizedRouter router(t, rng);
            Solution sol = router.solve();
            if (sol.valid) {
                sol.calculate_cost(all_nets, coupling_constraints);
                if (sol.cost < best_sol.cost) {
                    best_sol = sol;
                }
            }
        }
    }

stop_search:
    if (best_sol.valid) {
        merge_horizontal_segments(best_sol.segments);
        map<int, vector<Segment>> net_segs;
        for (const auto& s : best_sol.segments) net_segs[s.net_id].push_back(s);
        
        for (auto const& [nid, segs] : net_segs) {
            cout << ".begin " << nid << "\n";
            for (const auto& s : segs) {
                if (s.is_horizontal) cout << ".H " << s.x1 << " " << s.y1 << " " << s.x2 << "\n";
                else cout << ".V " << s.x1 << " " << s.y1 << " " << s.y2 << "\n";
            }
            cout << ".end\n";
        }
    }
    
    return 0;
}
