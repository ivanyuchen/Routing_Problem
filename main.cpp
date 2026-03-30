#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>
#include <map>
#include <algorithm>
#include <set>
#include <cmath>
#include <random>
#include <chrono>

using namespace std;

struct Segment {
    char type;
    int x1, y1, x2, y2;
    int net_id;
};

struct Net {
    int id;
    vector<pair<int, bool>> pins; 
    int leftmost_col = 1e9;
    int rightmost_col = -1;
    vector<Segment> segments;
};

struct Solution {
    int num_tracks;
    map<int, vector<Segment>> net_segments;
    double cost = 1e30;
    bool valid = false;

    void calculate_cost(double alpha, double beta, double gamma, double delta,
                        const map<int, Net>& all_nets, 
                        const set<pair<int, int>>& constraints) {
        if (!valid) return;
        
        long long total_wl = 0;
        int total_vias = 0;
        int max_x = 0;
        for (auto const& [nid, segs] : net_segments) {
            for (auto const& s : segs) {
                if (s.type == 'H') { total_wl += abs(s.x1 - s.x2); max_x = max(max_x, s.x2); }
                else { total_wl += abs(s.y1 - s.y2); total_vias += 2; }
            }
        }

        vector<vector<int>> grid(max_x + 1, vector<int>(num_tracks + 2, 0));
        for (auto const& [nid, segs] : net_segments) {
            for (auto const& s : segs) if (s.type == 'H') {
                for (int x = s.x1; x <= s.x2; ++x) grid[x][s.y1] = nid;
            }
        }

        map<int, int> coupling;
        for (int x = 0; x <= max_x; ++x) {
            for (int t = 1; t < num_tracks; ++t) {
                int n1 = grid[x][t], n2 = grid[x][t+1];
                if (n1 > 0 && n2 > 0 && n1 != n2) {
                    int k1 = min(n1, n2), k2 = max(n1, n2);
                    if (constraints.count({k1, k2})) { coupling[n1]++; coupling[n2]++; }
                }
            }
        }

        double max_ratio = 0;
        for (auto const& [nid, net] : all_nets) {
            int span = net.rightmost_col - net.leftmost_col;
            if (span > 0) {
                double ratio = (double)coupling[nid] / span;
                max_ratio = max(max_ratio, ratio);
            }
        }

        cost = alpha * exp((double)num_tracks / delta) + beta * (total_wl + 5.0 * total_vias);
        if (max_ratio >= 0.2) cost += 1e9 * (max_ratio + 1.0); 
    }
};

class GreedyRouter {
    int W;
    mt19937& rng;
    const vector<int>& top_p;
    const vector<int>& bot_p;
    map<int, Net>& all_nets;
    const set<pair<int, int>>& constraints;

public:
    GreedyRouter(int tracks, mt19937& gen, const vector<int>& t, const vector<int>& b, 
                 map<int, Net>& nets, const set<pair<int, int>>& cons) 
        : W(tracks), rng(gen), top_p(t), bot_p(b), all_nets(nets), constraints(cons) {}

    int get_next_pin_y(int nid, int cur_col) {
        for (auto& p : all_nets[nid].pins) if (p.first > cur_col) return p.second ? (W + 1) : 0;
        return -1;
    }

    Solution solve() {
        Solution sol; sol.num_tracks = W;
        vector<int> track_at(W + 2, 0);
        map<int, vector<Segment>> net_segs;
        int num_cols = max((int)top_p.size(), (int)bot_p.size());

        auto add_v = [&](int x, int y1, int y2, int nid) {
            if (y1 == y2) return;
            net_segs[nid].push_back({'V', x, min(y1, y2), x, max(y1, y2), nid});
        };
        auto add_h = [&](int x1, int x2, int y, int nid) {
            if (x1 == x2) return;
            net_segs[nid].push_back({'H', min(x1, x2), y, max(x1, x2), y, nid});
        };

        int c = 0;
        while (true) {
            bool active = false; for (int t=1; t<=W; ++t) if (track_at[t]>0) active = true;
            if (c >= num_cols && !active) break;
            if (c > num_cols + 500) return sol;

            int T = (c < (int)top_p.size()) ? top_p[c] : 0;
            int B = (c < (int)bot_p.size()) ? bot_p[c] : 0;
            vector<int> v_mask(W + 2, 0);

            if (T == B && T > 0) {
                add_v(c, 0, W + 1, T); for (int y=0; y<=W+1; ++y) v_mask[y]=T;
                bool found = false;
                for (int t=1; t<=W; ++t) if (track_at[t] == T) { found = true; break; }
                if (!found) {
                    for (int t=W; t>=1; --t) if (track_at[t] == 0) { track_at[t] = T; found = true; break; }
                }
                if (!found) return sol;
            } else {
                if (T > 0 && B > 0) {
                    int tT = -1, tB = -1;
                    int curT = -1, curB = -1;
                    for (int t=W; t>=1; --t) if (track_at[t] == T) { curT = t; break; }
                    for (int t=1; t<=W; ++t) if (track_at[t] == B) { curB = t; break; }
                    if (curT != -1 && curB != -1 && curT > curB) { tT = curT; tB = curB; }
                    else {
                        vector<pair<int, int>> pairs;
                        for (int tt=W; tt>=2; --tt) {
                            if (track_at[tt] != 0 && track_at[tt] != T) continue;
                            for (int tb=tt-1; tb>=1; --tb) {
                                if (track_at[tb] != 0 && track_at[tb] != B) continue;
                                pairs.push_back({tt, tb});
                            }
                        }
                        if (pairs.empty()) return sol;
                        shuffle(pairs.begin(), pairs.end(), rng);
                        tT = pairs[0].first; tB = pairs[0].second;
                    }
                    track_at[tT] = T; track_at[tB] = B;
                    add_v(c, tT, W+1, T); add_v(c, 0, tB, B);
                    for (int y=tT; y<=W+1; ++y) v_mask[y]=T;
                    for (int y=0; y<=tB; ++y) v_mask[y]=B;
                } else if (T > 0) {
                    int cur = -1; for (int t=W; t>=1; --t) if (track_at[t] == T) cur = t;
                    if (cur != -1) { add_v(c, cur, W+1, T); for (int y=cur; y<=W+1; ++y) v_mask[y]=T; }
                    else {
                        vector<int> cands; for (int t=W; t>=1; --t) if (track_at[t] == 0) cands.push_back(t);
                        if (cands.empty()) return sol;
                        int t = cands[rng() % cands.size()];
                        track_at[t] = T; add_v(c, t, W+1, T); for (int y=t; y<=W+1; ++y) v_mask[y]=T;
                    }
                } else if (B > 0) {
                    int cur = -1; for (int t=1; t<=W; ++t) if (track_at[t] == B && (v_mask[t] == 0 || v_mask[t] == B)) cur = t;
                    if (cur != -1) { add_v(c, 0, cur, B); for (int y=0; y<=cur; ++y) v_mask[y]=B; }
                    else {
                        vector<int> cands; for (int t=1; t<=W; ++t) if (track_at[t] == 0 && v_mask[t] == 0) cands.push_back(t);
                        if (cands.empty()) return sol;
                        int t = cands[rng() % cands.size()];
                        track_at[t] = B; add_v(c, 0, t, B); for (int y=0; y<=t; ++y) v_mask[y]=B;
                    }
                }
            }

            // Net Collapse
            for (auto const& [nid, net] : all_nets) {
                vector<int> my_t; for (int t=1; t<=W; ++t) if (track_at[t]==nid) my_t.push_back(t);
                if (my_t.size() > 1) {
                    int mn = my_t.front(), mx = my_t.back(); bool ok = true;
                    for (int y=mn; y<=mx; ++y) if (v_mask[y] != 0 && v_mask[y] != nid) ok = false;
                    if (ok) {
                        add_v(c, mn, mx, nid); int ny = get_next_pin_y(nid, c);
                        int keep = (ny > mx) ? mx : mn;
                        for (int t : my_t) if (t != keep) track_at[t] = 0;
                        for (int y=mn; y<=mx; ++y) v_mask[y] = nid;
                    }
                }
            }

            // Jogging
            for (int t=W; t>1; --t) { // Falling
                int nid = track_at[t];
                if (nid > 0 && get_next_pin_y(nid, c) != -1 && get_next_pin_y(nid, c) < t) {
                    if (track_at[t-1] == 0 && v_mask[t] == 0 && v_mask[t-1] == 0) {
                        add_v(c, t, t-1, nid); track_at[t-1] = nid; track_at[t] = 0; v_mask[t]=v_mask[t-1]=nid;
                    }
                }
            }
            for (int t=1; t<W; ++t) { // Rising
                int nid = track_at[t];
                if (nid > 0 && get_next_pin_y(nid, c) != -1 && get_next_pin_y(nid, c) > t) {
                    if (track_at[t+1] == 0 && v_mask[t] == 0 && v_mask[t+1] == 0) {
                        add_v(c, t, t+1, nid); track_at[t+1] = nid; track_at[t] = 0; v_mask[t]=v_mask[t+1]=nid;
                    }
                }
            }

            // Extension
            for (int t=1; t<=W; ++t) {
                int nid = track_at[t];
                if (nid > 0) {
                    add_h(c, c + 1, t, nid);
                    if (c + 1 >= all_nets[nid].rightmost_col) {
                        bool needed = false; for (int tt=1; tt<=W; ++tt) if (tt != t && track_at[tt] == nid) needed = true;
                        if (!needed) track_at[t] = 0;
                    }
                }
            }
            c++;
        }
        
        sol.valid = true;
        sol.net_segments = net_segs;
        return sol;
    }
};

int main(int argc, char* argv[]) {
    if (argc > 1) freopen(argv[1], "r", stdin);
    if (argc > 2) freopen(argv[2], "w", stdout);
    ios::sync_with_stdio(false); cin.tie(NULL);
    
    double alpha, beta, gamma, delta;
    if (!(cin >> alpha >> beta >> gamma >> delta)) return 0;
    
    vector<int> top_pins, bot_pins;
    string line; getline(cin, line);
    auto read_pins = [&](vector<int>& pins, map<int, Net>& nets) {
        if (!getline(cin, line) || line.empty()) getline(cin, line);
        stringstream ss(line); int p, col = 0;
        while (ss >> p) {
            pins.push_back(p);
            if (p > 0) {
                nets[p].id = p;
                nets[p].pins.push_back({col, &pins == &top_pins});
                nets[p].leftmost_col = min(nets[p].leftmost_col, col);
                nets[p].rightmost_col = max(nets[p].rightmost_col, col);
            }
            col++;
        }
    };
    map<int, Net> all_nets;
    read_pins(top_pins, all_nets);
    read_pins(bot_pins, all_nets);
    
    int num_constraints; if (!(cin >> num_constraints)) num_constraints = 0;
    set<pair<int, int>> constraints;
    for (int i = 0; i < num_constraints; ++i) {
        int n1, n2; cin >> n1 >> n2;
        constraints.insert({min(n1, n2), max(n1, n2)});
    }
    
    auto start_time = chrono::steady_clock::now();
    mt19937 rng(1337);
    Solution best_sol;
    
    int baseline = (delta < 30) ? 30 : 60;
    for (int t = baseline; t >= 1; --t) {
        bool found_at_t = false;
        for (int iter = 0; iter < 100; ++iter) {
            if (chrono::duration_cast<chrono::seconds>(chrono::steady_clock::now() - start_time).count() > 285) goto stop;
            
            GreedyRouter router(t, rng, top_pins, bot_pins, all_nets, constraints);
            Solution sol = router.solve();
            if (sol.valid) {
                sol.calculate_cost(alpha, beta, gamma, delta, all_nets, constraints);
                if (sol.cost < best_sol.cost) best_sol = sol;
                found_at_t = true;
            }
        }
        if (!found_at_t && t == baseline) {
            // If failed at baseline, try higher track counts briefly
            for (int t2 = baseline + 1; t2 <= baseline + 20; ++t2) {
                GreedyRouter router(t2, rng, top_pins, bot_pins, all_nets, constraints);
                Solution sol = router.solve();
                if (sol.valid) {
                    sol.calculate_cost(alpha, beta, gamma, delta, all_nets, constraints);
                    if (sol.cost < best_sol.cost) best_sol = sol;
                    goto stop;
                }
            }
        }
    }

stop:
    if (best_sol.valid) {
        for (auto const& [nid, segs] : best_sol.net_segments) {
            cout << ".begin " << nid << "\n";
            // Merge horizontal segments
            map<int, vector<pair<int, int>>> h_lines;
            for (auto const& s : segs) if (s.type == 'H') h_lines[s.y1].push_back({s.x1, s.x2});
            for (auto& [y, intervals] : h_lines) {
                sort(intervals.begin(), intervals.end());
                int cur_x1 = intervals[0].first, cur_x2 = intervals[0].second;
                for (size_t i = 1; i < intervals.size(); ++i) {
                    if (intervals[i].first <= cur_x2) cur_x2 = max(cur_x2, intervals[i].second);
                    else { cout << ".H " << cur_x1 << " " << y << " " << cur_x2 << "\n"; cur_x1 = intervals[i].first; cur_x2 = intervals[i].second; }
                }
                cout << ".H " << cur_x1 << " " << y << " " << cur_x2 << "\n";
            }
            for (auto const& s : segs) if (s.type == 'V') cout << ".V " << s.x1 << " " << s.y1 << " " << s.y2 << "\n";
            cout << ".end\n";
        }
    }
    return 0;
}
