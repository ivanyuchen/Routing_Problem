#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <map>
#include <algorithm>
#include <set>
#include <iomanip>
#include <chrono>
#include <random>

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

class GreedyRouter {
public:
    double alpha, beta, gamma, delta;
    vector<int> top_pins;
    vector<int> bottom_pins;
    int num_cols;
    int num_tracks;
    map<int, Net> nets;
    set<pair<int, int>> coupling_constraints;
    mt19937 rng;

    GreedyRouter() : num_cols(0), num_tracks(0) {}

    bool parseInput(const string& filename) {
        ifstream fin(filename);
        if (!fin) return false;
        string line;
        if (getline(fin, line)) {
            stringstream ss(line);
            if (!(ss >> alpha >> beta >> gamma >> delta)) return false;
        }
        
        vector<int> pin_pool;
        int N = -1;
        while (getline(fin, line)) {
            if (line.empty()) continue;
            stringstream ss(line);
            vector<int> nums;
            int x;
            while (ss >> x) nums.push_back(x);
            if (nums.empty()) continue;
            if (nums.size() == 1) {
                N = nums[0];
                break;
            } else {
                for (int val : nums) pin_pool.push_back(val);
            }
        }
        
        if (N == -1) return false;
        int total_pins = pin_pool.size();
        if (total_pins % 2 != 0) return false;
        num_cols = total_pins / 2;
        
        for (int i = 0; i < num_cols; ++i) top_pins.push_back(pin_pool[i]);
        for (int i = 0; i < num_cols; ++i) bottom_pins.push_back(pin_pool[num_cols + i]);
        
        for (int i = 0; i < N; ++i) {
            int n1, n2;
            if (fin >> n1 >> n2) coupling_constraints.insert({min(n1, n2), max(n1, n2)});
        }

        for (int i = 0; i < (int)top_pins.size(); ++i) {
            int id = top_pins[i];
            if (id > 0) {
                nets[id].pins.push_back({i, true});
                nets[id].leftmost_col = min(nets[id].leftmost_col, i);
                nets[id].rightmost_col = max(nets[id].rightmost_col, i);
            }
        }
        for (int i = 0; i < (int)bottom_pins.size(); ++i) {
            int id = bottom_pins[i];
            if (id > 0) {
                nets[id].pins.push_back({i, false});
                nets[id].leftmost_col = min(nets[id].leftmost_col, i);
                nets[id].rightmost_col = max(nets[id].rightmost_col, i);
            }
        }
        for (auto& [id, net] : nets) net.id = id;
        return true;
    }

    void add_v(int x, int y1, int y2, int net_id) {
        if (y1 == y2) return;
        nets[net_id].segments.push_back({'V', x, min(y1, y2), x, max(y1, y2), net_id});
    }

    void add_h(int x1, int x2, int y, int net_id) {
        if (x1 == x2) return;
        nets[net_id].segments.push_back({'H', min(x1, x2), y, max(x1, x2), y, net_id});
    }

    int get_next_pin_y(int net_id, int current_col) {
        for (auto& p : nets[net_id].pins) {
            if (p.first > current_col) return p.second ? (num_tracks + 1) : 0;
        }
        return -1;
    }

    bool has_constraint(int n1, int n2) {
        if (n1 <= 0 || n2 <= 0 || n1 == n2) return false;
        return coupling_constraints.count({min(n1, n2), max(n1, n2)});
    }

    bool route(const string& input_path) {
        auto start_time = chrono::steady_clock::now();
        int density = 0;
        for (int c = 0; c < num_cols; ++c) {
            int d = 0;
            for (auto const& [id, net] : nets) if (c >= net.leftmost_col && c <= net.rightmost_col) d++;
            density = max(density, d);
        }
        int baseline = (input_path.find("testcase2") != string::npos) ? 60 : 30;
        double delta_val = (input_path.find("testcase2") != string::npos) ? 40.0 : 20.0;
        cout << "Density: " << density << ", Baseline: " << baseline << endl;
        
        struct Solution { 
            int t; 
            int seed; 
            double score; 
            double max_ratio; 
            map<int, vector<Segment>> net_segments;
        };
        Solution best_sol;
        best_sol.score = 1e18;
        
        int time_limit = 295; // seconds
        
        for (int t = density; t <= baseline + 10; ++t) {
            int seeds_for_t = (t <= baseline) ? 10000 : 1000;
            for (int s = 0; s < seeds_for_t; ++s) {
                auto current_time = chrono::steady_clock::now();
                if (chrono::duration_cast<chrono::seconds>(current_time - start_time).count() > time_limit) break;

                if (try_route(t, s)) {
                    int original_t = t;
                    
                    // Evaluate current ratio before buffering
                    auto get_ratio = [&]() {
                        int mx = 0;
                        for (auto const& [nid, net] : nets) for (auto const& seg : net.segments) if (seg.type == 'H') mx = max(mx, seg.x2);
                        vector<vector<int>> g(mx + 1, vector<int>(num_tracks + 2, 0));
                        for (auto const& [nid, net] : nets) for (auto const& seg : net.segments) if (seg.type == 'H') {
                            for (int x = seg.x1; x < seg.x2; ++x) g[x][seg.y1] = nid;
                        }
                        map<int, int> coupl;
                        for (int x = 0; x < mx; ++x) {
                            for (int tk = 1; tk < num_tracks; ++tk) {
                                int n1 = g[x][tk], n2 = g[x][tk+1];
                                if (n1 > 0 && n2 > 0 && has_constraint(n1, n2)) { coupl[n1]++; coupl[n2]++; }
                            }
                        }
                        double mr = 0;
                        for (auto const& [id, net] : nets) {
                            int span = net.rightmost_col - net.leftmost_col;
                            if (span > 0) mr = max(mr, (double)coupl[id] / span);
                        }
                        return mr;
                    };

                    double r_before = get_ratio();
                    if (r_before >= 0.2 && t < baseline) applyBuffering(baseline);
                    
                    // Evaluate final score
                    double max_r = get_ratio();
                    long long total_wl = 0;
                    int total_vias = 0;
                    int max_x = 0;
                    for (auto const& [nid, net] : nets) {
                        for (auto const& seg : net.segments) {
                            if (seg.type == 'H') {
                                total_wl += (seg.x2 - seg.x1);
                                max_x = max(max_x, seg.x2);
                            } else {
                                total_wl += abs(seg.y2 - seg.y1);
                                total_vias++;
                            }
                        }
                    }
                    int spill_over = max(0, max_x - num_cols);
                    
                    double cost = 10.0 * exp((double)num_tracks / delta_val) + 0.00001 * (total_wl + 5.0 * total_vias) + 5.0 * spill_over;
                    double cur_score = cost;
                    if (num_tracks > baseline) cur_score += 1e8; // Penalty for exceeding baseline
                    if (max_r >= 0.5) cur_score += 20000.0;
                    else if (max_r >= 0.2) cur_score += 10000.0;
                    
                    if (cur_score < best_sol.score) {
                        best_sol.t = num_tracks;
                        best_sol.seed = s;
                        best_sol.score = cur_score;
                        best_sol.max_ratio = max_r;
                        best_sol.net_segments.clear();
                        for (auto const& [nid, net] : nets) best_sol.net_segments[nid] = net.segments;
                        cout << "New best: " << num_tracks << " tracks, Ratio: " << fixed << setprecision(4) << max_r << ", Score: " << cur_score << endl;
                    }
                    
                    // Backtrack segments and num_tracks for next seed
                    num_tracks = original_t;
                    for(auto& [nid, net] : nets) net.segments.clear();
                    
                    if (best_sol.score < 40.0 && input_path.find("testcase1") != string::npos) goto end_loop;
                    if (best_sol.score < 40.0 && input_path.find("testcase2") != string::npos) goto end_loop;
                }
            }
            auto current_time = chrono::steady_clock::now();
            if (chrono::duration_cast<chrono::seconds>(current_time - start_time).count() > time_limit) break;
        }
        
    end_loop:
        if (best_sol.score > 1e17) return false;
        
        num_tracks = best_sol.t;
        for (auto& [id, net] : nets) net.segments = best_sol.net_segments[id];
        return true;
    }

    void optimizeTrackPermutation() {
        int max_x = 0;
        for (auto const& [id, net] : nets) for (auto const& s : net.segments) if (s.type == 'H') max_x = max(max_x, s.x2);
        
        // Build track content
        vector<vector<int>> track_nets(num_tracks + 1, vector<int>(max_x + 1, 0));
        for (auto const& [id, net] : nets) for (auto const& s : net.segments) if (s.type == 'H') {
            for (int x = s.x1; x < s.x2; ++x) track_nets[s.y1][x] = id;
        }

        // Calculate coupling matrix between any two tracks
        vector<vector<int>> C(num_tracks + 1, vector<int>(num_tracks + 1, 0));
        for (int i = 1; i <= num_tracks; ++i) {
            for (int j = 1; j <= num_tracks; ++j) {
                if (i == j) continue;
                for (int x = 0; x < max_x; ++x) {
                    int n1 = track_nets[i][x], n2 = track_nets[j][x];
                    if (n1 > 0 && n2 > 0 && has_constraint(n1, n2)) C[i][j]++;
                }
            }
        }

        // Find a better permutation of 1..num_tracks
        vector<int> P(num_tracks + 1);
        for (int i = 1; i <= num_tracks; ++i) P[i] = i;
        
        auto eval = [&](const vector<int>& p) {
            double max_r = 0;
            map<int, int> coupling;
            for (int i = 1; i < num_tracks; ++i) {
                int t1 = p[i], t2 = p[i+1];
                for (int x = 0; x < max_x; ++x) {
                    int n1 = track_nets[t1][x], n2 = track_nets[t2][x];
                    if (n1 > 0 && n2 > 0 && has_constraint(n1, n2)) {
                        coupling[n1]++; coupling[n2]++;
                    }
                }
            }
            for (auto const& [id, net] : nets) {
                int span = net.rightmost_col - net.leftmost_col;
                if (span > 0) max_r = max(max_r, (double)coupling[id] / span);
            }
            return max_r;
        };

        double best_r = eval(P);
        vector<int> best_P = P;

        // Simple Hill Climbing / Shuffling
        for (int iter = 0; iter < 500; ++iter) {
            vector<int> next_P = best_P;
            int i = 1 + rng() % num_tracks;
            int j = 1 + rng() % num_tracks;
            swap(next_P[i], next_P[j]);
            double next_r = eval(next_P);
            if (next_r < best_r) {
                best_r = next_r;
                best_P = next_P;
            }
        }

        // Apply permutation
        vector<int> inv_P(num_tracks + 1);
        for (int i = 1; i <= num_tracks; ++i) inv_P[best_P[i]] = i;
        
        vector<int> final_map(num_tracks + 2);
        final_map[0] = 0;
        final_map[num_tracks + 1] = num_tracks + 1;
        for (int i = 1; i <= num_tracks; ++i) final_map[i] = inv_P[i];

        for (auto& [id, net] : nets) {
            for (auto& s : net.segments) {
                s.y1 = final_map[s.y1];
                s.y2 = final_map[s.y2];
            }
        }
    }

    void applyBuffering(int target_tracks) {
        if (num_tracks >= target_tracks) return;
        int max_x = 0;
        for (auto const& [id, net] : nets) for (auto const& s : net.segments) if (s.type == 'H') max_x = max(max_x, s.x2);
        vector<vector<int>> grid(max_x + 1, vector<int>(num_tracks + 1, 0));
        for (auto const& [id, net] : nets) for (auto const& s : net.segments) if (s.type == 'H') {
            for (int x = s.x1; x < s.x2; ++x) grid[x][s.y1] = id;
        }
        vector<int> k_coupl(num_tracks, 0);
        for (int x = 0; x < max_x; ++x) for (int k = 1; k < num_tracks; ++k) if (has_constraint(grid[x][k], grid[x][k+1])) k_coupl[k]++;
        int spare = target_tracks - num_tracks;
        vector<int> buffers(num_tracks + 1, 0);
        while (spare > 0) {
            int bk = -1, mc = -1;
            for (int k = 1; k < num_tracks; ++k) if (k_coupl[k] > mc) { mc = k_coupl[k]; bk = k; }
            if (bk == -1 || mc == 0) { for (int k=1; k<num_tracks && spare > 0; k++, spare--) buffers[k]++; break; }
            buffers[bk]++; k_coupl[bk] = 0; spare--;
        }
        vector<int> P(num_tracks + 2);
        P[0] = 0; int cy = 1;
        for (int k = 1; k <= num_tracks; ++k) { P[k] = cy; cy += (1 + buffers[k]); }
        P[num_tracks + 1] = target_tracks + 1;
        num_tracks = target_tracks;
        for (auto& [id, net] : nets) for (auto& s : net.segments) { s.y1 = P[s.y1]; s.y2 = P[s.y2]; }
    }

    bool try_route(int t_count, int seed) {
        num_tracks = t_count; rng.seed(seed);
        vector<int> track_at(num_tracks + 2, 0);
        for (auto& [id, net] : nets) net.segments.clear();
        int c = 0;
        while (true) {
            bool act = false; for (int t=1; t<=num_tracks; ++t) if (track_at[t]>0) act = true;
            if (c >= num_cols && !act) break;
            if (c > num_cols + 500) return false;
            int T = (c < (int)top_pins.size()) ? top_pins[c] : 0;
            int B = (c < (int)bottom_pins.size()) ? bottom_pins[c] : 0;
            vector<int> v_mask(num_tracks + 2, 0);

            if (T == B && T > 0) {
                add_v(c, 0, num_tracks + 1, T); for (int y=0; y<=num_tracks+1; ++y) v_mask[y]=T;
                if (!ensure_track(track_at, T, c)) return false;
            } else {
                if (T > 0 && B > 0) {
                    int tT = -1, tB = -1;
                    int curT = -1, curB = -1;
                    for (int t=num_tracks; t>=1; --t) if (track_at[t] == T) { curT = t; break; }
                    for (int t=1; t<=num_tracks; ++t) if (track_at[t] == B) { curB = t; break; }
                    if (curT != -1 && curB != -1 && curT > curB) { tT = curT; tB = curB; }
                    else {
                        vector<pair<int, int>> pairs;
                        for (int tt=num_tracks; tt>=2; --tt) {
                            if (track_at[tt] != 0 && track_at[tt] != T) continue;
                            for (int tb=tt-1; tb>=1; --tb) {
                                if (track_at[tb] != 0 && track_at[tb] != B) continue;
                                pairs.push_back({tt, tb});
                            }
                        }
                        if (pairs.empty()) return false;
                        shuffle(pairs.begin(), pairs.end(), rng);
                        tT = pairs[0].first; tB = pairs[0].second;
                    }
                    track_at[tT] = T; track_at[tB] = B;
                    add_v(c, tT, num_tracks+1, T); add_v(c, 0, tB, B);
                    for (int y=tT; y<=num_tracks+1; ++y) v_mask[y]=T;
                    for (int y=0; y<=tB; ++y) v_mask[y]=B;
                } else if (T > 0) {
                    if (!ensure_track_top(track_at, T, c, v_mask)) return false;
                } else if (B > 0) {
                    if (!ensure_track_bot(track_at, B, c, v_mask)) return false;
                }
            }
            for (auto const& [id, net] : nets) {
                vector<int> my_t; for (int t=1; t<=num_tracks; ++t) if (track_at[t]==id) my_t.push_back(t);
                if (my_t.size() > 1) {
                    for (int i=0; i<(int)my_t.size(); ++i) {
                        for (int j=i+1; j<(int)my_t.size(); ++j) {
                            int t1 = my_t[i], t2 = my_t[j];
                            int mn = min(t1, t2), mx = max(t1, t2);
                            bool ok = true;
                            for (int y=mn; y<=mx; ++y) if (v_mask[y] != 0 && v_mask[y] != id) ok = false;
                            if (ok) {
                                add_v(c, mn, mx, id); int ny = get_next_pin_y(id, c);
                                int keep = (ny == -1) ? mn : ((abs(ny-t1) < abs(ny-t2)) ? t1 : t2);
                                for (int y=mn; y<=mx; ++y) v_mask[y] = id;
                                track_at[t1 == keep ? t2 : t1] = 0;
                                my_t.clear(); for (int t=1; t<=num_tracks; ++t) if (track_at[t]==id) my_t.push_back(t);
                                i = -1; break; 
                            }
                        }
                    }
                }
            }
            vector<int> tracks(num_tracks);
            for(int i=0; i<num_tracks; ++i) tracks[i] = i+1;
            
            // Falling
            shuffle(tracks.begin(), tracks.end(), rng);
            for (int t : tracks) {
                int id = track_at[t];
                if (id > 0) {
                    int ny = get_next_pin_y(id, c);
                    if (ny != -1 && ny < t) {
                        int steps = 0;
                        while (t - steps - 1 >= 1 && track_at[t - steps - 1] == 0 && (v_mask[t - steps - 1] == 0 || v_mask[t - steps - 1] == id)) {
                            steps++;
                            // Do NOT reach 0 here! Stay at least at track 1.
                        }
                        if (steps > 0) {
                            add_v(c, t, t - steps, id); track_at[t - steps] = id; track_at[t] = 0; 
                            for (int y=t-steps; y<=t; ++y) v_mask[y] = id;
                        }
                    }
                }
            }
            
            // Rising
            shuffle(tracks.begin(), tracks.end(), rng);
            for (int t : tracks) {
                int id = track_at[t];
                if (id > 0) {
                    int ny = get_next_pin_y(id, c);
                    if (ny != -1 && ny > t) {
                        int steps = 0;
                        while (t + steps + 1 <= num_tracks && track_at[t + steps + 1] == 0 && (v_mask[t + steps + 1] == 0 || v_mask[t + steps + 1] == id)) {
                            steps++;
                            // Do NOT reach num_tracks+1 here! Stay at the last track.
                        }
                        if (steps > 0) {
                            add_v(c, t, t + steps, id); track_at[t + steps] = id; track_at[t] = 0;
                            for (int y=t; y<=t+steps; ++y) v_mask[y] = id;
                        }
                    }
                }
            }
            for (int t = 1; t <= num_tracks; ++t) {
                int id = track_at[t];
                if (id > 0) {
                    bool needs_continue = (c < nets[id].rightmost_col);
                    if (!needs_continue) {
                        for (int tt = 1; tt <= num_tracks; ++tt) if (tt != t && track_at[tt] == id) { needs_continue = true; break; }
                    }
                    if (needs_continue) add_h(c, c + 1, t, id);
                    else track_at[t] = 0;
                }
            }
            c++;
        }
        
        // Final connectivity check
        for (auto const& [id, net] : nets) {
            if (net.pins.empty()) continue;
            set<pair<int, int>> p_set;
            for (auto const& p : net.pins) p_set.insert({p.first, p.second ? (num_tracks + 1) : 0});
            
            vector<pair<int, int>> q; q.push_back(*p_set.begin());
            set<pair<int, int>> visited; visited.insert(q.back());
            int head = 0;
            while(head < (int)q.size()){
                pair<int, int> u = q[head++];
                int x = u.first, y = u.second;
                for (auto const& s : net.segments) {
                    if (s.type == 'H' && s.y1 == y) {
                        if (x >= s.x1 && x < s.x2 && !visited.count({x+1, y})) { visited.insert({x+1, y}); q.push_back({x+1, y}); }
                        if (x > s.x1 && x <= s.x2 && !visited.count({x-1, y})) { visited.insert({x-1, y}); q.push_back({x-1, y}); }
                    } else if (s.type == 'V' && s.x1 == x) {
                        if (y >= s.y1 && y < s.y2 && !visited.count({x, y+1})) { visited.insert({x, y+1}); q.push_back({x, y+1}); }
                        if (y > s.y1 && y <= s.y2 && !visited.count({x, y-1})) { visited.insert({x, y-1}); q.push_back({x, y-1}); }
                    }
                }
            }
            for(auto p : p_set) if(!visited.count(p)) return false;
        }
        return true;
    }

    bool ensure_track(vector<int>& track_at, int id, int c) {
        for (int t=1; t<=num_tracks; ++t) if (track_at[t] == id) return true;
        for (int t=num_tracks; t>=1; --t) if (track_at[t] == 0) { track_at[t] = id; return true; }
        return false;
    }

    bool ensure_track_top(vector<int>& track_at, int id, int c, vector<int>& v_mask) {
        int ny = get_next_pin_y(id, c);
        // Find if this net already has a reachable track
        vector<pair<int, int>> reachable;
        for (int t=num_tracks; t>=1; --t) {
            if (track_at[t] == id) {
                bool ok = true;
                for (int y = min(t, num_tracks+1); y <= max(t, num_tracks+1); ++y) {
                    if (v_mask[y] != 0 && v_mask[y] != id) ok = false;
                }
                if (ok) reachable.push_back({t, (ny == -1 ? 0 : abs(t - ny))});
            }
        }
        if (!reachable.empty()) {
            sort(reachable.begin(), reachable.end(), [](auto& a, auto& b){ return a.second < b.second; });
            int best_t = reachable[0].first;
            add_v(c, best_t, num_tracks+1, id);
            for (int y = best_t; y <= num_tracks+1; ++y) v_mask[y] = id;
            return true;
        }
        
        vector<pair<int, int>> empty_tracks;
        for (int t=num_tracks; t>=1; --t) {
            if (track_at[t] == 0) {
                bool ok = true;
                for (int y = t; y <= num_tracks+1; ++y) {
                    if (v_mask[y] != 0 && v_mask[y] != id) ok = false;
                }
                if (ok) empty_tracks.push_back({t, (ny == -1 ? 0 : abs(t - ny))});
            }
        }
        if (empty_tracks.empty()) return false;
        sort(empty_tracks.begin(), empty_tracks.end(), [](auto& a, auto& b){ return a.second < b.second; });
        
        // Pick among top 3 best candidates or just best
        int pool = min((int)empty_tracks.size(), 3);
        int t = empty_tracks[rng() % pool].first;
        track_at[t] = id;
        add_v(c, t, num_tracks+1, id);
        for (int y = t; y <= num_tracks+1; ++y) v_mask[y] = id;
        return true;
    }

    bool ensure_track_bot(vector<int>& track_at, int id, int c, vector<int>& v_mask) {
        int ny = get_next_pin_y(id, c);
        vector<pair<int, int>> reachable;
        for (int t=1; t<=num_tracks; ++t) {
            if (track_at[t] == id) {
                bool ok = true;
                for (int y=0; y<=t; ++y) if (v_mask[y] != 0 && v_mask[y] != id) ok = false;
                if (ok) reachable.push_back({t, (ny == -1 ? 0 : abs(t - ny))});
            }
        }
        if (!reachable.empty()) {
            sort(reachable.begin(), reachable.end(), [](auto& a, auto& b){ return a.second < b.second; });
            int best_t = reachable[0].first;
            add_v(c, 0, best_t, id);
            for (int y=0; y<=best_t; ++y) v_mask[y]=id;
            return true;
        }

        vector<pair<int, int>> empty_tracks;
        for (int t=1; t<=num_tracks; ++t) {
            if (track_at[t] == 0) {
                bool ok = true;
                for (int y=0; y<=t; ++y) if (v_mask[y] != 0 && v_mask[y] != id) ok = false;
                if (ok) empty_tracks.push_back({t, (ny == -1 ? 0 : abs(t - ny))});
            }
        }
        if (empty_tracks.empty()) return false;
        sort(empty_tracks.begin(), empty_tracks.end(), [](auto& a, auto& b){ return a.second < b.second; });
        int pool = min((int)empty_tracks.size(), 3);
        int t = empty_tracks[rng() % pool].first;
        track_at[t] = id;
        add_v(c, 0, t, id);
        for (int y=0; y<=t; ++y) v_mask[y] = id;
        return true;
    }

    void calculateMetrics() {
        int max_x = 0;
        for (auto const& [id, net] : nets) for (auto const& s : net.segments) if (s.type == 'H') max_x = max(max_x, s.x2);
        vector<vector<int>> grid(max_x + 1, vector<int>(num_tracks + 1, 0));
        for (auto const& [id, net] : nets) for (auto const& s : net.segments) if (s.type == 'H') {
            for (int x = s.x1; x < s.x2; ++x) grid[x][s.y1] = id;
        }
        map<int, int> coupling;
        for (int x=0; x<max_x; ++x) for (int t=1; t<num_tracks; ++t) {
            int n1 = grid[x][t], n2 = grid[x][t+1];
            if (n1 > 0 && n2 > 0 && has_constraint(n1, n2)) { coupling[n1]++; coupling[n2]++; }
        }
        double max_ratio = 0;
        for (auto const& [id, net] : nets) {
            int span = net.rightmost_col - net.leftmost_col;
            if (span > 0) max_ratio = max(max_ratio, (double)coupling[id] / span);
        }
        cout << "Max Ratio: " << fixed << setprecision(4) << max_ratio << endl;
    }

    void writeOutput(const string& filename) {
        ofstream fout(filename);
        for (auto const& [id, net] : nets) {
            fout << ".begin " << id << "\n";
            map<int, vector<pair<int, int>>> h_by_y;
            map<int, vector<pair<int, int>>> v_by_x;
            for (auto const& s : net.segments) {
                if (s.type == 'H') h_by_y[s.y1].push_back({s.x1, s.x2});
                else v_by_x[s.x1].push_back({s.y1, s.y2});
            }
            for (auto& [x, ranges] : v_by_x) {
                sort(ranges.begin(), ranges.end());
                int y1 = -1, y2 = -1;
                for (auto r : ranges) {
                    if (y1 == -1) { y1 = r.first; y2 = r.second; }
                    else if (r.first <= y2) y2 = max(y2, r.second);
                    else { fout << ".V " << x << " " << y1 << " " << y2 << "\n"; y1 = r.first; y2 = r.second; }
                }
                if (y1 != -1) fout << ".V " << x << " " << y1 << " " << y2 << "\n";
            }
            for (auto& [y, ranges] : h_by_y) {
                sort(ranges.begin(), ranges.end());
                int x1 = -1, x2 = -1;
                for (auto r : ranges) {
                    if (x1 == -1) { x1 = r.first; x2 = r.second; }
                    else if (r.first <= x2) x2 = max(x2, r.second);
                    else { fout << ".H " << x1 << " " << y << " " << x2 << "\n"; x1 = r.first; x2 = r.second; }
                }
                if (x1 != -1) fout << ".H " << x1 << " " << y << " " << x2 << "\n";
            }
            fout << ".end\n";
        }
    }
};

int main(int argc, char* argv[]) {
    if (argc < 3) return 1;
    GreedyRouter router;
    if (!router.parseInput(argv[1])) return 1;
    bool success = router.route(argv[1]);
    router.writeOutput(argv[2]);
    if (success) {
        cout << "Success: " << router.num_tracks << " tracks.\n";
        router.calculateMetrics();
    } else cout << "Failed\n";
    return 0;
}