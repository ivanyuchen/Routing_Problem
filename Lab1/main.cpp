#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>
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
};

struct Net {
    int id;
    int rightmost_col = 0; 
    int min_col = 1e9;
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
    vector<int> tracks; 

public:
    ChannelRouter(int num_tracks) : num_tracks(num_tracks) {
        tracks.resize(num_tracks + 2, 0); 
    }

    int getDensity() {
        int max_d = 0;
        int max_col = max(top_pins.size(), bottom_pins.size());
        for (int c = 0; c < max_col; ++c) {
            int d = 0;
            for (const auto& pair : nets) {
                if (c >= pair.second.min_col && c <= pair.second.rightmost_col) {
                    d++;
                }
            }
            max_d = max(max_d, d);
        }
        return max_d;
    }

    bool parseInput(const vector<string>& lines) {
        if (lines.empty()) return false;
        
        stringstream ss0(lines[0]);
        ss0 >> alpha >> beta >> gamma >> delta;

        stringstream ss1(lines[1]);
        int pin;
        while (ss1 >> pin) top_pins.push_back(pin);

        stringstream ss2(lines[2]);
        while (ss2 >> pin) bottom_pins.push_back(pin);

        stringstream ss3(lines[3]);
        int constraint_num;
        ss3 >> constraint_num;
        
        for (int i = 0; i < constraint_num; ++i) {
            stringstream css(lines[4 + i]);
            int n1, n2;
            css >> n1 >> n2;
            constraints.push_back({n1, n2});
        }

        for (int i = 0; i < top_pins.size(); ++i) {
            if (top_pins[i] > 0) {
                nets[top_pins[i]].rightmost_col = max(nets[top_pins[i]].rightmost_col, i);
                if (nets[top_pins[i]].min_col == 0 && nets[top_pins[i]].rightmost_col == 0) nets[top_pins[i]].min_col = i;
                else nets[top_pins[i]].min_col = min(nets[top_pins[i]].min_col, i);
            }
        }
        for (int i = 0; i < bottom_pins.size(); ++i) {
            if (bottom_pins[i] > 0) {
                nets[bottom_pins[i]].rightmost_col = max(nets[bottom_pins[i]].rightmost_col, i);
                if (nets[bottom_pins[i]].min_col == 0 && nets[bottom_pins[i]].rightmost_col == 0) nets[bottom_pins[i]].min_col = i;
                else nets[bottom_pins[i]].min_col = min(nets[bottom_pins[i]].min_col, i);
            }
        }
        return true;
    }

    bool check_v(int y1, int y2, int net_id, const vector<bool>& v_occupied) {
        int mn = min(y1, y2), mx = max(y1, y2);
        for (int i = mn; i <= mx; ++i) if (v_occupied[i]) return false; 
        return true;
    }

    void add_v(int col, int y1, int y2, int net_id, vector<bool>& v_occupied) {
        int mn = min(y1, y2), mx = max(y1, y2);
        for (int i = mn; i <= mx; ++i) v_occupied[i] = true;
        nets[net_id].segments.push_back({'V', col, mn, col, mx});
    }

    // ===== Net Ordering by Density & Span =====
    vector<int> getNetOrderByDensity() {
        map<int, pair<int, int>> net_bounds; // net_id -> (min_col, max_col)
        
        for (int i = 0; i < top_pins.size(); ++i) {
            if (top_pins[i] > 0) {
                net_bounds[top_pins[i]].first = min(net_bounds[top_pins[i]].first, i);
                net_bounds[top_pins[i]].second = max(net_bounds[top_pins[i]].second, i);
            }
        }
        for (int i = 0; i < bottom_pins.size(); ++i) {
            if (bottom_pins[i] > 0) {
                net_bounds[bottom_pins[i]].first = min(net_bounds[bottom_pins[i]].first, i);
                net_bounds[bottom_pins[i]].second = max(net_bounds[bottom_pins[i]].second, i);
            }
        }
        
        vector<pair<double, int>> net_scores;
        for (const auto& [net_id, bounds] : net_bounds) {
            int span = bounds.second - bounds.first + 1;
            double density = 2.0 / max(1, span); // higher = more constrained
            
            // Check if net has constraints
            for (const auto& c : constraints) {
                if (c.from_net == net_id || c.to_net == net_id) {
                    density += 1.0; // boost score for constrained nets
                    break;
                }
            }
            
            net_scores.push_back({density, net_id});
        }
        
        // Sort by density descending (constrained nets first)
        sort(net_scores.rbegin(), net_scores.rend());
        
        vector<int> ordered_nets;
        for (const auto& [score, net_id] : net_scores) {
            ordered_nets.push_back(net_id);
        }
        return ordered_nets;
    }
    
    bool route(bool ignore_constraints = false, int seed = 0) {
        mt19937 rng(seed); 
        
        int top_row_y = num_tracks + 1;
        int col = 0;
        int mid = num_tracks / 2;

        set<pair<int, int>> bad_pairs;
        for (const auto& c : constraints) bad_pairs.insert({min(c.from_net, c.to_net), max(c.from_net, c.to_net)});

        auto is_conflict = [&](int n1, int n2) {
            if (ignore_constraints) return false; 
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

            auto process_top = [&]() {
                if (T <= 0) return true;
                bool connected = false;
                auto try_assign = [&](bool check_neighbors) {
                    for (int i = mid; i <= num_tracks; ++i) if (tracks[i] == T && check_v(i, top_row_y, T, v_occupied))
                        if (!check_neighbors || (!is_conflict(T, tracks[i-1]) && !is_conflict(T, tracks[i+1]))) { add_v(col, i, top_row_y, T, v_occupied); return true; }
                    for (int i = mid; i <= num_tracks; ++i) if (tracks[i] == 0 && check_v(i, top_row_y, T, v_occupied))
                        if (!check_neighbors || (!is_conflict(T, tracks[i-1]) && !is_conflict(T, tracks[i+1]))) { tracks[i] = T; add_v(col, i, top_row_y, T, v_occupied); return true; }
                    for (int i = mid - 1; i >= 1; --i) if (tracks[i] == T && check_v(i, top_row_y, T, v_occupied))
                        if (!check_neighbors || (!is_conflict(T, tracks[i-1]) && !is_conflict(T, tracks[i+1]))) { add_v(col, i, top_row_y, T, v_occupied); return true; }
                    for (int i = mid - 1; i >= 1; --i) if (tracks[i] == 0 && check_v(i, top_row_y, T, v_occupied))
                        if (!check_neighbors || (!is_conflict(T, tracks[i-1]) && !is_conflict(T, tracks[i+1]))) { tracks[i] = T; add_v(col, i, top_row_y, T, v_occupied); return true; }
                    return false;
                };
                connected = try_assign(true);  
                if (!connected) connected = try_assign(false); 
                return connected;
            };

            auto process_bot = [&]() {
                if (B <= 0) return true;
                bool connected = false;
                auto try_assign_b = [&](bool check_neighbors) {
                    for (int i = mid + 1; i >= 1; --i) if (tracks[i] == B && check_v(0, i, B, v_occupied))
                        if (!check_neighbors || (!is_conflict(B, tracks[i-1]) && !is_conflict(B, tracks[i+1]))) { add_v(col, 0, i, B, v_occupied); return true; }
                    for (int i = mid + 1; i >= 1; --i) if (tracks[i] == 0 && check_v(0, i, B, v_occupied))
                        if (!check_neighbors || (!is_conflict(B, tracks[i-1]) && !is_conflict(B, tracks[i+1]))) { tracks[i] = B; add_v(col, 0, i, B, v_occupied); return true; }
                    for (int i = mid + 2; i <= num_tracks; ++i) if (tracks[i] == B && check_v(0, i, B, v_occupied))
                        if (!check_neighbors || (!is_conflict(B, tracks[i-1]) && !is_conflict(B, tracks[i+1]))) { add_v(col, 0, i, B, v_occupied); return true; }
                    for (int i = mid + 2; i <= num_tracks; ++i) if (tracks[i] == 0 && check_v(0, i, B, v_occupied))
                        if (!check_neighbors || (!is_conflict(B, tracks[i-1]) && !is_conflict(B, tracks[i+1]))) { tracks[i] = B; add_v(col, 0, i, B, v_occupied); return true; }
                    return false;
                };
                connected = try_assign_b(true); 
                if (!connected) connected = try_assign_b(false);
                return connected;
            };

            if (rng() % 2 == 0) {
                if (!process_top() || !process_bot()) return false;
            } else {
                if (!process_bot() || !process_top()) return false;
            }

            vector<int> net_keys;
            for (auto const& pair : nets) net_keys.push_back(pair.first);
            shuffle(net_keys.begin(), net_keys.end(), rng); 

            for (int n : net_keys) {
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

            vector<int> trk_keys;
            for(int i = 1; i <= num_tracks; ++i) trk_keys.push_back(i);
            shuffle(trk_keys.begin(), trk_keys.end(), rng); 

            for (int i : trk_keys) {
                int n = tracks[i]; if (n <= 0) continue;
                
                if (rng() % 100 < 5) continue; 

                if (i > mid + 1 && tracks[i-1] == 0 && check_v(i, i-1, n, v_occupied)) {
                    if (ignore_constraints || i == num_tracks || !is_conflict(n, tracks[i-2])) { 
                        add_v(col, i, i-1, n, v_occupied);
                        tracks[i-1] = n; tracks[i] = 0;
                    }
                } 
                else if (i < mid - 1 && tracks[i+1] == 0 && check_v(i, i+1, n, v_occupied)) {
                    if (ignore_constraints || i == 1 || !is_conflict(n, tracks[i+2])) {
                        add_v(col, i, i+1, n, v_occupied);
                        tracks[i+1] = n; tracks[i] = 0;
                    }
                }
            }

            for (int i = 1; i <= num_tracks; ++i) {
                int n = tracks[i];
                if (n > 0 && col >= nets[n].rightmost_col) {
                    int count = 0;
                    for (int j = 1; j <= num_tracks; ++j) if (tracks[j] == n) count++;
                    if (count == 1) tracks[i] = 0; 
                }
            }
            for (int i = 1; i <= num_tracks; ++i) {
                if (tracks[i] > 0) nets[tracks[i]].segments.push_back({'H', col, i, col + 1, i});
            }
            col++;
        }
        return true;
    }

    double calculateCost() {
        double total_length = 0;
        int total_vias = 0;
        int max_col = 0;

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

        int coupling_length = 0;
        set<pair<int, int>> bad_pairs;
        for (const auto& c : constraints) bad_pairs.insert({min(c.from_net, c.to_net), max(c.from_net, c.to_net)});

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
                if (seg.type == 'V') fout << ".V " << seg.x1 << " " << seg.y1 << " " << seg.y2 << "\n";
                else h_segs.push_back(seg);
            }

            sort(h_segs.begin(), h_segs.end(), [](const Segment& a, const Segment& b){
                if (a.y1 != b.y1) return a.y1 < b.y1;
                return a.x1 < b.x1;
            });

            vector<Segment> merged_h;
            for (const auto& s : h_segs) {
                if (merged_h.empty()) merged_h.push_back(s);
                else {
                    auto& last = merged_h.back();
                    if (last.y1 == s.y1 && last.x2 == s.x1) last.x2 = s.x2; 
                    else merged_h.push_back(s);
                }
            }

            for (const auto& s : merged_h) fout << ".H " << s.x1 << " " << s.y1 << " " << s.x2 << "\n";
            fout << ".end\n";
        }
        fout.close();
    }
};

int main(int argc, char* argv[]) {
    if (argc < 3) {
        cerr << "Usage: " << argv[0] << " <input_file> <output_file>\n";
        return 1;
    }

    vector<string> file_lines;
    ifstream fin(argv[1]);
    string line;
    while(getline(fin, line)) file_lines.push_back(line);
    fin.close();

    auto start_time = chrono::high_resolution_clock::now();
    
    int best_tracks = -1;
    double min_cost = 1e15; 
    int best_seed = 0;
    bool best_use_ignore_mode = false;
    ChannelRouter dummy(1);
    dummy.parseInput(file_lines);
    int density = dummy.getDensity();
    int min_search_tracks = density;
    int max_search_tracks = density + 15;
    cout << "   網路密度下限 (Channel Density): " << density << " 軌\n";

    int time_limit_seconds = 280; // Leave 20s buffer before 300s hard limit
    vector<int> track_attempts(max_search_tracks + 1, 0);
    map<int, double> best_cost_per_track;

    cout << "🚀 [改善型智慧搜尋 V1.1] 啟動！\n";
    cout << "   策略：密度排序 + 前瞻評估 + 耦合代價\n";
    cout << "   極限時間 " << time_limit_seconds << " 秒...\n\n";
    
    // Phase 1: Quick exploration
    cout << "📊 Phase 1: 快速探索軌道數 (耦合優先)...\n";
    
    for (int tracks = min_search_tracks; tracks <= max_search_tracks; ++tracks) {
        int max_seeds = (tracks <= min_search_tracks + 3) ? 20000 : (tracks <= min_search_tracks + 6) ? 10000 : 5000;
        
        for (int seed = 1; seed <= max_seeds; ++seed) {
            auto current_time = chrono::high_resolution_clock::now();
            if (chrono::duration_cast<chrono::seconds>(current_time - start_time).count() > time_limit_seconds * 0.65) {
                cout << "\n⏱️ Phase 1 時間到，進入 Phase 2...\n";
                goto PHASE2;
            }
            
            bool success = false;
            
            // Attempt with constraints first
            ChannelRouter router(tracks);
            router.parseInput(file_lines);
            if (router.route(false, seed)) {
                success = true;
            } else {
                // Fallback to ignore constraints
                ChannelRouter router_fallback(tracks);
                router_fallback.parseInput(file_lines);
                if (router_fallback.route(true, seed)) {
                    success = true;
                    router = router_fallback;
                }
            }
            
            if (success) {
                double current_cost = router.calculateCost();
                track_attempts[tracks]++;
                
                if (current_cost < min_cost) {
                    min_cost = current_cost;
                    best_tracks = tracks;
                    best_seed = seed;
                    best_use_ignore_mode = false;
                    best_cost_per_track[tracks] = current_cost;
                    
                    cout << "   👑 新紀錄！ Tracks: " << tracks << " | Seed: " << seed 
                         << " | Cost: " << min_cost << "\n";
                    
                    // Write immediately
                    ChannelRouter temp_router(best_tracks);
                    temp_router.parseInput(file_lines);
                    temp_router.route(best_use_ignore_mode, best_seed);
                    temp_router.mergeAndWriteOutput(argv[2]);
                }
            }
        }
    }
    
PHASE2:
    // Phase 2: Deep optimization on best track count + adjacent track counts
    cout << "\n🎯 Phase 2: 深度優化最佳軌道數 + 鄰近軌道...\n";
    if (best_tracks != -1) {
        cout << "   目標軌道數: " << best_tracks << " 軌 (及 " << best_tracks-1 << ", " << best_tracks+1 << ")\n";
        
        // Also try adjacent track counts with fewer seeds
        vector<int> track_list = {best_tracks};
        if (best_tracks > min_search_tracks) track_list.push_back(best_tracks - 1);
        if (best_tracks < max_search_tracks) track_list.push_back(best_tracks + 1);
        
        for (int track_count : track_list) {
            int deep_seeds = (track_count == best_tracks) ? 150000 : 30000;
            int initial_seed = (track_count == best_tracks) ? best_seed : 1;
            
            for (int seed = initial_seed + 1; seed <= initial_seed + deep_seeds; ++seed) {
                auto current_time = chrono::high_resolution_clock::now();
                if (chrono::duration_cast<chrono::seconds>(current_time - start_time).count() > time_limit_seconds) {
                    cout << "\n⏱️ 達到時間限制！\n";
                    goto END_SEARCH;
                }
                
                ChannelRouter router(track_count);
                router.parseInput(file_lines);
                if (router.route(best_use_ignore_mode, seed)) {
                    double current_cost = router.calculateCost();
                    if (current_cost < min_cost) {
                        min_cost = current_cost;
                        best_seed = seed;
                        best_tracks = track_count;
                        
                        cout << "   ✨ 更新！ T:" << track_count << " | Cost: " << min_cost << "\n";
                        
                        ChannelRouter temp_router(best_tracks);
                        temp_router.parseInput(file_lines);
                        temp_router.route(best_use_ignore_mode, best_seed);
                        temp_router.mergeAndWriteOutput(argv[2]);
                    }
                }
            }
        }
    }

END_SEARCH:
    auto end_time = chrono::high_resolution_clock::now();
    int elapsed = chrono::duration_cast<chrono::seconds>(end_time - start_time).count();

    cout << "\n--------------------------------------------------\n";
    if (best_tracks != -1) {
        cout << "🎉 搜尋完成！耗時 " << elapsed << " 秒。\n";
        cout << "👉 最佳軌道數: " << best_tracks << " 軌\n";
        cout << "👉 最佳種子數: " << best_seed << "\n";
        cout << "👉 最低總成本: " << min_cost << "\n";
        cout << "✅ 結果已保存至: " << argv[2] << "\n";
    } else {
        cout << "❌ 錯誤：無法完成。\n";
    }

    return 0;
}