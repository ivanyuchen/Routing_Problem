#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>
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
    int rightmost_col; 
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
            if (top_pins[i] > 0) nets[top_pins[i]].rightmost_col = max(nets[top_pins[i]].rightmost_col, i);
        }
        for (int i = 0; i < bottom_pins.size(); ++i) {
            if (bottom_pins[i] > 0) nets[bottom_pins[i]].rightmost_col = max(nets[bottom_pins[i]].rightmost_col, i);
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
    bool use_ignore_mode = false; 

    // 🔥 Kaggle 煉丹模式：1小時 (3600秒)、每一軌 1000 萬次
    int max_search_tracks = 30;
    long long iterations_per_track = 10000000;
    int time_limit_seconds = 3600; 

    cout << "🚀 [無限制本地煉丹模式] 啟動！極限時間拉長至 " << time_limit_seconds << " 秒...\n";
    cout << "💡 提示：你可以隨時按下 Ctrl+C 終止，或者等它找到滿意的分數為止。\n";
    
    int min_possible_track = max_search_tracks + 1;
    for(int tracks = 20; tracks <= max_search_tracks; ++tracks) {
        ChannelRouter router(tracks);
        router.parseInput(file_lines);
        if (router.route(false, 0)) {
            min_possible_track = tracks;
            break;
        }
    }

    bool require_ignore_mode = false;
    if (min_possible_track > max_search_tracks) {
        cout << "⚠️ 避嫌模式完全無法在 30 軌內連通，切換至 [保命硬擠模式] 暴搜...\n";
        require_ignore_mode = true;
        min_possible_track = 20; 
    } else {
        cout << "✅ 最小可行軌道約為 " << min_possible_track << " 軌，準備開始刷分！\n";
    }

    long long total_searches = 0;
    for (int tracks = min_possible_track; tracks <= max_search_tracks; ++tracks) {
        for (long long seed = 1; seed <= iterations_per_track; ++seed) {
            
            auto current_time = chrono::high_resolution_clock::now();
            if (chrono::duration_cast<chrono::seconds>(current_time - start_time).count() > time_limit_seconds) {
                cout << "\n⏱️ 達到設定的時間限制 (" << time_limit_seconds << " 秒)，自動煞車！\n";
                goto END_SEARCH;
            }

            ChannelRouter router(tracks);
            router.parseInput(file_lines);
            
            if (router.route(require_ignore_mode, seed)) {
                double current_cost = router.calculateCost();
                if (current_cost < min_cost) {
                    min_cost = current_cost;
                    best_tracks = tracks;
                    best_seed = seed;
                    use_ignore_mode = require_ignore_mode;
                    cout << "   👑 發現新紀錄！ Tracks: " << tracks << " | Seed: " << seed << " | Cost 降至: " << min_cost << "\n";
                    
                    // 每次破紀錄就順便寫出檔案，這樣就算你提早 Ctrl+C 終止，也能拿到破紀錄的檔案！
                    ChannelRouter temp_router(best_tracks);
                    temp_router.parseInput(file_lines);
                    temp_router.route(use_ignore_mode, best_seed);
                    temp_router.mergeAndWriteOutput(argv[2]);
                }
            }
            total_searches++;
        }
    }

END_SEARCH:
    auto end_time = chrono::high_resolution_clock::now();
    int elapsed = chrono::duration_cast<chrono::seconds>(end_time - start_time).count();

    cout << "--------------------------------------------------\n";
    if (best_tracks != -1) {
        cout << "🎉 煉丹結束！總共嘗試了 " << total_searches << " 種走線排列，耗時 " << elapsed << " 秒。\n";
        cout << "👉 冠軍軌道數: " << best_tracks << " 軌 (Seed: " << best_seed << ")\n";
        cout << "👉 史低總成本: " << min_cost << "\n";
        cout << "✅ 檔案已即時更新至: " << argv[2] << "。上傳 Kaggle 拿高分吧！\n";
        cout << "⚠️ 注意：交作業給助教前，記得把 time_limit_seconds 改回 285，不然 E3 系統會 TLE 給 0 分喔！\n";
    } else {
        cout << "❌ 錯誤：就算完全不避嫌且暴搜，還是無法在 " << max_search_tracks << " 軌內完成。\n";
    }

    return 0;
}