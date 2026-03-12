#include "routing_structure.h"

int main(int argc, char* argv[]) {
    if (argc < 3) {
        cerr << "Usage: " << argv[0] << " <input_file> <output_file>\n";
        return 1;
    }

    int best_tracks = -1;
    double min_cost = 1e15; // 初始超大 Cost

    cout << "🚀 正在進行 Design Space Exploration...\n";

    // 從 20 軌開始慢慢往上找，找到 Cost 最低的那個
    for(int tracks = 20; tracks <= 1000; ++tracks) {
        ChannelRouter router(tracks);
        if (!router.parseInput(argv[1])) return 1;
        
        if (router.route()) {
            double cost = router.calculateCost();
            // 可以取消註解下面這行，觀察每一種軌道數的分數變化
            // cout << "Tracks: " << tracks << " | Cost: " << cost << "\n";
            
            if (cost < min_cost) {
                min_cost = cost;
                best_tracks = tracks;
            }
        }
    }

    if (best_tracks != -1) {
        cout << "\n🎉 探索完成！\n";
        cout << "👉 最佳軌道數 (Best Tracks): " << best_tracks << "\n";
        cout << "👉 最低成本 (Min Cost): " << min_cost << "\n";

        // 用找到的冠軍軌道數，產生最終的 output 檔案
        ChannelRouter final_router(best_tracks);
        final_router.parseInput(argv[1]);
        final_router.route();
        final_router.mergeAndWriteOutput(argv[2]);
        cout << "✅ 最佳繞線結果已輸出至: " << argv[2] << "\n";
    } else {
        cout << "❌ 錯誤：無法在 20~80 軌內完成繞線。\n";
    }

    return 0;
}