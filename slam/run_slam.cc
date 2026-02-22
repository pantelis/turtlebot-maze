/**
 * Minimal stella_vslam driver for directory-based frame input.
 *
 * Watches a frame directory for new images (written by slam_bridge.py),
 * feeds them to stella_vslam RGBD pipeline, and writes pose estimates
 * to stdout as JSON lines.
 *
 * Usage:
 *   run_slam -v vocab.fbow -c config.yaml -d /path/to/frames [--viewer socket_publisher]
 */

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>
#include <string>
#include <thread>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <stella_vslam/config.h>
#include <stella_vslam/system.h>

namespace fs = std::filesystem;

int main(int argc, char* argv[]) {
    // Simple arg parsing
    std::string vocab_path, config_path, frame_dir, viewer_type;
    std::string map_db_out = "/tmp/slam_map.msg";

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if ((arg == "-v" || arg == "--vocab") && i + 1 < argc) {
            vocab_path = argv[++i];
        } else if ((arg == "-c" || arg == "--config") && i + 1 < argc) {
            config_path = argv[++i];
        } else if ((arg == "-d" || arg == "--frame-dir") && i + 1 < argc) {
            frame_dir = argv[++i];
        } else if (arg == "--viewer" && i + 1 < argc) {
            viewer_type = argv[++i];
        } else if (arg == "--map-db-out" && i + 1 < argc) {
            map_db_out = argv[++i];
        }
    }

    if (vocab_path.empty() || config_path.empty() || frame_dir.empty()) {
        std::cerr << "Usage: run_slam -v <vocab> -c <config> -d <frame_dir> "
                  << "[--viewer socket_publisher] [--map-db-out path]"
                  << std::endl;
        return 1;
    }

    // Load config
    auto cfg = std::make_shared<stella_vslam::config>(config_path);

    // Create SLAM system
    auto slam = std::make_shared<stella_vslam::system>(cfg, vocab_path);
    slam->startup();

    std::cerr << "stella_vslam started. Watching " << frame_dir << std::endl;

    // Track which frames we've already processed
    std::set<std::string> processed;
    unsigned int frame_id = 0;
    const auto start_time = std::chrono::steady_clock::now();

    while (true) {
        // Scan for new frames
        bool found_new = false;

        if (fs::exists(frame_dir)) {
            std::vector<fs::path> entries;
            for (const auto& entry : fs::directory_iterator(frame_dir)) {
                if (entry.path().extension() == ".png") {
                    entries.push_back(entry.path());
                }
            }
            std::sort(entries.begin(), entries.end());

            for (const auto& path : entries) {
                std::string name = path.filename().string();
                if (processed.count(name)) continue;

                // Read frame
                cv::Mat img = cv::imread(path.string(), cv::IMREAD_COLOR);
                if (img.empty()) continue;

                // Compute timestamp relative to start
                auto now = std::chrono::steady_clock::now();
                double timestamp =
                    std::chrono::duration<double>(now - start_time).count();

                // Feed to SLAM (monocular for now, RGBD needs depth)
                auto pose = slam->feed_monocular_frame(img, timestamp);

                // Output pose as JSON line
                if (pose) {
                    const auto& m = *pose;
                    std::cout << "{\"frame\":" << frame_id
                              << ",\"pose\":["
                              << m(0, 0) << "," << m(0, 1) << ","
                              << m(0, 2) << "," << m(0, 3) << ","
                              << m(1, 0) << "," << m(1, 1) << ","
                              << m(1, 2) << "," << m(1, 3) << ","
                              << m(2, 0) << "," << m(2, 1) << ","
                              << m(2, 2) << "," << m(2, 3)
                              << "]}" << std::endl;
                }

                processed.insert(name);
                frame_id++;
                found_new = true;

                // Clean up processed frame
                fs::remove(path);
            }
        }

        if (!found_new) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        // Check for shutdown signal (empty file ".done" in frame dir)
        if (fs::exists(frame_dir + "/.done")) {
            break;
        }
    }

    // Save map and shutdown
    slam->save_map_database(map_db_out);
    slam->shutdown();

    std::cerr << "stella_vslam stopped. Processed " << frame_id << " frames."
              << std::endl;
    return 0;
}
