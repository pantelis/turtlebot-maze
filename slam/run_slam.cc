/**
 * Minimal stella_vslam driver for stdin-based frame input.
 *
 * Reads frames from stdin as length-prefixed raw BGR bytes (no file I/O),
 * feeds them to stella_vslam monocular pipeline, and writes pose estimates
 * to stdout as JSON lines.
 *
 * Frame wire format (all uint32, little-endian):
 *   [4 bytes width][4 bytes height][4 bytes channels]
 *   [width * height * channels bytes of BGR pixel data]
 *
 * Usage:
 *   run_slam -v vocab.fbow -c config.yaml [--map-db-out path]
 */

#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>

#include <opencv2/core.hpp>

#include <stella_vslam/config.h>
#include <stella_vslam/system.h>

int main(int argc, char* argv[]) {
    std::string vocab_path, config_path;
    std::string map_db_out = "/tmp/slam_map.msg";

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if ((arg == "-v" || arg == "--vocab") && i + 1 < argc) {
            vocab_path = argv[++i];
        } else if ((arg == "-c" || arg == "--config") && i + 1 < argc) {
            config_path = argv[++i];
        } else if (arg == "--map-db-out" && i + 1 < argc) {
            map_db_out = argv[++i];
        }
    }

    if (vocab_path.empty() || config_path.empty()) {
        std::cerr << "Usage: run_slam -v <vocab> -c <config> [--map-db-out path]"
                  << std::endl;
        return 1;
    }

    // Load config and create SLAM system
    auto cfg = std::make_shared<stella_vslam::config>(config_path);
    auto slam = std::make_shared<stella_vslam::system>(cfg, vocab_path);
    slam->startup();

    std::cerr << "stella_vslam started. Reading frames from stdin." << std::endl;

    unsigned int frame_id = 0;
    const auto start_time = std::chrono::steady_clock::now();

    while (std::cin.good()) {
        // Read frame header: [width uint32][height uint32][channels uint32]
        uint32_t width = 0, height = 0, channels = 0;

        if (!std::cin.read(reinterpret_cast<char*>(&width), 4)) break;
        if (!std::cin.read(reinterpret_cast<char*>(&height), 4)) break;
        if (!std::cin.read(reinterpret_cast<char*>(&channels), 4)) break;

        if (width == 0 || height == 0 || channels == 0) break;

        // Read raw pixel data directly into cv::Mat (no PNG decode, no file I/O)
        const auto nbytes =
            static_cast<std::streamsize>(width) * height * channels;
        cv::Mat img(static_cast<int>(height), static_cast<int>(width),
                    CV_8UC(static_cast<int>(channels)));

        if (!std::cin.read(reinterpret_cast<char*>(img.data), nbytes)) break;

        // Timestamp relative to startup
        auto now = std::chrono::steady_clock::now();
        double timestamp = std::chrono::duration<double>(now - start_time).count();

        // Feed to SLAM (monocular)
        auto pose = slam->feed_monocular_frame(img, timestamp);

        // Output pose as JSON line on stdout
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

        frame_id++;
    }

    slam->save_map_database(map_db_out);
    slam->shutdown();

    std::cerr << "stella_vslam stopped. Processed " << frame_id << " frames."
              << std::endl;
    return 0;
}
