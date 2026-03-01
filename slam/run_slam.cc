/**
 * Minimal stella_vslam driver for stdin-based frame input.
 *
 * Reads frames from stdin as length-prefixed raw bytes (no file I/O),
 * feeds them to stella_vslam, and writes pose estimates to stdout as
 * JSON lines.
 *
 * Monocular wire format (colour only):
 *   [4 bytes width][4 bytes height][4 bytes channels]
 *   [width * height * channels bytes of BGR pixel data, uint8]
 *
 * RGBD wire format (colour frame followed immediately by depth frame):
 *   Colour: [4 bytes width][4 bytes height][4 bytes channels=3]
 *           [width * height * 3 bytes of BGR pixel data, uint8]
 *   Depth:  [4 bytes width][4 bytes height][4 bytes channels=1]
 *           [width * height * 4 bytes of depth data, float32 metres]
 *
 * slam_bridge.py normalises depth to float32 metres before sending,
 * so depthmap_factor in the YAML config should be 1.0.
 *
 * Usage:
 *   run_slam -v vocab.fbow -c config.yaml [--mode monocular|rgbd] [--map-db-out path]
 */

#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>

#include <opencv2/core.hpp>

#include <stella_vslam/config.h>
#include <stella_vslam/system.h>

static bool read_u8_frame(cv::Mat& out) {
    uint32_t w = 0, h = 0, c = 0;
    if (!std::cin.read(reinterpret_cast<char*>(&w), 4)) return false;
    if (!std::cin.read(reinterpret_cast<char*>(&h), 4)) return false;
    if (!std::cin.read(reinterpret_cast<char*>(&c), 4)) return false;
    if (!w || !h || !c) return false;
    out = cv::Mat(static_cast<int>(h), static_cast<int>(w),
                  CV_8UC(static_cast<int>(c)));
    const auto nbytes = static_cast<std::streamsize>(w) * h * c;
    return bool(std::cin.read(reinterpret_cast<char*>(out.data), nbytes));
}

static bool read_f32_frame(cv::Mat& out) {
    uint32_t w = 0, h = 0, c = 0;
    if (!std::cin.read(reinterpret_cast<char*>(&w), 4)) return false;
    if (!std::cin.read(reinterpret_cast<char*>(&h), 4)) return false;
    if (!std::cin.read(reinterpret_cast<char*>(&c), 4)) return false;
    if (!w || !h) return false;
    out = cv::Mat(static_cast<int>(h), static_cast<int>(w), CV_32FC1);
    const auto nbytes = static_cast<std::streamsize>(w) * h * 4;
    return bool(std::cin.read(reinterpret_cast<char*>(out.data), nbytes));
}

int main(int argc, char* argv[]) {
    std::string vocab_path, config_path;
    std::string map_db_out = "/tmp/slam_map.msg";
    std::string mode = "monocular";

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if ((arg == "-v" || arg == "--vocab") && i + 1 < argc) {
            vocab_path = argv[++i];
        } else if ((arg == "-c" || arg == "--config") && i + 1 < argc) {
            config_path = argv[++i];
        } else if (arg == "--map-db-out" && i + 1 < argc) {
            map_db_out = argv[++i];
        } else if (arg == "--mode" && i + 1 < argc) {
            mode = argv[++i];
        }
    }

    if (vocab_path.empty() || config_path.empty()) {
        std::cerr << "Usage: run_slam -v <vocab> -c <config>"
                     " [--mode monocular|rgbd] [--map-db-out path]"
                  << std::endl;
        return 1;
    }

    if (mode != "monocular" && mode != "rgbd") {
        std::cerr << "Unknown mode: " << mode
                  << " (expected monocular or rgbd)" << std::endl;
        return 1;
    }

    // Load config and create SLAM system
    auto cfg  = std::make_shared<stella_vslam::config>(config_path);
    auto slam = std::make_shared<stella_vslam::system>(cfg, vocab_path);
    slam->startup();

    std::cerr << "stella_vslam started. mode=" << mode
              << ". Reading frames from stdin." << std::endl;

    unsigned int frame_id = 0;
    const auto start_time = std::chrono::steady_clock::now();

    while (std::cin.good()) {
        cv::Mat color;
        if (!read_u8_frame(color)) break;

        const auto now = std::chrono::steady_clock::now();
        const double timestamp =
            std::chrono::duration<double>(now - start_time).count();

        std::shared_ptr<stella_vslam::Mat44_t> pose;

        if (mode == "rgbd") {
            cv::Mat depth;
            if (!read_f32_frame(depth)) break;
            pose = slam->feed_RGBD_frame(color, depth, timestamp);
        } else {
            pose = slam->feed_monocular_frame(color, timestamp);
        }

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
