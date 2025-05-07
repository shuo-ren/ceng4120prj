// ------------------------------------------------------------
// File: src/main.cpp
// ------------------------------------------------------------
#include "router.h"
#include "dataloader.h"
#include <filesystem>
#include <iostream>
#include <chrono>

int main(int argc, char* argv[]) {
    if (argc < 4) {
        std::cerr << "Usage: fpga_router <device> <netlist> <output.route>"
                     " [--threads N] [--debug]\n";
        return 1;
    }
    std::string devFile  = argv[1];
    std::string netFile  = argv[2];
    std::string outFile  = argv[3];
    int  threads = 8;
    bool debug   = false;

    for (int i = 4; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--threads" && i + 1 < argc) {
            threads = std::stoi(argv[++i]);
        } else if (arg == "--debug") {
            debug = true;
        }
    }
    threads = std::clamp(threads, 1, 8);

    try {
        std::filesystem::path outPath(outFile);
        std::filesystem::create_directories(outPath.parent_path());

        DataLoader dl(threads);
        auto t0 = std::chrono::high_resolution_clock::now();
        dl.loadDevice(devFile);
        dl.loadNetlist(netFile);
        auto t1 = std::chrono::high_resolution_clock::now();
        double t_read = std::chrono::duration<double>(t1 - t0).count();

        double limit = (netFile.find("design5") != std::string::npos) ? 250.0 : 100.0;

        Router rt(dl.getNodes(), dl.getAdjacency(), dl.getNetlist(),
                  outFile, debug, threads, t_read, limit);
        rt.route();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
    return 0;
}