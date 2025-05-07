// main.cpp
#include "src/dataloader.h"
#include "src/router_factory.h"
#include <filesystem>
#include <iostream>
#include <string>
#include <chrono>

int main(int argc, char* argv[]) {
    if (argc < 4) {
        std::cerr << "Usage: fpga_router <device> <netlist> <output.route>"
                     " [--threads <num_threads>] [--debug]\n";
        return 1;
    }
    std::string devFile   = argv[1];
    std::string netFile   = argv[2];
    std::string outRoute  = argv[3];
    int   threads = 8;
    bool  debug   = false;

    for (int i = 4; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--threads" && i + 1 < argc) {
            threads = std::stoi(argv[++i]);
        } else if (arg == "--debug") {
            debug = true;
        }
    }
    threads = std::max(1, std::min(threads, 8));

    try {
        std::filesystem::path outPath(outRoute);
        std::filesystem::create_directories(outPath.parent_path());

        auto startTime = std::chrono::high_resolution_clock::now();

        /* ---------- 读取阶段 ---------- */
        auto loadStart = std::chrono::high_resolution_clock::now();

        DataLoader dl(threads);
        dl.loadDevice(devFile);
        dl.loadNetlist(netFile);

        auto loadEnd = std::chrono::high_resolution_clock::now();
        double t_read = 
            std::chrono::duration<double>(loadEnd - loadStart).count();
        if (debug)
            std::cout << "Data loading completed in "
                      << t_read << " s\n";

        /* ---------- 推断时间限制 ---------- */
        double timeLimit = 100.0;
        if (netFile.find("design5") != std::string::npos) timeLimit = 250.0;

        /* ---------- 选择并运行合适的路由器 ---------- */
        if (debug) {
            std::cout << "Selecting router based on design: " 
                      << netFile << " (size: " << dl.getNetlist().size() << ")\n";
        }
        
        // Create the appropriate router based on design size
        auto router = RouterFactory::createRouter(
            dl.getNodes(), dl.getAdjacency(), dl.getNetlist(),
            outRoute, debug, threads, t_read, timeLimit);
            
        // Run the routing with the selected implementation
        router->route();

        auto endTime = std::chrono::high_resolution_clock::now();
        double totalDuration =
            std::chrono::duration<double>(endTime - startTime).count();
        std::cout << "Total processing time: " << totalDuration << " s\n";
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << '\n';
        return 1;
    }
    return 0;
}