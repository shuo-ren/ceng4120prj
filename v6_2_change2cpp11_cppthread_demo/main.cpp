// main.cpp

#include "src/dataloader.h"
#include "src/router.h"
// Remove std::filesystem
#include <iostream>
#include <string>
#include <chrono>
#include <sys/stat.h>

// Helper function to create directories (replacing std::filesystem functionality)
void create_directories(const std::string& path) {
    size_t pos = 0;
    std::string dir;
    
    while ((pos = path.find('/', pos)) != std::string::npos) {
        dir = path.substr(0, pos++);
        if (dir.length() == 0) continue; // Skip if it's the root directory
        
        #ifdef _WIN32
        mkdir(dir.c_str());
        #else
        mkdir(dir.c_str(), 0755);
        #endif
    }
}

// Helper function to get parent path
std::string parent_path(const std::string& path) {
    size_t pos = path.find_last_of('/');
    if (pos == std::string::npos) return "";
    return path.substr(0, pos);
}

int main(int argc, char* argv[]) {
    if (argc < 4) {
        std::cerr << "Usage: fpga_router <device> <netlist> <output.route>"
                  << " [--threads <num_threads>] [--debug]\n";
        return 1;
    }
    std::string devFile   = argv[1];
    std::string netFile   = argv[2];
    std::string outRoute  = argv[3];
    int threads = 8;  // 默认线程数
    bool debug = false;

    for (int i = 4; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--threads" && i + 1 < argc) {
            threads = std::stoi(argv[++i]);
        } else if (arg == "--debug") {
            debug = true;
        }
    }

    // 限制线程数在有效范围内
    threads = std::max(1, std::min(threads, 8));

    try {
        // 确保输出目录存在
        std::string parentPath = parent_path(outRoute);
        if (!parentPath.empty()) {
            create_directories(parentPath);
        }
        
        auto startTime = std::chrono::high_resolution_clock::now();
        
        // 记录加载开始时间
        auto loadStart = std::chrono::high_resolution_clock::now();
        
        DataLoader dl(threads);
        dl.loadDevice(devFile);
        dl.loadNetlist(netFile);
        
        // 记录加载结束时间
        auto loadEnd = std::chrono::high_resolution_clock::now();
        auto loadDuration = std::chrono::duration_cast<std::chrono::seconds>(loadEnd - loadStart).count();
        
        if (debug) {
            std::cout << "Data loading completed in " << loadDuration << " seconds.\n";
        }

        // 记录路由开始时间
        auto routeStart = std::chrono::high_resolution_clock::now();
        
        Router rt(dl.getNodes(), dl.getAdjacency(), dl.getNetlist(), outRoute, debug, threads);
        rt.route();
        
        // 记录路由结束时间
        auto routeEnd = std::chrono::high_resolution_clock::now();
        auto routeDuration = std::chrono::duration_cast<std::chrono::seconds>(routeEnd - routeStart).count();
        
        if (debug) {
            std::cout << "Routing completed in " << routeDuration << " seconds.\n";
        }

        // 记录总处理时间
        auto endTime = std::chrono::high_resolution_clock::now();
        auto totalDuration = std::chrono::duration_cast<std::chrono::seconds>(endTime - startTime).count();
        
        std::cout << "Total processing time: " << totalDuration << " seconds.\n";
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << '\n';
        return 1;
    }
    return 0;
}