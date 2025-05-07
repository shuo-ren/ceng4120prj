// main.cpp

#include "src/dataloader.h"
#include "src/router.h"
#include "src/routecache.h"
#include <filesystem>
#include <iostream>

int main(int argc, char* argv[]) {
    if (argc < 4) {
        std::cerr << "Usage: fpga_router <device> <netlist> <output.route>"
                  << " [--cache <cache_file>]\n";
        return 1;
    }
    std::string devFile   = argv[1];
    std::string netFile   = argv[2];
    std::string outRoute  = argv[3];
    std::string cacheFile;
    bool useCache = false;

    for (int i = 4; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--cache" && i + 1 < argc) {
            cacheFile = argv[++i];
            useCache  = true;
        }
    }

    try {
        DataLoader dl(8);
        dl.loadDevice(devFile);
        dl.loadNetlist(netFile);

        RouteCache rc;
        if (useCache && std::filesystem::exists(cacheFile))
            rc.load(cacheFile);

        Router rt(dl.getNodes(), dl.getAdjacency(), dl.getNetlist(), outRoute);
        if (useCache) rt.setCache(&rc);

        rt.route();

        if (useCache) rc.save(cacheFile);
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << '\n';
        return 1;
    }
    return 0;
}


// ./fpga_router xcvu3p.device benchmarks/design2.netlist output/design2.route --cache cache/design2.cache