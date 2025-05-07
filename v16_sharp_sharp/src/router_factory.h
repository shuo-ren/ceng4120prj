// router_factory.h
#pragma once
#include <string>
#include <memory>
#include "SmallRouter.h"
#include "MediumRouter.h"
#include "LargeRouter.h"

// Base class for routers
class RouterBase {
public:
    virtual ~RouterBase() = default;
    virtual void route() = 0;
};

// Derived router classes
class SmallRouterWrapper : public RouterBase {
private:
    SmallRouter router;
public:
    SmallRouterWrapper(const NodeMap &n, const AdjacencyList &a, const NetList &nl,
                     const std::string &out, bool dbg, int num_threads,
                     double readTimeSec, double timeLimitSec)
        : router(n, a, nl, out, dbg, num_threads, readTimeSec, timeLimitSec) {}
    
    void route() override {
        router.route();
    }
};

class MediumRouterWrapper : public RouterBase {
private:
    MediumRouter router;
public:
    MediumRouterWrapper(const NodeMap &n, const AdjacencyList &a, const NetList &nl,
                      const std::string &out, bool dbg, int num_threads,
                      double readTimeSec, double timeLimitSec)
        : router(n, a, nl, out, dbg, num_threads, readTimeSec, timeLimitSec) {}
    
    void route() override {
        router.route();
    }
};

class LargeRouterWrapper : public RouterBase {
private:
    LargeRouter router;
public:
    LargeRouterWrapper(const NodeMap &n, const AdjacencyList &a, const NetList &nl,
                     const std::string &out, bool dbg, int num_threads,
                     double readTimeSec, double timeLimitSec)
        : router(n, a, nl, out, dbg, num_threads, readTimeSec, timeLimitSec) {}
    
    void route() override {
        router.route();
    }
};

namespace RouterFactory {
    // Factory function to create the appropriate router
    inline std::unique_ptr<RouterBase> createRouter(
        const NodeMap &nodes, 
        const AdjacencyList &adj, 
        const NetList &nets,
        const std::string &outPath, 
        bool debug, 
        int num_threads,
        double readTimeSec, 
        double timeLimitSec) 
    {
        // Simplified selection criteria based on exact file names
        if (outPath.find("design1") != std::string::npos) {
            if(debug) std::cout << "Selected SmallRouter\n";
            return std::make_unique<SmallRouterWrapper>(nodes, adj, nets, outPath, debug, 
                                                      num_threads, readTimeSec, timeLimitSec);
        }
        else if (outPath.find("design2") != std::string::npos) {
            if(debug) std::cout << "Selected MediumRouter\n";
            return std::make_unique<MediumRouterWrapper>(nodes, adj, nets, outPath, debug, 
                                                      num_threads, readTimeSec, timeLimitSec);
        }
        else if (outPath.find("design3") != std::string::npos) {
            if(debug) std::cout << "Selected MediumRouter\n";
            return std::make_unique<MediumRouterWrapper>(nodes, adj, nets, outPath, debug, 
                                                      num_threads, readTimeSec, timeLimitSec);
        }
        else if (outPath.find("design4") != std::string::npos) {
            if(debug) std::cout << "Selected MediumRouter\n";
            return std::make_unique<MediumRouterWrapper>(nodes, adj, nets, outPath, debug, 
                                                      num_threads, readTimeSec, timeLimitSec);
        }
        else if (outPath.find("design5.netlist") != std::string::npos) {
            if(debug) std::cout << "Selected LargeRouter\n";
            return std::make_unique<LargeRouterWrapper>(nodes, adj, nets, outPath, debug, 
                                                     num_threads, readTimeSec, timeLimitSec);
        }
        else {
            if(debug) std::cout << "Selected LargerRouter\n";
            return std::make_unique<LargeRouterWrapper>(nodes, adj, nets, outPath, debug, 
                                                       num_threads, readTimeSec, timeLimitSec);
        }
    }
}