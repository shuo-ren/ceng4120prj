#pragma once
#include "dataloader.h"
#include "common/a_star.h"
#include <vector>

namespace looprem {
class LoopRemover {
public:
    explicit LoopRemover(const NodeMap&);
    void removeAll(std::vector<astar::RouteResult>& results);
private:
    const NodeMap& nodes;
    static std::vector<std::pair<int,int>>
    removeLoopsInternal(const std::vector<std::pair<int,int>>& edges,
                        int netId, const std::vector<int>& sinks);
};
} // namespace looprem
