// ------------------------------------------------------------
// File: src/strategy/throughput.h
// ------------------------------------------------------------
#pragma once
#include "strategy/strategy.h"
class ThroughputStrategy : public IStrategy {
public:
    void run(RouteContext &ctx, double budget) override;
};