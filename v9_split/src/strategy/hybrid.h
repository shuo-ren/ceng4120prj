// ------------------------------------------------------------
// File: src/strategy/hybrid.h
// ------------------------------------------------------------
#pragma once
#include "strategy/strategy.h"
class HybridStrategy : public IStrategy {
public:
    void run(RouteContext &ctx, double budget) override;
};