// quality.h
#pragma once
#include "strategy/strategy.h"

class QualityStrategy : public Strategy {
public:
    void run(RouteContext &ctx, double budget) override;
};
