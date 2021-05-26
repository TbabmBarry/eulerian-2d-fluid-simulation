#pragma once
#include "Solver.h"

class RungeSovler : public Solver {
public:
    void simulateStep(System* system, float h) override;
    RungeSovler();
};