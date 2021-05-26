#pragma once
#include "Solver.h"

class MidpointSolver : public Solver {
public:
    void simulateStep(System* system, float h) override;
    MidpointSolver();
};
