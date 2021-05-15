#pragma once
#include "Solver.h"

class EulerSolver : public Solver {
public:
    void simulateStep(System* system, float h) override;
    enum TYPE {
        EXPLICIT,
        IMPLICIT,
        SEMI
    };
    TYPE type;

    EulerSolver(TYPE type);
private:
    void explicitS(System* system, float h);
    void implicitS(System* system, float h);
    void semiS(System* system, float h);
};