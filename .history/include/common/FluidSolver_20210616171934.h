#pragma once
#include "Solver.h"

class FluidSolver : public Solver {
public:
    void simulateStep(System* system, float h) override;
    void add_source(int N, float* x, float* s, float dt);
    FluidSolver();
private:
    float dt;
    float density;
    float density_previous;
    float u;
    float u_previous;
    float v;
    float v_previous;
    float vorticity;
    float width;
    float height;
    int N;
    int size=(N+2)*(N+2);

};