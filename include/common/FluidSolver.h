#pragma once
#include "Solver.h"
#include "Particle.h"

class FluidSolver : public Solver {
public:
    void simulateStep(System* system, float h) override;
    void add_source(int N, float* x, float* s, float dt);
    void set_bnd ( int N, int b, float * x );
    void lin_solve ( int N, int b, float * x, float * x0, float a, float c );
    void diffuse ( int N, int b, float * x, float * x0, float diff, float dt );
    void advect ( int N, int b, float * d, float * d0, float * u, float * v, float dt );
    void project ( int N, float * u, float * v, float * p, float * div );
    void vorticity_confinement(int N, float dt, float * d0, float* u, float* v, float * u0, float * v0);
    vector<Particle*> rigidbodies;
    int dsim=0;
    void setDensity(float* d, float* dprev, int inputN);
    float getDensity(int i, int j);
    void setVelocity(float* xu, float* xv, float* xuprev, float* xvprev, int inputN);
    float getXVelocity(int i, int j);
    float getYVelocity(int i, int j);
    void dens_step ( int N, float * x, float * x0, float * u, float * v, float diff, float dt );
    void vel_step ( int N, float * u, float * v, float * u0, float * v0, float visc, float dt );
    FluidSolver(float* u,float* v);
    FluidSolver();
private:
    float h;
    float* density;
    float* density_previous;
    float* u;
    float* u_previous;
    float* v;
    float* v_previous;
    float vorticity;
    float width;
    float height;
    int N;
    int size=(N+2)*(N+2);
};