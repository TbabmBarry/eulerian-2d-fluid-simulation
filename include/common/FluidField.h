#pragma once
#include "Particle.h"
#include "System.h"


class System;
class FluidField {
public:
    FluidField(System* sys, int grid_N);

    System* sys;
    float dt;
    float* density;
    float* density_previous;
    float* u;
    float* u_previous;
    float* v;
    float* v_previous;
    float vorticity;
    float diff;
    float visc;
    float width;
    float height;
    int N;
    int size=(N+2)*(N+2);

    void simulationStep();
    void addSource(float* x, float* s);
    void setBnd (int b, float* x);
    void linSolve (int b, float* x, float* x0, float a, float c);
    void diffuse (int b, float * x, float * x0, float diff);
    void advect (int b, float * d, float * d0, float * u, float * v);
    void project (float * u, float * v, float * p, float * div);
    void vorticityConfinement();
    void densStep ();
    void velStep ();
    void drawDensity();
    void drawVelocity();
    bool allocate();
    void freeData();
    void reset();
private:
    
};