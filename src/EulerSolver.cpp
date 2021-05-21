#pragma once
#include "EulerSolver.h"
#include "Eigen/Dense"
#include "Eigen/IterativeLinearSolvers"

EulerSolver::EulerSolver(EulerSolver::TYPE type) : type(type) {}

void EulerSolver::simulateStep(System *system, float h) {

    switch (type)
    {
    case EXPLICIT:
        explicitS(system, h);
        break;
    case IMPLICIT:
        implicitS(system, h);
        break;
    case SEMI:
        semiS(system, h);
    }

}

void EulerSolver::explicitS(System *system, float h) {

    // Get the old state
    VectorXf oldState = system->particleGetState();

    // Evaluate derivative
    VectorXf stateDeriv = system->particleAcceleration();
    // Calculate the new state
    VectorXf newState = oldState + h * stateDeriv;

    if (system->wall)
        newState = system->collisionValidation(newState);
    //set the new state
    system->particleSetState(newState, system->particleGetTime() + h);
}

void EulerSolver::semiS(System *system, float h) {
    // Get the old state
    VectorXf oldState = system->particleGetState();

    // Evaluate derivative
    VectorXf stateDeriv = system->particleAcceleration();

    // Calculate the new state
    VectorXf newState = oldState + h * stateDeriv;
    //set the new state
    system->particleSetState(newState, system->particleGetTime() + h);

    //Use new vectory
    // VectorXf semiState(system->particleDims());
    for (int i = 0; i < newState.size(); i += 4) {
        newState[i + 0] = oldState[i + 0] + h * newState[i + 2];
        newState[i + 1] = oldState[i + 1] + h * newState[i + 3];
    }

    if (system->wall)
        newState = system->collisionValidation(newState);
    //set the new state
    system->particleSetState(newState, system->particleGetTime() + h);            
}


void EulerSolver::implicitS(System *system, float h) {
    // Get the old state
    VectorXf oldState = system->particleGetState();
    int particleDim = 2;
    size_t particleSysSize = system->particles.size() * particleDim;
    // Initiate mass matrix for the system
    MatrixXf M = MatrixXf::Zero(particleSysSize, particleSysSize);
    MatrixXf fdx = MatrixXf::Zero(particleSysSize, particleSysSize);
    MatrixXf fdv = MatrixXf::Zero(particleSysSize, particleSysSize);

    VectorXf f0 = VectorXf::Zero(particleSysSize);
    VectorXf v0 = VectorXf::Zero(particleSysSize);
    for (int i = 0; i < particleSysSize; i += particleDim)
    {
        Particle *p = system->particles[i / 2];
        for (int j = 0; j < particleDim; j++)
            M(i+j, i+j) = p->mass;
    }
}
    
