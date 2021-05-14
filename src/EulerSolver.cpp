#pragma once
#include "EulerSolver.h"

EulerSolver::EulerSolver(EulerSolver::TYPE type) : type(type) {}

void EulerSolver::simulateStep(System *system, float h) {
    if (type == IMPLICIT) {
        // Something to add
    } else {
        // Get the old state
        VectorXf oldState = system->particleGetState();

        // Evaluate derivative
        VectorXf stateDeriv = system->particleAcceleration();

        // Calculate the new state
        // VectorXf newState = oldState + h*stateDeriv;
        VectorXf newState = oldState + h * stateDeriv;
        

        if (type == SEMI) {
            //Something to add
        }
    }
}