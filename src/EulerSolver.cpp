#pragma once
#include "EulerSolver.h"

EulerSolver::EulerSolver(EulerSolver::TYPE type) : type(type) {}

void EulerSolver::simulateStep(System *system, float h) {
    if (type == IMPLICIT) {
        // Something to add
    } else {
        // Get the old state
        vector<float> oldState = system->particleGetState();

        // Evaluate derivative
        vector<float> stateDeriv = system->particleDerivative();

        // Calculate the new state
        // vector<float> newState = oldState + h*stateDeriv;
        vector<float> newState;
        for (auto i = 0; i< stateDeriv.size(); i++) {
            newState.push_back( oldState[i] + h*stateDeriv[i] );
        }
        

        if (type == SEMI) {
            //Something to add
        }
    }
}