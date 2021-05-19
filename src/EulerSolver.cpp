#pragma once
#include "EulerSolver.h"

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
    // std::cout << "old state" << oldState << std::endl;
    // Evaluate derivative
    VectorXf stateDeriv = system->particleAcceleration();
    
    // Calculate the new state
    VectorXf newState = oldState + h * stateDeriv;
    // std::cout << "new state" <<newState << std::endl;

    //set the new state
    system->particleSetState(newState, system->particleGetTime() + h);      

    for (int i = 0; i < system->particles.size(); i++)
    {
        std::cout << "after force" << system->particles[i]->m_Force << std::endl;
        std::cout << "after velocity" << system->particles[i]->m_Velocity << std::endl;
    }        
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

    //set the new state
    system->particleSetState(newState, system->particleGetTime() + h);            
}


void EulerSolver::implicitS(System *system, float h) {

}
    
