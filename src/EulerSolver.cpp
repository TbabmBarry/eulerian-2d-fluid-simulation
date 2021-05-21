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

    // Evaluate derivative
    VectorXf stateDeriv = system->particleAcceleration();
    cout << "oldState: " << oldState << endl;
    cout << "stateDeriv: " << stateDeriv << endl;
    // Calculate the new state
    VectorXf newState = oldState + h * stateDeriv;
    cout << "newState: " << newState << endl;

    if (system->wall)
        newState = system->collisionValidation(newState);
    //set the new state
    system->particleSetState(newState, system->particleGetTime() + h);
    cout << "position " << system->particles[0]->m_Position <<endl;
    // cout << "mass: " << system->particles[0]->mass<< endl;
    // cout << "Force: " << system->particles[0]->m_Force << " Velocity: " << system->particles[0]->m_Velocity << endl;
    // for (int i = 0; i < system->particles.size(); i++)
    // {
    //     std::cout << "after force" << system->particles[i]->m_Force << std::endl;
    //     std::cout << "after velocity" << system->particles[i]->m_Velocity << std::endl;
    // }        
    // std::cout << "position1:" << system->particles[0]->m_Position <<"     position2:" << system->particles[1]->m_Position << std::endl;
    // std::cout << "" << std::endl;
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

}
    
