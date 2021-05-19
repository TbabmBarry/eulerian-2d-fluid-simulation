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

    if (system->wall)
        newState = system->collisionValidation(newState);
    //set the new state
    system->particleSetState(newState, system->particleGetTime() + h);      
<<<<<<< HEAD

=======
    cout << "Force: " << system->particles[0]->m_Force << " Velocity: " << system->particles[0]->m_Velocity << endl;
>>>>>>> 8baa2ab0ba6e61065ffb79d8f891d1230b5d073c
    // for (int i = 0; i < system->particles.size(); i++)
    // {
    //     std::cout << "after force" << system->particles[i]->m_Force << std::endl;
    //     std::cout << "after velocity" << system->particles[i]->m_Velocity << std::endl;
    // }        
<<<<<<< HEAD
    std::cout << "position1:" << system->particles[0]->m_Position << std::endl;
=======
<<<<<<< HEAD

    // std::cout << "position1:" << system->particles[0]->m_Position <<"     position2:" << system->particles[1]->m_Position << "     position3:" << system->particles[2]->m_Position << "     position4:" << system->particles[3]->m_Position  << std::endl;
=======
>>>>>>> 8baa2ab0ba6e61065ffb79d8f891d1230b5d073c
    // std::cout << "position1:" << system->particles[0]->m_Position <<"     position2:" << system->particles[1]->m_Position << std::endl;
>>>>>>> d93438ed873c7aabd5bcc576f04622219f624093
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
    
