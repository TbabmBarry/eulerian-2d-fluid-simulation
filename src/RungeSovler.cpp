#include "RungeSovler.h"

RungeSovler::RungeSovler(){}

void RungeSovler::simulateStep(System *system, float h) {

    // Get the old state
    VectorXf oldState = system->particleGetState();
    float oldTime = system->particleGetTime();

    //Evaluate derivative
    VectorXf stateDeriv = system->particleAcceleration();

    //Calculate k1
    VectorXf k1 = h * stateDeriv;

    //Get midpoint and Calculate k2 
    VectorXf newState = oldState + k1 / 2;
    system->particleSetState(newState, oldTime + h / 2);
    stateDeriv = system->particleAcceleration();
    VectorXf k2 = h * stateDeriv;

    //Get midpoint and Calculate k3 
    newState = oldState + k2 / 2;
    system->particleSetState(newState, oldTime + h / 2);
    stateDeriv = system->particleAcceleration();
    VectorXf k3 = h * stateDeriv;


    //Get midpoint and Calculate k4 
    newState = oldState + k3;
    system->particleSetState(newState, oldTime + h);
    stateDeriv = system->particleAcceleration();
    VectorXf k4 = h * stateDeriv;

    //Final state generation
    newState = oldState + 1.0f / 6.0f * k1 + 1.0f / 3.0f * k2 + 1.0f / 3.0f * k3 + 1.0f / 6.0f * k4;

    if (system->wall)
        newState = system->collisionValidation(newState);
    //Set final state
    system->particleSetState(newState, oldTime + h);
    

}