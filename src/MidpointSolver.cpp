#include "MidpointSolver.h"

void MidpointSolver::simulateStep(System *system, float h) {

    // Get the initial state
    Vector2f oldState = system->particleGetState();

    // Evaluate a deriv step
    Vector2f stateDeriv = system->particleAcceleration();

    // Compute the halfway point
    Vector2f midPointState = oldState + h * 0.5f * stateDeriv;

    // Set the state to this midpoint
    system->particleSetState(midPointState, system->particleGetTime() + h);

    // Evaluate derivative at the midpoint
    stateDeriv = system->particleAcceleration();

    // Update the state based on the computation from this midpoint
    VectorXf newState = oldState + h * stateDeriv;

    if (system->wall)
        newState = system->collisionValidation(newState);
    system->particleSetState(newState, system->particleGetTime() + h);
}