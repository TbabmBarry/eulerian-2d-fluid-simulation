#include "MidpointSolver.h"

void MidpointSolver::simulateStep(System *system, float h) {

    // Get the initial state
    VectorXf oldState = system->particleGetState();

    // Evaluate a deriv step
    VectorXf deriv = system->particleAcceleration();

    // Compute the halfway point
    VectorXf midPointState = oldState + h * 0.5f * deriv;

    // Set the state to this midpoint
    system->particleSetState(midPointState, system->particleGetTime() + h);

    // Evaluate derivative at the midpoint
    deriv = system->particleAcceleration();

    // Update the state based on the computation from this midpoint
    VectorXf newState = oldState + h * deriv;
}