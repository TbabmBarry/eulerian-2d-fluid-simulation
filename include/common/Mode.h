#include "System.h"
#include "FluidSolver.h"

class Mode
{
public:

    void RigidBodyCollision(System *system, FluidSolver *fluid);
    void RigidBody(System *system, FluidSolver *fluid);
    void Fix(System *system, FluidSolver *fluid);
    void Move(System *system, FluidSolver *fluid);
    void FluidCloth(System *system);
    
};