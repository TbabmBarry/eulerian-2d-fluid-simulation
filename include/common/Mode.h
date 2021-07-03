#include "System.h"
#include "Fluid.h"

class Mode
{
public:
    void RigidBodyCollision(System *system, Fluid *fluid);
    void RigidBody(System *system, Fluid *fluid);
    void Fix(System *system, Fluid *fluid);
    void Move(System *system, Fluid *fluid);
    void FluidCloth(System *system);
};