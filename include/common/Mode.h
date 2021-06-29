#include "System.h"
#include "FluidSolver.h"

class Mode {
    public:
        void CircularGravityRod(System* system);
        void hair(System* system);
        void CircularCloth(System* system);
        void RigidBodyCollision(System* system, FluidSolver *fluid);
};