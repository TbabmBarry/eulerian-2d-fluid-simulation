#ifndef SPECIFIC_SYSTEM_H
#define SPECIFIC_SYSTEM_H

#include "Particle.h"
#include "Force.h"
#include "Constraint.h"
#include "Solver.h"
#include <vector>
#include "Eigen/Dense"

using namespace Eigen;

class Solver;
class System {

    public:
        System(Solver* solver);

        // Store all the particles in the system
        vector<Particle*> particles;
        // Store all the forces to be applied on particles
        vector<Force*> forces;
        // Store all the constraints to be maintained
        vector<Constraint*> constraints;


        bool springsCanBreak = false;
        bool wall;
        Solver* solver;
        // Unit step time
        float dt;

        void addParticle(Particle* p);
        void addForce(Force* f);
        void addConstraint(Constraint* constraint);
        // ODE Interface

        VectorXf particleAcceleration();
        // Calculate the derivative: Divide force by mass to get acceleration, and gather the derivatives into a global vector for the solver
        VectorXf particleDerivative();
        // Gather state from the particles into dst
        VectorXf particleGetState();

        // Check the wall collision and update state
        VectorXf collisionValidation(VectorXf newState);

        float particleGetTime();
        // Scatter state from src into the particles
        void particleSetState(VectorXf src);
        void particleSetState(VectorXf newState, float time);
        // Get length of state derivative, and force vectors
        int particleDims();

        void free();
        void reset();
        void simulationStep();
        void drawParticles();
        void drawForces();
        void drawConstraints();
        void drawSystem();
    private:
        // Clear forces: zero each particle's force accumulator
        void clearForces();
        // Loop over all force objects, allowing each to add forces to the particles it influences
        void applyForces();
        float time;

};

#endif //SPECIFIC_SYSTEM_H