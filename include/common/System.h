#include "Particle.h"
#include "Force.h"
#include <vector>

using namespace std;

class System {

    public:
        System();

        // Store all the particles in the system
        vector<Particle*> particles;
        // Store all the forces to be applied on particles
        vector<Force*> forces;

        bool springsCanBreak = false;

        // Unit step time
        float dt;

        void addParticle(Particle* p);
        void addForce(Force* f);

        // ODE Interface

        // Calculate the derivative: Divide force by mass to get acceleration, and gather the derivatives into a global vector for the solver
        vector<float> particleDerivative();
        // Gather state from the particles into dst
        vector<float> particleGetState();

        float particleGetTime();
        // Scatter state from src into the particles
        void particleSetState(vector<float> src);
        void particleSetState(vector<float> newState, float time);
        // Get length of state derivative, and force vectors
        int particleDims();
    
    private:
        // Clear forces: zero each particle's force accumulator
        void clearForces();
        // Loop over all force objects, allowing each to add forces to the particles it influences
        void applyForces();
        float time;

};