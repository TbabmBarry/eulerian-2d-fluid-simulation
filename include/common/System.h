#include "Particle.h"
#include <vector>
#include <gfx/vec4.h>

using namespace std;

class System {

    public:
        System();

        vector<Particle*> particles;
        float dt;

        void addParticle(Particle* p);

        // ODE Interface
        Vec4f particleDerivative();
        Vec4f particleGetState();
        float particleGetTime();
        void particleSetState(Vec4f src);
        void particleSetState(Vec4f newState, float time);
        int particleDims();
    
    private:
        float time;

};