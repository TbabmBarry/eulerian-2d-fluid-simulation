#ifndef SPECIFIC_CONTRAINT_H
#define SPECIFIC_CONTRAINT_H

#include <vector>
#include "Particle.h"

using namespace std;

class Constraint {

    public:
        Constraint(vector<Particle*> particles) : particles(particles) {}

        virtual void draw() = 0;
        virtual float C() = 0;
        virtual float legal_velocity() = 0;
        virtual float legal_accelerate() = 0;
        virtual vector<Vec2f> Jacobian() = 0;
        virtual vector<Vec2f> jd() = 0;
    private:
        vector<Particle*> particles;
};

#endif //SPECIFIC_CONTRAINT_H