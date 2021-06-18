#include "ExternalForce.h"

ExternalForce::ExternalForce(std::vector<Particle*> particles, float force, Vector2f direction) : direction(direction)
{
    this->setTarget(particles);
    this->setForce(force);
}

void ExternalForce::setTarget(std::vector<Particle*> particles)
{
    this->particles = particles;
}

void ExternalForce::setForce(float f)
{
    this->force = f;
}


void ExternalForce::setActive(bool state){
    active = state;
}

void ExternalForce::apply(bool springCanBreak)
{
    if (!active)
        return;
    for (Particle* p : particles) {
        p->m_Force += force * direction;
    }
}

map<int, map<int, float>> ExternalForce::dx()
{
    return map<int, map<int, float>>();
}

MatrixXf ExternalForce::dv()
{
    return MatrixXf();
}

void ExternalForce::draw()
{

}