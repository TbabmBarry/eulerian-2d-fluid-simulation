#include "ExternalForce.h"

ExternalForce::ExternalForce(std::vector<Particle*> particles, Vec2f direction) : direction(direction)
{
    this->setTarget(particles);
}

void ExternalForce::setTarget(std::vector<Particle*> particles)
{
    this->particles = particles;
}

void ExternalForce::setActive(bool state){
    active = state;
}

void ExternalForce::apply(bool springCanBreak)
{
    if (!active)
        return;
    for (Particle* p : particles) {
        p->m_Force += 0.1f * direction;
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