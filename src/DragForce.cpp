#include "DragForce.h"

DragForce::DragForce(vector<Particle*> particles, float drag_k) : drag_k(drag_k)
{
    this->setTarget(particles);
}

void DragForce::setTarget(vector<Particle*> particles)
{
    this->particles = particles;
}

void DragForce::apply(bool springsCanBreak)
{
    if (this->active)
    {
        for (Particle* p : particles) {
            p->m_Force -= p->m_Velocity * drag_k;
        }
    }
    
}

map<int, map<int, float>> DragForce::dx()
{
    return map<int, map<int, float>>();
}

MatrixXf DragForce::dv()
{
    return MatrixXf();
}

void DragForce::draw()
{

}