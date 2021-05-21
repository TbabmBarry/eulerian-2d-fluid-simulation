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

MatrixXf DragForce::dx()
{
    return MatrixXf();
}

MatrixXf DragForce::dv()
{
    return drag_k * MatrixXf::Identity(2, 2);
}

void DragForce::draw()
{

}