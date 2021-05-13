#include "System.h"

System::System() : time(0.0f), dt(0.005)
{
}

void System::addParticle(Particle* p)
{
    particles.push_back(p);
}

void System::addForce(Force* f)
{
    forces.push_back(f);
}

int System::particleDims()
{
    return particles.size() * 2 * 2;
}

vector<float> System::particleGetState()
{
    vector<float> s(this->particleDims());

    for (int i = 0; i < this->particles.size(); i++)
    {
        Particle* p = particles[i];
        s[i * 4 + 0] = p->m_Position[0];
        s[i * 4 + 1] = p->m_Position[1];
        s[i * 4 + 2] = p->m_Velocity[0];
        s[i * 4 + 3] = p->m_Velocity[1];
    }

    return s;
}

float System::particleGetTime()
{
    return time;
}

void System::particleSetState(vector<float> src)
{
    this->particleSetState(src, this->particleGetTime());
}

void System::particleSetState(vector<float> newState, float time)
{
    for (int i = 0; i < particles.size(); i++)
    {
        particles[i]->m_Position[0] = newState[i * 4 + 0];
        particles[i]->m_Position[1] = newState[i * 4 + 1];
        particles[i]->m_Velocity[0] = newState[i * 4 + 2];
        particles[i]->m_Velocity[0] = newState[i * 4 + 3];
    }
    this->time = time;
}

vector<float> System::particleDerivative()
{
    vector<float> dst(this->particleDims());
    for (int i = 0; i < particles.size(); i++)
    {
        Particle *p = particles[i];
        dst[i * 4 + 0] = p->m_Velocity[0];
        dst[i * 4 + 1] = p->m_Velocity[1];
        dst[i * 4 + 2] = p->m_Force[0] / p->mass;
        dst[i * 4 + 3] = p->m_Force[1] / p->mass;
    }
    return dst;
}

void System::clearForces() {
    for (Particle *p : particles) {
        p->m_Force = Vec2f(0.0f, 0.0f);
    }
}

void System::applyForces() {
    for (Force *f : forces) {
        f->apply(springsCanBreak);
    }
}

