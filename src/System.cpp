#include "System.h"
#include "ConstraintMaintainer.h"


System::System(Solver *solver) : solver(solver), time(0.0f), dt(0.1f)
{
}

void System::addParticle(Particle* p)
{
    particles.push_back(p);
    // std::cout<<p->m_Position<<std::endl;
}

void System::addForce(Force* f)
{
    forces.push_back(f);
}

void System::addConstraint(Constraint* constraint)
{
    constraints.push_back(constraint);
}

int System::particleDims()
{
    return particles.size() * 2 * 2;
}

void System::free() 
{
    particles.clear();
    forces.clear();
}

void System::reset() 
{
    for (Particle *p : particles) 
    {
        p->reset();
    }
}

VectorXf System::particleGetState()
{
    VectorXf s(this->particleDims());

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

void System::particleSetState(VectorXf src)
{
    this->particleSetState(src, this->particleGetTime());
}

void System::particleSetState(VectorXf newState, float time)
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

VectorXf System::particleAcceleration()
{
    clearForces();
    applyForces();
    // TBD: Compute constraint force
    ConstraintMaintainer::maintainConstraint(this, 100.0f, 10.0f);
    
    return particleDerivative();
}

VectorXf System::particleDerivative()
{
    VectorXf dst(this->particleDims());
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

void System::simulationStep()
{
    solver->simulateStep(this, dt);
}

void System::clearForces() 
{
    for (Particle *p : particles) 
    {
        p->m_Force = Vec2f(0.0f, 0.0f);
    }
}

void System::applyForces() 
{
    for (Force *f : forces) 
    {
        f->apply(springsCanBreak);
        // printf("hi");
    }
}

void System::drawParticles()
{
    for (auto *p : particles)
    {
        p->draw();
        // std::cout<<p->m_Position<<std::endl;
    }
}

void System::drawForces()
{
    for (auto *f : forces)
    {
        // f->draw();
    }
}

void System::drawConstraints()
{
    for (auto *c : constraints)
    {
        c->draw();
    }
}

void System::drawSystem()
{
    drawParticles();
    drawForces();
    drawConstraints();
}