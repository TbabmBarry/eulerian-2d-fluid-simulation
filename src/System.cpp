#include "System.h"
#include "ConstraintMaintainer.h"


System::System(Solver *solver) : solver(solver), wall(false), time(0.0f), dt(0.001f)
{
}

void System::addParticle(Particle* p)
{
    if(p->rigid==0){
        particles.push_back(p);
    }
}
void System::addRigid(Particle* p)
{
    if(p->rigid==1){
        rigidbodies.push_back(p);
    }
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
int System::rigidDims()
{
    return rigidbodies.size() * 9;
}

void System::free() 
{
    rigidbodies.clear();
    particles.clear();
    forces.clear();
}

void System::reset() 
{
    for (Particle *p : particles) 
    {
        p->reset();
    }
    for (Particle *r : rigidbodies) 
    {
        r->reset();
    }
}

VectorXf System::particleGetState()
{
    VectorXf s(this->particleDims());
    for (int i = 0; i < particles.size(); i++)
    {
        s[i * 4 + 0] = particles[i]->m_Position[0];
        s[i * 4 + 1] = particles[i]->m_Position[1];
        s[i * 4 + 2] = particles[i]->m_Velocity[0];
        s[i * 4 + 3] = particles[i]->m_Velocity[1];
    }
    return s;
}
// VectorXf System::rigidGetState()
// {
//     VectorXf s(this->rigidDims());
//     for (int i = 0; i < rigidbodies.size(); i++)
//     {
//         s[i * 9 + 0] = rigidbodies[i]->x[0];
//         s[i * 9 + 1] = rigidbodies[i]->x[1];
//         s[i * 9 + 2] = rigidbodies[i]->R[0];
//         s[i * 9 + 3] = rigidbodies[i]->R[1];
//         s[i * 9 + 4] = rigidbodies[i]->R[2];
//         s[i * 9 + 5] = rigidbodies[i]->R[3];
//         s[i * 9 + 6] = rigidbodies[i]->P[0];
//         s[i * 9 + 7] = rigidbodies[i]->P[1];
//         s[i * 9 + 8] = rigidbodies[i]->L;
//     }
//     return s;
// }

float System::particleGetTime()
{
    return time;
}

void System::particleSetState(VectorXf src)
{
    this->particleSetState(src, this->particleGetTime());
}
// void System::rigidSetState(VectorXf src)
// {
//     this->rigidSetState(src, this->particleGetTime());
// }

void System::particleSetState(VectorXf newState, float time)
{
    for (int i = 0; i < particles.size(); i++)
    {
        particles[i]->m_Position[0] = newState[i * 4 + 0];
        particles[i]->m_Position[1] = newState[i * 4 + 1];
        particles[i]->m_Velocity[0] = newState[i * 4 + 2];
        particles[i]->m_Velocity[1] = newState[i * 4 + 3];
    }
    this->time = time;
}
// void System::rigidSetState(VectorXf newState, float time)
// {
//     for (int i = 0; i < rigidbodies.size(); i++)
//     {
//         rigidbodies[i]->x[0] = newState[i * 9 + 0];
//         rigidbodies[i]->x[1] = newState[i * 9 + 1];
//         rigidbodies[i]->R[0] = newState[i * 9 + 2];
//         rigidbodies[i]->R[1] = newState[i * 9 + 3];
//         rigidbodies[i]->R[2] = newState[i * 9 + 4];
//         rigidbodies[i]->R[3] = newState[i * 9 + 5];
//         rigidbodies[i]->P[0] = newState[i * 9 + 6];
//         rigidbodies[i]->P[1] = newState[i * 9 + 7];
//         rigidbodies[i]->L = newState[i * 9 + 8];
//         //Compute derived variables
//         rigidbodies[i]->v = rigidbodies[i]->P / rigidbodies[i]->M;
//         rigidbodies[i]->I = rigidbodies[i]->M* (pow(rigidbodies[i]->dimension,2)+pow(rigidbodies[i]->dimension,2));
//         rigidbodies[i]->omega = rigidbodies[i]->L/rigidbodies[i]->I;
//         //update positions
//         for (int k=0; k<rigidbodies[i]->corners.size();++k) {
//             //corners rotated pos = corner pos*R + masscenter pos
//             rigidbodies[i]->corners[k] = rigidbodies[i]->R * rigidbodies[i]->corners[k] + rigidbodies[i]->x;
//         }
//     }
//     this->time = time;
// }

VectorXf System::collisionValidation(VectorXf newState)
{
    for (int i = 0; i < particles.size(); i++)
    {
        if (newState[i * 4] < -0.9f)
        {
            // particles[i]->m_Force+= Vec2f(10,0);
            newState[i * 4] = -0.9f;
            newState[i * 4 + 2] = -newState[i * 4 + 2];
        }

        if (newState[i * 4] > 0.9f)
        {
            // particles[i]->m_Force+= Vec2f(-10,0);
            newState[i * 4] = 0.9f;
            newState[i * 4 + 2] = -newState[i * 4 + 2];
        }

        if (newState[i * 4 + 1] < -0.9f)
        {
            // particles[i]->m_Force+= Vec2f(0,100);
            newState[i * 4 + 1] = -0.9f;
            newState[i * 4 + 3] = -newState[i * 4 + 3];
        }


        if (newState[i * 4 + 1] > 0.9f)
        {
            // particles[i]->m_Force+= Vec2f(0,-100);
            newState[i * 4 + 1] = 0.9f;
            newState[i * 4 + 3] = -newState[i * 4 + 3];
        }
    }
    return newState;
}


VectorXf System::particleAcceleration()
{
    clearForces();

    applyForces();

    // TBD: Compute constraint force
    ConstraintMaintainer::maintainConstraint(this, 0.0f, 0.0f);
    
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
// VectorXf System::rigidDerivative()
// {
//     VectorXf y(this->rigidDims());
//     for (int i = 0; i < rigidbodies.size(); i++)
//     {
//         updateForce();
//         updateTorque();
//         VectorXf y(6);
//         //xdot, i.e velocity
//         y[0] = rigidbodies[i]->v[0];
//         y[1] = rigidbodies[i]->v[1];
//         //calculate product, convert to resulting matrix to quaternion
//         y[2] = rigidbodies[i]->omega * rigidbodies[i]->R[0];
//         y[3] = rigidbodies[i]->omega * rigidbodies[i]->R[1];
//         y[4] = rigidbodies[i]->omega * rigidbodies[i]->R[2];
//         y[5] = rigidbodies[i]->omega * rigidbodies[i]->R[3];

//         //Pdot = F
//         y[6] = rigidbodies[i]->force[0];
//         y[7] = rigidbodies[i]->force[1];

//         //Ldot = torque
//         y[8] = rigidbodies[i]->torque;
//         return y;

//         Particle *p = rigidbodies[i];
//         dst[i * 4 + 0] = p->m_Velocity[0];
//         dst[i * 4 + 1] = p->m_Velocity[1];
//         dst[i * 4 + 2] = p->m_Force[0] / p->mass;
//         dst[i * 4 + 3] = p->m_Force[1] / p->mass;
//     }
//     return dst;
// }

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
    // for (Particle *r : rigidbodies) 
    // {
    //     r->force = Vec2f(0.0f, 0.0f);
    // }
}

void System::applyForces() 
{
    for (int i = 0; i < forces.size(); i++) 
    {
        forces[i]->apply(springsCanBreak);
    }
}

void System::drawParticles()
{
    for (auto *p : particles)
    {
        p->draw();
    }
}

void System::drawForces()
{
    for (auto *f : forces)
    {
        if (f->active)
        {
            f->draw();
        }
        
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