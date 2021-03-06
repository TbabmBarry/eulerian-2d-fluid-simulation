#include "System.h"
#include "ConstraintMaintainer.h"

System::System(Solver *solver, Fluid *fluid) : solver(solver), fluid(fluid), wall(false), time(0.0f), dt(0.001f)
{
}

void System::addParticle(Particle *p)
{
    if (p->rigid == 0)
    {
        particles.push_back(p);
    }
}
void System::addRigid(Particle *p)
{
    if (p->rigid != 0)
    {
        rigidbodies.push_back(p);
    }
}

void System::addForce(Force *f)
{
    forces.push_back(f);
}

void System::addRigidForce(Force *f)
{
    rigidForces.push_back(f);
}

void System::addConstraint(Constraint *constraint)
{
    constraints.push_back(constraint);
}

int System::particleDims()
{
    return particles.size() * 2 * 2;
}
int System::rigidDims()
{
    return rigidbodies.size() * 8;
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
        // s[i * 4 + 2] = particles[i]->m_Velocity[0];
        s[i * 4 + 2] = fluid->getXVelocity(particles[i]->LocolGrid(128)[0], particles[i]->LocolGrid(128)[1]) * 50;
        // s[i * 4 + 3] = particles[i]->m_Velocity[1];
        s[i * 4 + 3] = fluid->getYVelocity(particles[i]->LocolGrid(128)[0], particles[i]->LocolGrid(128)[1]) * 50;
    }
    return s;
}
VectorXf System::rigidGetState()
{
    VectorXf s(this->rigidDims());
    for (int i = 0; i < rigidbodies.size(); i++)
    {
        s[i * 8 + 0] = rigidbodies[i]->x[0];
        s[i * 8 + 1] = rigidbodies[i]->x[1];
        s[i * 8 + 2] = 0;
        s[i * 8 + 3] = rigidbodies[i]->P[0];
        s[i * 8 + 4] = rigidbodies[i]->P[1];
        s[i * 8 + 5] = rigidbodies[i]->L;
        s[i * 8 + 6] = rigidbodies[i]->mass;
        s[i * 8 + 7] = rigidbodies[i]->I;
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
void System::rigidSetState(VectorXf src)
{
    this->rigidSetState(src, this->particleGetTime());
}

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
void System::rigidSetState(VectorXf newState, float time)
{
    for (int i = 0; i < rigidbodies.size(); i++)
    {
        for (int k = 0; k < rigidbodies[i]->corners.size(); ++k)
        {
            rigidbodies[i]->corners[k] -= rigidbodies[i]->x;
        }
        rigidbodies[i]->x[0] = newState[i * 8 + 0];
        rigidbodies[i]->x[1] = newState[i * 8 + 1];
        rigidbodies[i]->angle = rigidbodies[i]->rigid != 1 ? 0.0f : newState[i * 8 + 2];
        rigidbodies[i]->P[0] = rigidbodies[i]->rigid == 2 ? 0.0f : newState[i * 8 + 3];
        rigidbodies[i]->P[1] = rigidbodies[i]->rigid == 2 ? 0.0f : newState[i * 8 + 4];
        rigidbodies[i]->L = rigidbodies[i]->rigid != 1 ? 0.0f : newState[i * 8 + 5];
        // cout << "linear momentum: " << rigidbodies[i]->P << endl;
        //Compute derived variables
        rigidbodies[i]->R(0, 0) = cos(rigidbodies[i]->angle);
        rigidbodies[i]->R(0, 1) = -sin(rigidbodies[i]->angle);
        rigidbodies[i]->R(1, 0) = sin(rigidbodies[i]->angle);
        rigidbodies[i]->R(1, 1) = cos(rigidbodies[i]->angle);
        rigidbodies[i]->m_Velocity = rigidbodies[i]->rigid == 2 ? Vector2f(0.0f, 0.0f) : (rigidbodies[i]->P / rigidbodies[i]->mass);
        rigidbodies[i]->I = rigidbodies[i]->mass * (pow(rigidbodies[i]->dimension, 2) + pow(rigidbodies[i]->dimension, 2));
        rigidbodies[i]->omega = rigidbodies[i]->L / (rigidbodies[i]->I + 0.00000000001);
        if (rigidbodies[i]->omega > 0.1)
        {
            rigidbodies[i]->omega = 0.1;
        }
        if (rigidbodies[i]->omega < -0.1)
        {
            rigidbodies[i]->omega = -0.1;
        }
        // cout << "velocity: " << rigidbodies[i]->v << endl;
        // cout << "angle: " << rigidbodies[i]->angle << endl;
        // cout << "angular momentum: " << rigidbodies[i]->L << endl;
        // cout << "angular velocity: " << rigidbodies[i]->omega << endl;
        //update positions
        for (int k = 0; k < rigidbodies[i]->corners.size(); ++k)
        {
            //corners rotated pos = corner pos*R + masscenter pos
            rigidbodies[i]->corners[k] = rigidbodies[i]->R * rigidbodies[i]->corners[k] + rigidbodies[i]->x;
        }

        // vector<Vector4f> boundgrids = rigidbodies[i]->BoundingGrid(8);
        // cout<< "new_bound" << endl;
        // for (int i=0; i< boundgrids.size();i++){
        //     cout<< "bound_grids"<< boundgrids[i][0] << " " << boundgrids[i][1] << " "  << boundgrids[i][2] << " "  << boundgrids[i][3] <<endl;
        // }

        // vector<Vector2i> innergrids = rigidbodies[i]->InnerGrid(boundgrids);
        // cout<< "new_inner" << endl;
        // for (int i=0; i< innergrids.size();i++){
        //     cout<< "inner_grids"<< innergrids[i][0] << " " << innergrids[i][1]  <<endl;
        //     if (innergrids[i][0] > 10) break;
        // }
    }
    this->time = time;
}

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

VectorXf System::collisionValidationRigid(VectorXf newState)
{
    for (int i = 0; i < rigidbodies.size(); i++)
    {
        if (newState[i * 8] < -(0.9f - rigidbodies[i]->dimension / 2))
        {
            // particles[i]->m_Force+= Vec2f(10,0);
            newState[i * 8] = -(0.9f - rigidbodies[i]->dimension / 2);
            newState[i * 8 + 3] *= -1;
        }

        if (newState[i * 8] > (0.9f - rigidbodies[i]->dimension / 2))
        {
            // particles[i]->m_Force+= Vec2f(-10,0);
            newState[i * 8] = (0.9f - rigidbodies[i]->dimension / 2);
            newState[i * 8 + 3] *= -1;
        }

        if (newState[i * 8 + 1] < -(0.9f - rigidbodies[i]->dimension / 2))
        {
            // particles[i]->m_Force+= Vec2f(0,100);
            newState[i * 8 + 1] = -(0.9f - rigidbodies[i]->dimension / 2);
            newState[i * 8 + 4] *= -1;
        }

        if (newState[i * 8 + 1] > (0.9f - rigidbodies[i]->dimension / 2))
        {
            newState[i * 8 + 1] = (0.9f - rigidbodies[i]->dimension / 2);
            newState[i * 8 + 4] *= -1;
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

VectorXf System::rigidAcceleration()
{
    clearRigidForces();
    applyRigidForces();
    return rigidDerivative();
}

VectorXf System::rigidDerivative()
{
    VectorXf y(this->rigidDims());
    for (int i = 0; i < rigidbodies.size(); i++)
    {
        Particle *rb = rigidbodies[i];
        // updateForce();
        // updateTorque();
        y[i * 8 + 0] = rb->m_Velocity[0];
        y[i * 8 + 1] = rb->m_Velocity[1];
        y[i * 8 + 2] = rb->omega;
        y[i * 8 + 3] = rb->m_Force[0];
        y[i * 8 + 4] = rb->m_Force[1];
        y[i * 8 + 5] = rb->torque;
        y[i * 8 + 6] = 0.0f;
        y[i * 8 + 7] = 0.0f;
    }
    return y;
}

void System::simulationStep()
{
    solver->simulateStep(this, dt);
}

void System::clearForces()
{
    for (Particle *p : particles)
    {
        p->m_Force = Vector2f(0.0f, 0.0f);
    }
}

void System::clearRigidForces()
{
    for (Particle *rb : rigidbodies)
    {
        rb->m_Force = Vector2f(0.0f, 0.0f);
        rb->torque = 0.0f;
    }
}

void System::applyForces()
{
    for (int i = 0; i < forces.size(); i++)
    {
        forces[i]->apply(springsCanBreak);
    }
}

void System::applyRigidForces()
{
    for (int i = 0; i < rigidForces.size(); i++)
    {
        rigidForces[i]->apply(springsCanBreak);
    }
}

void System::drawParticles()
{
    for (auto *p : particles)
    {
        p->draw();
        // p->drawLocal();
    }
}
void System::drawRigids()
{
    for (auto *r : rigidbodies)
    {
        r->draw();
        // r->drawBound();
        // r->drawInner();
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
    drawRigids();
    drawForces();
    drawConstraints();
}