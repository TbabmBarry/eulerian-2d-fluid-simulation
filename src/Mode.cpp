#include "Particle.h"
#include "DragForce.h"
#include "GravityForce.h"
#include "SpringForce.h"
#include "GridForce.h"
#include "AngularSpring.h"
#include "CollisionForce.h"
#include "GridForce.h"
#include "RodConstraint.h"
#include "CircularWireConstraint.h"
#include "FixedPointConstraint.h"
#include "System.h"
#include "EulerSolver.h"
#include "imageio.h"
#include "GravityForce.h"

#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <GL/glut.h>
#include <gfx/vec2.h>

#include "unistd.h"
#include "Mode.h"

void Mode::RigidBodyCollision(System *sys, Fluid *fluid)
{
    // for test
    sys->wall = true;
    sys->addRigid(new Particle(Vector2f(-0.2f, 0.2f), 5.0f, 0, Particle::RIGID));
    sys->addRigid(new Particle(Vector2f(0.5f, -0.4f), 5.0f, 0, Particle::RIGID));
    sys->addRigidForce(new GridForce(sys->rigidbodies, fluid, 1000));
    sys->addRigidForce(new CollisionForce(sys->rigidbodies, 0, 0.9));

    // sys->addRigid(new Particle(Vector2f(0.0f, 0.0f), 5.1f, 0, Particle::RIGID));
    // sys->addRigid(new Particle(Vector2f(-0.5f, 0.5f), 5.1f, 1, Particle::RIGID));
    // sys->addRigidForce(new CollisionForce(sys->rigidbodies, 0.0f, 0.8f));
}

void Mode::RigidBody(System *sys, Fluid *fluid)
{
    sys->wall = true;
    sys->addRigid(new Particle(Vector2f(-0.2f, 0.2f), 5.0f, 0, Particle::RIGID));
    sys->addRigidForce(new GridForce(sys->rigidbodies, fluid, 1000));
}

void Mode::Fix(System *sys, Fluid *fluid)
{
    sys->wall = true;
    sys->addRigid(new Particle(Vector2f(-0.2f, 0.2f), 5.0f, 0, Particle::FIX));
    sys->addRigidForce(new GridForce(sys->rigidbodies, fluid, 1000));
}

void Mode::Move(System *sys, Fluid *fluid)
{
    sys->wall = true;
    sys->addRigid(new Particle(Vector2f(-0.2f, 0.2f), 5.0f, 0, Particle::MOVING));
    sys->addRigidForce(new GridForce(sys->rigidbodies, fluid, 1000));
}
void Mode::FluidCloth(System *sys)
{
    const int xSize = 5, ySize = 5;
    const float dist = 0.3;
    int index = 0;

    for (int j = 0; j < ySize; j++)
    {
        for (int i = 0; i < xSize; i++)
        {
            sys->addParticle(new Particle(Vector2f(-0.6f + i * dist, 0.4f - j * dist), 0.1f, index, Particle::NORMAL));
            index++;
        }
    }

    float ks = 250.0f;
    float kd = 1.5f;
    sys->wall = true;
    // const Vector2f center(-0.8f, 0.6f);

    // sys->addConstraint(new CircularWireConstraint(sys->particles[0], center, sqrt(2) * 0.2));
    // sys->addForce(new GravityForce(sys->particles, Vector2f(0.0f, -9.8f)));
    sys->addForce(new DragForce(sys->particles, 0.3f));
    sys->wall = true;
    for (int j = 0; j < ySize; j++)
    { //right,left
        for (int i = 0; i < xSize - 1; i++)
        {
            sys->addForce(new SpringForce(sys->particles[i + j * xSize],
                                          sys->particles[i + 1 + j * xSize],
                                          dist, ks, kd));
        }
    }

    for (int j = 0; j < ySize - 1; j++)
    { //up,down
        for (int i = 0; i < xSize; i++)
        {
            sys->addForce(new SpringForce(sys->particles[i + j * xSize],
                                          sys->particles[i + (j + 1) * xSize],
                                          dist, ks, kd));
        }
    }

    for (int j = 0; j < ySize - 1; j++)
    { //diagonal
        for (int i = 0; i < xSize - 1; i++)
        {
            sys->addForce(new SpringForce(sys->particles[i + j * xSize],
                                          sys->particles[i + 1 + (j + 1) * xSize],
                                          sqrt(pow(dist, 2) + pow(dist, 2)), ks, kd));
            //   sqrt(pow(deltax, 2) + pow(deltay, 2) + pow(deltay, 2)), ks, kd));
        }
    }

    for (int y = 0; y < ySize - 1; y++)
    { //diagonal
        for (int x = 1; x < xSize; x++)
        {
            sys->addForce(new SpringForce(sys->particles[x + y * xSize],
                                          sys->particles[x - 1 + (y + 1) * xSize],
                                          sqrt(pow(dist, 2) + pow(dist, 2)), ks, kd));
        }
    }
}
