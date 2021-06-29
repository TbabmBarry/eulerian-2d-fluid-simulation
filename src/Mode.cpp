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

void Mode::CircularGravityRod(System *sys)
{
    const double dist = 0.2;
    const Vector2f center(0.0, 0.0);
    const Vector2f offset(dist, 0.0);

    sys->addParticle(new Particle(center + offset, 10.0f, 0, Particle::NORMAL));
    // printf("1st");
    sys->addParticle(new Particle(center + 2 * offset, 1.0f, 1, Particle::NORMAL));
    sys->addParticle(new Particle(center + 3 * offset, 1.0f, 2, Particle::NORMAL));
    sys->addForce(new GravityForce(sys->particles, Vector2f(0.0f, -9.8f)));
    sys->addForce(new SpringForce(sys->particles[0], sys->particles[1], dist / 2, 120.f, 1.0f));
    sys->addConstraint(new RodConstraint(sys->particles[1], sys->particles[2], dist));
    sys->addConstraint(new CircularWireConstraint(sys->particles[0], center, dist));
}

void Mode::CircularCloth(System *sys)
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
    const Vector2f center(-0.8f, 0.6f);

    sys->addConstraint(new CircularWireConstraint(sys->particles[0], center, sqrt(2) * 0.2));
    sys->addForce(new GravityForce(sys->particles, Vector2f(0.0f, -9.8f)));
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

void Mode::hair(System *sys)
{
    vector<Vector2f> center;

    const int numHairs = 20;
    const int num_particles = 12;
    for (int i = 0; i < numHairs; ++i)
    {
        center.push_back(Vector2f(0.0 + 0.03 * i, 0.5f));
    }
    // Vec2f step = (end-start)/(num_particles+1);
    float ks = 180.0f;
    float kd = 1.5f;
    sys->wall = true;

    for (int i = 0; i < numHairs; i++)
    {
        // Initialize particles
        for (int j = 0; j < num_particles; j++)
        {
            if (j == 0)
            {
                sys->addParticle(new Particle(Vector2f(0.0f + 0.03f * i, 0.5f - j * 0.05), 0.1f, i * num_particles + j, Particle::NORMAL));
            }
            else
            {
                sys->addParticle(new Particle(Vector2f(0.0f + 0.03f * i + pow(-1, j) * 0.02, 0.5f - j * 0.05), 0.1f, i * num_particles + j, Particle::NORMAL));
            }
        }
        for (int j = 0; j < num_particles - 1; j++)
        {
            sys->addForce(new SpringForce(sys->particles[i * num_particles + j],
                                          sys->particles[i * num_particles + j + 1],
                                          0.05, ks, kd));
        }

        for (int j = 2; j < num_particles - 2; j++)
        {
            sys->addForce(new AngularSpring(sys->particles[i * num_particles + j],
                                            sys->particles[i * num_particles + j + 1],
                                            sys->particles[i * num_particles + j + 2],
                                            90, ks / 2, kd));
        }
        sys->addConstraint(new FixedPointConstraint(sys->particles[i * num_particles],
                                                    center[i]));
    }
    // Add gravity and drag to all particles
    sys->addForce(new GravityForce(sys->particles, Vector2f(0.0f, -9.8f)));
    sys->addForce(new DragForce(sys->particles, 0.3f));
}

// void Mode::RigidBodyCollision(System *sys, FluidSolver *fluid)
// {
//     // for test
//     sys->wall = true;
//     sys->addRigid(new Particle(Vector2f(-0.125f, 0.125f), 5.0f, 0, Particle::RIGID));
//     sys->addRigidForce(new GridForce(sys->rigidbodies, fluid));

//     // sys->addRigid(new Particle(Vector2f(0.0f, 0.0f), 5.1f, 0, Particle::RIGID));
//     // sys->addRigid(new Particle(Vector2f(-0.5f, 0.5f), 5.1f, 1, Particle::RIGID));
//     // sys->addRigidForce(new CollisionForce(sys->rigidbodies, 0.0f, 0.8f));
// }

void Mode::RigidBodyCollision(System *sys, FluidField *fluid)
{
    // for test
    sys->wall = true;
    sys->addRigid(new Particle(Vector2f(-0.125f, 0.125f), 5.0f, 0, Particle::RIGID));
    sys->addRigidForce(new GridForce(sys->rigidbodies, fluid));

    // sys->addRigid(new Particle(Vector2f(0.0f, 0.0f), 5.1f, 0, Particle::RIGID));
    // sys->addRigid(new Particle(Vector2f(-0.5f, 0.5f), 5.1f, 1, Particle::RIGID));
    // sys->addRigidForce(new CollisionForce(sys->rigidbodies, 0.0f, 0.8f));
}