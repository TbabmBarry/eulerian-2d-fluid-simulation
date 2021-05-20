#include "Particle.h"
#include "DragForce.h"
#include "GravityForce.h"
#include "SpringForce.h"
#include "AngularSpring.h"
#include "RodConstraint.h"
#include "CircularWireConstraint.h"
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

void Mode::Spring(System* sys) {
    const double dist = 0.2;
	const Vec2f center(0.0, 0.0);
	const Vec2f offset(dist, 0.0);

    sys->addParticle(new Particle(center + offset, 10.0f, 0));
	// printf("1st");
	sys->addParticle(new Particle(center + 2 * offset, 10.0f, 1));
	sys->addForce(new SpringForce(sys->particles[0], sys->particles[1], dist/2, 10.f, 1.0f));

}


void Mode::SpringRod(System* sys) {
    const double dist = 0.2;
	const Vec2f center(0.0, 0.0);
	const Vec2f offset(dist, 0.0);

    sys->addParticle(new Particle(center + offset, 10.0f, 0));
	// printf("1st");
	sys->addParticle(new Particle(center + 2 * offset, 10.0f, 1));
    sys->addParticle(new Particle(center + 3 * offset, 10.0f, 2));
	sys->addParticle(new Particle(center + 4 * offset, 10.0f, 3));
	sys->addForce(new SpringForce(sys->particles[0], sys->particles[1], dist/2, 10.f, 1.0f));
    sys->addForce(new SpringForce(sys->particles[2], sys->particles[3], dist/2, 10.f, 1.0f));
    sys->addConstraint(new RodConstraint(sys->particles[1], sys->particles[2], dist));

}


void Mode::SpringCircular(System* sys) {
    const double dist = 0.2;
	const Vec2f center(0.0, 0.0);
	const Vec2f offset(dist, 0.0);

    sys->addParticle(new Particle(center + offset, 10.0f, 0));
	// printf("1st");
	// sys->addParticle(new Particle(center + 2 * offset, 10.0f, 1));
	// printf("2nd");
	// sys->addParticle(new Particle(center + 3 * offset, 10.0f, 2));
	// sys->addParticle(new Particle(center + 4 * offset, 10.0f, 3));
	// sys->addParticle(new Particle(center + 5 * offset, 10.0f, 4));
	// sys->addParticle(new Particle(center + 4 * offset, 2.0f, 5));

	// You shoud replace these with a vector generalized forces and one of
	// constraints...
	// delete_this_dummy_spring = new SpringForce(pVector[0], pVector[1], dist, 1.0, 1.0);
	// delete_this_dummy_rod = new RodConstraint(pVector[1], pVector[2], dist);
	// delete_this_dummy_wire = new CircularWireConstraint(pVector[0], center, dist);

	sys->addForce(new GravityForce(sys->particles, Vec2f(0.0f, -9.8f)));
	// sys->addForce(new SpringForce(sys->particles[0], sys->particles[1], dist/2, 10.f, 1.0f));
	// sys->addForce(new SpringForce(sys->particles[2], sys->particles[3], dist/2, 10.f, 1.0f));
	// sys->addForce(new SpringForce(sys->particles[3], sys->particles[4], dist, 10.f, 1.0f));
    // sys->addConstraint(new RodConstraint(sys->particles[1], sys->particles[2], dist));
	// sys->addConstraint(new RodConstraint(sys->particles[2], sys->particles[3], dist));
	// sys->addConstraint(new RodConstraint(sys->particles[1], sys->particles[3], dist));
	sys->addConstraint(new CircularWireConstraint(sys->particles[0], center, dist));
}

void Mode::Rod(System *sys) {
    const double dist = 0.2;
	const Vec2f center(0.0, 0.0);
	const Vec2f offset(dist, 0.0);

    sys->addParticle(new Particle(center + offset, 10.0f, 0));
	// printf("1st");
	sys->addParticle(new Particle(center + 2 * offset, 10.0f, 1));
    sys->addParticle(new Particle(center + 3 * offset, 10.0f, 2));

    sys->addForce(new SpringForce(sys->particles[0], sys->particles[1], dist/2, 10.f, 1.0f));
    sys->addConstraint(new RodConstraint(sys->particles[1], sys->particles[2], dist));
    sys->addConstraint(new CircularWireConstraint(sys->particles[0], center, dist));
}

void Mode::Gravity(System *sys) {
    const double dist = 0.2;
	const Vec2f center(0.0, 0.0);
	const Vec2f offset(0.0, -dist);

    sys->addParticle(new Particle(center + offset, 1.0f, 0));
	sys->addParticle(new Particle(center + 2 * offset, 1.0f, 1));

    sys->addForce(new GravityForce(sys->particles, Vec2f(0.0f, -9.8f)));
    sys->addForce(new SpringForce(sys->particles[0], sys->particles[1], dist, 10.f, 1.0f));
    sys->addConstraint(new CircularWireConstraint(sys->particles[0], center, dist));
}


void Mode::cloth(System *sys) {
    const int xSize = 3, ySize = 3;
    const float dist = 0.3;
    int index = 0;

    for (int j = 0; j < ySize; j++) {
        for (int i = 0; i < xSize; i++) {
            sys->addParticle(new Particle(Vec2f(0.0f + i * dist, 0.0f - j * dist), 0.2f, index));
            index++;
        }
    }

    float ks = 10.0f;
    float kd = 1.5f;

    for (int j = 0; j < ySize; j++) {//right,left
        for (int i = 0; i < xSize - 1; i++) {
            sys->addForce(new SpringForce(sys->particles[i + j * xSize],
                                          sys->particles[i + 1 + j * xSize],
                                          dist, ks, kd));
        }
    }

    for (int j = 0; j < ySize - 1; j++) {//up,down
        for (int i = 0; i < xSize; i++) {
            sys->addForce(new SpringForce(sys->particles[i + j * xSize],
                                          sys->particles[i + (j + 1) * xSize],
                                          dist/2, ks, kd));
        }
    }

    for (int j = 0; j < ySize - 1; j++) {//diagonal
        for (int i = 0; i < xSize - 1; i++) {
            sys->addForce(new SpringForce(sys->particles[i + j * xSize],
                                          sys->particles[i + 1 + (j + 1) * xSize],
                                          sqrt(pow(dist,2) + pow(dist/2,2)), ks, kd));
                                        //   sqrt(pow(deltax, 2) + pow(deltay, 2) + pow(deltay, 2)), ks, kd));
        }
    }

    for (int y = 0; y < ySize - 1; y++) {//diagonal
        for (int x = 1; x < xSize; x++) {
            sys->addForce(new SpringForce(sys->particles[x + y * xSize],
                                          sys->particles[x - 1 + (y + 1) * xSize],
                                          sqrt(pow(dist,2) + pow(dist/2,2)), ks, kd));
        }
    }
}



// void Mode::cloth(System *sys) {

//     const int xSize = 3, ySize = 3;
//     const float deltax = 2.0f/xSize, deltay = 2.5f/ySize;
//     int index = 0;
//     // Initialize particles
//     for (int j = 0; j < ySize; j++) {
//         for (int i = 0; i < xSize; i++) {
//             sys->addParticle(new Particle(Vec2f(-0.5f + i * deltax, 0.5f - j * deltay), 0.2f, index));
//             index++;
//         }
//     }
//     // Add gravity and drag to all particles
//     // sys->addForce(new GravityForce(sys->particles, Vec2f(0, -9.81f)));

//     float ks = 10.0f;
//     float kd = 1.5f;

//     for (int j = 0; j < ySize; j++) {//right,left
//         for (int i = 0; i < xSize - 1; i++) {
//             sys->addForce(new SpringForce(sys->particles[i + j * xSize],
//                                           sys->particles[i + 1 + j * xSize],
//                                           deltax, ks, kd));
//         }
//     }

//     for (int j = 0; j < ySize - 1; j++) {//up,down
//         for (int i = 0; i < xSize; i++) {
//             sys->addForce(new SpringForce(sys->particles[i + j * xSize],
//                                           sys->particles[i + (j + 1) * xSize],
//                                           deltay, ks, kd));
//         }
//     }

//     for (int j = 0; j < ySize - 1; j++) {//diagonal
//         for (int i = 0; i < xSize - 1; i++) {
//             sys->addForce(new SpringForce(sys->particles[i + j * xSize],
//                                           sys->particles[i + 1 + (j + 1) * xSize],
//                                           sqrt(pow(deltax, 2) + pow(deltay, 2)), ks, kd));
//                                         //   sqrt(pow(deltax, 2) + pow(deltay, 2) + pow(deltay, 2)), ks, kd));
//         }
//     }

//     for (int y = 0; y < ySize - 1; y++) {//diagonal
//         for (int x = 1; x < xSize; x++) {
//             sys->addForce(new SpringForce(sys->particles[x + y * xSize],
//                                           sys->particles[x - 1 + (y + 1) * xSize],
//                                           sqrt(pow(deltax, 2) + pow(deltay, 2)), ks, kd));
//         }
//     }

//     float radius = 0.05f;
//     // sys->addConstraint(new CircularWireConstraint(sys->particles[0],
//     //                                               Vec2f(0.0f,0.0f) + Vec2f(-radius, 0.f),
//     //                                               radius));
//     // sys->addConstraint(new CircularWireConstraint(sys->particles[xSize-1],
//     //                                               Vec2f(0.0f,0.0f) + Vec2f(-radius, 0.f),
//     //                                               radius));

// }

void Mode::hair(System *sys){

}

void Mode::mouse(System *sys){

}