#include "Particle.h"
#include "DragForce.h"
#include "GravityForce.h"
#include "SpringForce.h"
#include "AngularSpring.h"
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
    // sys->addForce(new GravityForce(sys->particles, Vec2f(0.0f, -9.8f)));
	sys->addForce(new SpringForce(sys->particles[0], sys->particles[1], dist/2, 10.f, 1.0f));
    sys->addForce(new SpringForce(sys->particles[2], sys->particles[3], dist/2, 10.f, 1.0f));
    sys->addConstraint(new RodConstraint(sys->particles[1], sys->particles[2], dist));
}


void Mode::SpringCircular(System* sys) {
    const double dist = 0.2;
	const Vec2f center(0.0, 0.0);
	const Vec2f offset(dist, 0.0);

    sys->addParticle(new Particle(center + offset, 10.0f, 0));
	sys->addForce(new GravityForce(sys->particles, Vec2f(0.0f, -9.8f)));
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
	sys->addForce(new GravityForce(sys->particles, Vec2f(0.0f, -9.8f)));
    sys->addForce(new SpringForce(sys->particles[0], sys->particles[1], dist/2, 10.f, 1.0f));
    sys->addConstraint(new RodConstraint(sys->particles[1], sys->particles[2], dist));
    sys->addConstraint(new CircularWireConstraint(sys->particles[0], center, dist));
}

void Mode::Gravity(System *sys) {
    const double dist = 0.2;
	const Vec2f center(0.0, 0.0);
	// const Vec2f offset(0.0, -dist);
	const Vec2f offset(dist, 0.0);

    sys->addParticle(new Particle(center + offset, 0.1f, 0));
	sys->addParticle(new Particle(center + 2 * offset, 0.1f, 1));

    sys->addForce(new GravityForce(sys->particles, Vec2f(0.0f, -9.8f)));
    sys->addForce(new SpringForce(sys->particles[0], sys->particles[1], dist, 50.f, 1.5f));
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

    float ks = 80.0f;
    float kd = 1.5f;

    const Vec2f center(0.0f, 0.0f);
    sys->addConstraint(new FixedPointConstraint(sys->particles[0], center));
    sys->addForce(new GravityForce(sys->particles, Vec2f(0.0f, -9.8f)));
    sys->addForce(new DragForce(sys->particles, 0.3f));
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
                                          dist, ks, kd));
        }
    }

    for (int j = 0; j < ySize - 1; j++) {//diagonal
        for (int i = 0; i < xSize - 1; i++) {
            sys->addForce(new SpringForce(sys->particles[i + j * xSize],
                                          sys->particles[i + 1 + (j + 1) * xSize],
                                          sqrt(pow(dist,2) + pow(dist,2)), ks, kd));
                                        //   sqrt(pow(deltax, 2) + pow(deltay, 2) + pow(deltay, 2)), ks, kd));
        }
    }

    for (int y = 0; y < ySize - 1; y++) {//diagonal
        for (int x = 1; x < xSize; x++) {
            sys->addForce(new SpringForce(sys->particles[x + y * xSize],
                                          sys->particles[x - 1 + (y + 1) * xSize],
                                          sqrt(pow(dist,2) + pow(dist,2)), ks, kd));
        }
    }
}



void Mode::hair(System *sys){
	vector<Vec2f> center;

    const int numHairs = 8;
    vector<Vec2f> start;
    const int num_particles = 12;
    vector<Vec2f> end;
    for(int i=0;i<numHairs;++i){
        center.push_back(Vec2f(0.0+0.03*i, 0.5f));
        start.push_back(Vec2f(0.0+0.03*i, -0.5f));
        end.push_back(Vec2f(0.0+0.03*i,0.5f));
    }
    Vec2f step=(end[0]-start[0])/(num_particles+1);
    const float rest = norm(step);
    // Vec2f step = (end-start)/(num_particles+1);
	float ks = 180.0f;
    float kd = 1.5f;

    for (int i = 0; i < numHairs; i++) {
        // Initialize particles
        // for (int j = 0; j < num_particles+2; j +=2) {
        //     sys->addParticle(new Particle(start[i]+step * j, 0.1f, i * (num_particles+2) + j));
        //     sys->addParticle(new Particle(start[i]+step * j + Vec2f(0.05f,0.0), 0.1f, i * (num_particles+2) + j + 1));
        // }
        // for (int j = 0; j < num_particles+1; j ++) {
        //     sys->addForce(new SpringForce(sys->particles[i * (num_particles+2) + j],
		// 									sys->particles[i * (num_particles+2) + j + 1],
		// 									rest, ks, kd));
        // }
        // for (int j = 1; j < num_particles-1; j ++) {
        //     sys->addForce(new AngularSpring(sys->particles[i * (num_particles+2) + j - 1],
		// 									sys->particles[i * (num_particles+2) + j],
		// 									sys->particles[i * (num_particles+2) + j + 1],
		// 									150, ks / 5, kd))
        // };
        for (int j = 0; j < num_particles; j++) {
            sys->addParticle(new Particle(Vec2f(0.0f + 0.03f * i, 0.5f - j * 0.05), 0.1f, i * num_particles + j));
        }
        for (int j = 0; j < num_particles - 1; j++) {
            sys->addForce(new SpringForce(sys->particles[i * num_particles + j],
                                          sys->particles[i * num_particles + j + 1],
                                          rest, ks, kd));
        }

        for (int j = 2; j < num_particles - 2; j++) {
            sys->addForce(new AngularSpring(sys->particles[i * num_particles + j],
                                                 sys->particles[i * num_particles + j + 1],
                                                 sys->particles[i * num_particles + j + 2],
                                                 120, ks / 5, kd));
        }
        sys->addConstraint(new FixedPointConstraint(sys->particles[i * num_particles],
                                                    center[i]));

        // cout<<center[i]<<endl;
        // sys->addConstraint(new FixedPointConstraint(sys->particles[i * (num_particles+2) + num_particles ], center[i]));
    }
    // Add gravity and drag to all particles
    sys->addForce(new GravityForce(sys->particles, Vec2f(0.0f, -9.8f)));
    sys->addForce(new DragForce(sys->particles, 0.3f));
}

void Mode::test(System *sys){
    const int xSize = 3, ySize = 3;
    const float dist = 0.3;
    int index = 0;

    sys->addParticle(new Particle(Vec2f(0.2f, 0.2f), 10.0f, index));
    // sys->addForce(new GravityForce(sys->particles, Vec2f(0, -9.81f)));

    // for (int j = 0; j < ySize; j++) {
    //     for (int i = 0; i < xSize; i++) {
    //         sys->addParticle(new Particle(Vec2f(0.0f + i * dist, 0.0f - j * dist), 0.2f, index));
    //         index++;
    //     }
    // }

    // float ks = 80.0f;
    // float kd = 1.5f;

    // const Vec2f center(-sqrt(2)/2*dist, sqrt(2)/2*dist);
    // // sys->addConstraint(new CircularWireConstraint(sys->particles[0], center, dist));
    // // sys->addForce(new GravityForce(sys->particles, Vec2f(0.0f, -9.8f)));
    // sys->addForce(new DragForce(sys->particles, 0.3f));
    // for (int j = 0; j < ySize; j++) {//right,left
    //     for (int i = 0; i < xSize - 1; i++) {
    //         sys->addForce(new SpringForce(sys->particles[i + j * xSize],
    //                                       sys->particles[i + 1 + j * xSize],
    //                                       dist, ks, kd));
    //     }
    // }

    // for (int j = 0; j < ySize - 1; j++) {//up,down
    //     for (int i = 0; i < xSize; i++) {
    //         sys->addForce(new SpringForce(sys->particles[i + j * xSize],
    //                                       sys->particles[i + (j + 1) * xSize],
    //                                       dist, ks, kd));
    //     }
    // }

    // for (int j = 0; j < ySize - 1; j++) {//diagonal
    //     for (int i = 0; i < xSize - 1; i++) {
    //         sys->addForce(new SpringForce(sys->particles[i + j * xSize],
    //                                       sys->particles[i + 1 + (j + 1) * xSize],
    //                                       sqrt(pow(dist,2) + pow(dist,2)), ks, kd));
    //                                     //   sqrt(pow(deltax, 2) + pow(deltay, 2) + pow(deltay, 2)), ks, kd));
    //     }
    // }

    // for (int y = 0; y < ySize - 1; y++) {//diagonal
    //     for (int x = 1; x < xSize; x++) {
    //         sys->addForce(new SpringForce(sys->particles[x + y * xSize],
    //                                       sys->particles[x - 1 + (y + 1) * xSize],
    //                                       sqrt(pow(dist,2) + pow(dist,2)), ks, kd));
    //     }
    // }
}