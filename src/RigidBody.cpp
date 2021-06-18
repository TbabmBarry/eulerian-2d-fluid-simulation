// #include "RigidBody.h"
// #include <GL/glut.h>
// #include <iostream>
// #ifndef M_PI
//     #define M_PI 3.14159265358979323846
// #endif

// RigidBody::RigidBody(Vec2f MassCenter, Vec2f numParticles, float particleMass) :
//         MassCenter(MassCenter), dimension(dimension) {
//     initializeVariables();

//     float density = particleMass / (dimension[0]*dimension[1]); //mass/(x*y)
//     //generate particles with body dimensions
//     //assume mass center (0,0) for convenience
//     int index = 0;
//     for (int x = 0; x < numParticles[0]; x++) {
//         for (int y = 0; y < numParticles[1]; y++) {//global coordinate
//             float xStart = -dimension[0] / 2 + dimension[0] * (float) x / (numParticles[0] - 1);
//             float yStart = -dimension[1] / 2 + dimension[1] * (float) y / (numParticles[1] - 1);
//             Particle *p = new Particle(Vec2f(xStart, yStart), particleMass, index, true);
//             //A rigid body has constant density
//             // p->density = density;
//             particles.push_back(p);
//             index++;
//         }
//     }
//     //Calculate total mass
//     for (Particle *p : particles) {
//         M += p->mass;
//     }
//     //Calculate Ibody
//     // Ibody = M* (pow(dimension[0],2)+pow(dimension[0],2));
// }

// RigidBody::~RigidBody(void) {
// }

// void RigidBody::reset() {
//     initializeVariables();
//     for (Particle *p : particles) {
//         p->reset();
//     }
// }

// void RigidBody::initializeVariables() {
//     x = MassCenter;
//     R = Matrix2f::Identity();
//     P = Vec2f(0, 0);//M*v(t)
//     L = 0.0;//I*w(t)
//     v = Vec2f(0, 0);
//     omega = L/I;
//     force = Vec2f(0, 0);
//     torque = 0.0;
//     angle = 10 * 180 / M_PI;
// }

// // Vec2f RigidBody::getBodyCoordinates(Vec2f world) {
// //     return R.transpose() * (world - x);
// // }

// void RigidBody::updateForce() {
//     force = Vec2f(0, 0);
//     for (Particle *p : particles)
//         force += p->m_Force;
// }

// void RigidBody::updateTorque() {
//     torque = 0.0;
//     for (Particle *p : particles)
//         torque += (p->m_Position[0] - x[0])*p->m_Force[1] 
//         - (p->m_Position[1] - x[1])*p->m_Force[0]; //(0,0,torque)=(0,0,x*Fy-y*Fx) in 2D
// }

// void RigidBody::setState(VectorXf newState) {
//     x[0] = newState[0];
//     x[1] = newState[1];
//     R[0] = newState[2];
//     R[1] = newState[3];
//     R[2] = newState[4];
//     R[3] = newState[5];
//     //cos(angle), -sin(angle), sin(angle), cos(angle)); //counter-clockwise
//     P[0] = newState[6];
//     P[1] = newState[7];
//     L = newState[8];

//     //Compute derived variables
//     v = P / M;
//     I = M* (pow(dimension[0],2)+pow(dimension[0],2));
//     omega = L/I;

//     //update positions
//     for (Particle * p : particles) {
//         p->m_Position = R * p->m_Position + x;
//     }
// }

// /*
//  * pack x, R, P and L into a single vector
//  */
// VectorXf RigidBody::getState() {
//     VectorXf state(9);
//     state[0] = x[0];
//     state[1] = x[1];
//     state[2] = R[0];
//     state[3] = R[1];
//     state[4] = R[2];
//     state[5] = R[3];
//     state[6] = P[0];
//     state[7] = P[1];
//     state[8] = L;
//     return state;
// }

// VectorXf RigidBody::getDerivativeState() {
//     updateForce();
//     updateTorque();
//     VectorXf y(9);
//     //xdot, i.e velocity
//     y[0] = v[0];
//     y[1] = v[1];

//     //calculate product, convert to resulting matrix to quaternion
//     y[2] = omega * R[0];
//     y[3] = omega * R[1];
//     y[4] = omega * R[2];
//     y[5] = omega * R[3];

//     //Pdot = F
//     y[6] = force[0];
//     y[7] = force[1];

//     //Ldot = torque
//     y[8] = torque;
//     return y;
// }

// void RigidBody::draw(bool drawVelocity, bool drawForce) {
//     Vec2f v1 = R * Vec2f(-dimension[0] / 2, -dimension[1] / 2) + x;
//     Vec2f v2 = R * Vec2f(dimension[0] / 2, -dimension[1] / 2) + x;
//     Vec2f v3 = R * Vec2f(-dimension[0] / 2, -dimension[1] / 2) + x;
//     Vec2f v4 = R * Vec2f(dimension[0] / 2, -dimension[1] / 2) + x;
//     Vec2f v5 = R * Vec2f(-dimension[0] / 2, dimension[1] / 2) + x;
//     Vec2f v6 = R * Vec2f(dimension[0] / 2, dimension[1] / 2) + x;
//     Vec2f v7 = R * Vec2f(-dimension[0] / 2, dimension[1] / 2) + x;
//     Vec2f v8 = R * Vec2f(dimension[0] / 2, dimension[1] / 2) + x;
//     glBegin(GL_LINES);
//     // glColor2f(1.f, 1.f);
//     glVertex2f(v1[0], v1[1]);
//     glVertex2f(v2[0], v2[1]);
//     glVertex2f(v1[0], v1[1]);
//     glVertex2f(v3[0], v3[1]);
//     glVertex2f(v2[0], v2[1]);
//     glVertex2f(v4[0], v4[1]);
//     glVertex2f(v3[0], v3[1]);
//     glVertex2f(v4[0], v4[1]);

//     glVertex2f(v5[0], v5[1]);
//     glVertex2f(v6[0], v6[1]);
//     glVertex2f(v5[0], v5[1]);
//     glVertex2f(v7[0], v7[1]);
//     glVertex2f(v6[0], v6[1]);
//     glVertex2f(v8[0], v8[1]);
//     glVertex2f(v7[0], v7[1]);
//     glVertex2f(v8[0], v8[1]);

//     glVertex2f(v1[0], v1[1]);
//     glVertex2f(v5[0], v5[1]);
//     glVertex2f(v2[0], v2[1]);
//     glVertex2f(v6[0], v6[1]);
//     glVertex2f(v3[0], v3[1]);
//     glVertex2f(v7[0], v7[1]);
//     glVertex2f(v4[0], v4[1]);
//     glVertex2f(v8[0], v8[1]);
//     glEnd();

// }