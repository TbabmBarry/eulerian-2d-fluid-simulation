#include "CollisionForce.h"
#include <GL/glut.h>
#include <assert.h>

CollisionForce::CollisionForce(Particle *rb1, Particle *rb2, float threshold, float epsilon) : CollisionForce({rb1, rb2}, threshold, epsilon) {}

CollisionForce::CollisionForce(vector<Particle *> rigidbodies, float threshold, float epsilon) : threshold(0.0f), epsilon(0.2f)
{
    this->setTarget(rigidbodies);
}

void CollisionForce::setTarget(vector<Particle *> rigidbodies)
{
    assert(rigidbodies.size() == 2);
    this->particles = rigidbodies;
}

void CollisionForce::apply(bool springsCanBreak)
{
    vector<Vector2f> rb1Corners = particles[0]->getBoundingBox();
    for (const auto &corner : rb1Corners)
    {
        if (colliding(corner, particles[0], particles[1]))
        {
            collision(corner, particles[0], particles[1]);
            if (containing(corner, particles[0], particles[1]))
                collision(corner, particles[0], particles[1]);
        }
    }
    vector<Vector2f> rb2Corners = particles[1]->getBoundingBox();
    for (const auto &corner : rb2Corners)
    {
        if (colliding(corner, particles[1], particles[0]))
        {
            collision(corner, particles[1], particles[0]);
            if (containing(corner, particles[1], particles[0]))
                collision(corner, particles[1], particles[0]);
        }
    }
}

void CollisionForce::draw()
{
}

bool CollisionForce::colliding(Vector2f point, Particle *rb1, Particle *rb2)
{
    // cout << " " <<endl;
    // cout << "corner: " << point << endl;
    // get the vector of points constructing the closest edge
    vector<Vector2f> edgeVec = rb2->getClosestEdge(point);
    // check if the distance from point to line close enough
    float minDist = rb1->minDistance(edgeVec[0], edgeVec[1], point);
    if (minDist > 0.02)
        return false;
    // get the edge vector
    Vector2f closestEdge = edgeVec[0] - edgeVec[1];
    // cout << " " <<endl;
    // cout << "mindist" << minDist <<endl;
    // cout << "corner: " << point << endl;
    // cout << "edge0: " << edgeVec[0] <<endl;
    // cout << "edge1: " << edgeVec[1] <<endl;
    // get the normal vector of the closest edge
    Vector2f normal = Vector2f(closestEdge[1], -closestEdge[0]).normalized();
    Vector2f ra = point - rb1->x, rb = point - rb2->x;
    Vector2f rv = (rb1->m_Velocity + rb1->omega * ra) - (rb2->m_Velocity + rb2->omega * rb);
    // Vector2f rv = (rb2->x + rb2->omega * rb) - (rb1->x - rb1->omega * rb);
    float vrel = normal.dot(rv);
    // cout << "velocity: " << rv << endl;
    // cout << "vrel: " << vrel << endl;
    if (vrel > threshold)
        return false;
    else
        return true;
}

bool CollisionForce::containing(Vector2f point, Particle *rb1, Particle *rb2)
{
    Vector2f e12 = rb2->corners[0] - rb2->corners[1];
    Vector2f e14 = rb2->corners[0] - rb2->corners[3];
    Vector2f e1i = rb2->corners[0] - point;
    float a = e1i.dot(e12), b = e1i.dot(e14);
    if (a > 0 && a < e12.dot(e12) && b > 0 && b < e14.dot(e14))
        return true;
    return false;
}

void CollisionForce::collision(Vector2f point, Particle *rb1, Particle *rb2)
{
    // get the vector of points constructing the closest edge
    vector<Vector2f> edgeVec = rb2->getClosestEdge(point);
    // get the edge vector
    Vector2f closestEdge = edgeVec[0] - edgeVec[1];
    // get the normalized normal vector of the closest edge
    Vector2f normal = Vector2f(closestEdge[1], -closestEdge[0]).normalized();
    Vector2f ra = point - rb1->x, rb = point - rb2->x;
    Vector2f rv = (rb1->m_Velocity + rb1->omega * ra) - (rb2->m_Velocity + rb2->omega * rb);
    float vrel = normal.dot(rv), numerator = -(1 + epsilon) * vrel;
    float term1 = 1 / rb1->mass;
    float term2 = 1 / rb2->mass;
    MatrixXf raN(2, 2), rbN(2, 2), raF(2, 2), rbF(2, 2);
    raN(0, 0) = ra[0], raN(0, 1) = ra[1], raN(1, 0) = normal[0], raN(1, 1) = normal[1];
    rbN(0, 0) = rb[0], rbN(0, 1) = rb[1], rbN(1, 0) = normal[0], rbN(1, 1) = normal[1];
    float raCrossN = raN.determinant();
    float rbCrossN = rbN.determinant();
    float term3 = (raCrossN * raCrossN) * (1 / rb1->I);
    float term4 = (rbCrossN * rbCrossN) * (1 / rb2->I);
    float j = numerator / (term1 + term2 + term3 + term4);
    float scale = 3;
    Vector2f force = scale * j * normal;
    raF(0, 0) = ra[0], raF(0, 1) = ra[1], raF(1, 0) = force[0], raF(1, 1) = force[1];
    rbF(0, 0) = rb[0], rbF(0, 1) = rb[1], rbF(1, 0) = force[0], rbF(1, 1) = force[1];
    // cout << "force: " << force << endl;
    if (abs(force[0]) < 20)
        force[0] *= 45;
    else
        force[0] *= 15;
    if (abs(force[1]) < 20)
        force[1] *= 45;
    else
        force[1] *= 15;
    // cout <<"force "<< force << endl;
    rb1->m_Force += force;
    rb2->m_Force -= force;
    rb1->torque += (1 / rb1->I) / 8 * raF.determinant();
    rb2->torque -= (1 / rb2->I) / 8 * rbF.determinant();
}

map<int, map<int, float>> CollisionForce::dx()
{
    return map<int, map<int, float>>();
}

MatrixXf CollisionForce::dv()
{
    return MatrixXf();
}