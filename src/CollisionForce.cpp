#include "CollisionForce.h"
#include <GL/glut.h>
#include <assert.h>

CollisionForce::CollisionForce(Particle *rb1, Particle *rb2, float threshold, float epsilon) :
  CollisionForce({rb1, rb2}, threshold, epsilon) {}

CollisionForce::CollisionForce(vector<Particle*> rigidbodies, float threshold, float epsilon) : threshold(0.0f), epsilon(0.2f)
{
    this->setTarget(rigidbodies);
}

void CollisionForce::setTarget(vector<Particle*> rigidbodies)
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
        }
    }
    vector<Vector2f> rb2Corners = particles[1]->getBoundingBox();
    for (const auto &corner : rb2Corners)
    {
        if (colliding(corner, particles[1], particles[0]))
        {
            collision(corner, particles[1], particles[0]);
        }
    }
}

void CollisionForce::draw()
{

}

bool CollisionForce::colliding(Vector2f point, Particle* rb1, Particle* rb2)
{
    // get the vector of points constructing the closest edge
    vector<Vector2f> edgeVec = rb2->getClosestEdge(point);
    // check if the distance from point to line close enough
    float minDist = rb1->minDistance(edgeVec[0], edgeVec[1], point);
    if (minDist > 0.05)
        return false;
    // get the edge vector
    Vector2f closestEdge = edgeVec[0] - edgeVec[1];
    // get the normal vector of the closest edge
    Vector2f normal = Vector2f(-closestEdge[1], closestEdge[0]).normalized();
    Vector2f ra = point - rb1->x, rb = point - rb2->x;
    Vector2f rv = (rb1->m_Velocity + rb1->omega * ra) - (rb2->m_Velocity + rb2->omega * rb);
    float vrel = normal.dot(rv);
    if (vrel > threshold)
        return false;
    else
        return true;
}

void CollisionForce::collision(Vector2f point, Particle* rb1, Particle* rb2)
{
    // get the vector of points constructing the closest edge
    vector<Vector2f> edgeVec = rb2->getClosestEdge(point);
    // get the edge vector
    Vector2f closestEdge = edgeVec[0] - edgeVec[1];
    // get the normalized normal vector of the closest edge
    Vector2f normal = Vector2f(-closestEdge[1], closestEdge[0]).normalized();
    Vector2f ra = point - rb1->x, rb = point - rb2->x;
    Vector2f rv = (rb1->m_Velocity + rb1->omega * ra) - (rb2->m_Velocity + rb2->omega * rb);
    float vrel = normal.dot(rv), numerator = -(1 + epsilon) * vrel;
    float term1 = 1 / rb1->mass;
    float term2 = 1 / rb2->mass;
    MatrixXf raN(2,2), rbN(2,2),raF(2,2), rbF(2,2);
    raN(0,0) = ra[0],raN(0,1) = ra[1],raN(1,0) = normal[0],raN(1,1) = normal[1];
    rbN(0,0) = rb[0],rbN(0,1) = rb[1],rbN(1,0) = normal[0],rbN(1,1) = normal[1];
    float raCrossN = raN.determinant();
    float rbCrossN = rbN.determinant();
    float term3 = (raCrossN*raCrossN)*(1/(rb1->I+0.00000000001));
    float term4 = (rbCrossN*rbCrossN)*(1/(rb2->I+0.00000000001));
    float j = numerator / (term1 + term2 + term3 + term4);
    Vector2f force = j * normal;
    rb1->m_Force += force;
    rb2->m_Force -= force;
    raF(0,0) = ra[0],raF(0,1) = ra[1],raF(1,0) = force[0],raF(1,1) = force[1];
    rbF(0,0) = rb[0],rbF(0,1) = rb[1],rbF(1,0) = force[0],rbF(1,1) = force[1];
    rb1->torque += (1/(rb1->I+0.00000000001))/2 * raF.determinant();
    rb2->torque -= (1/(rb2->I+0.00000000001))/2 * rbF.determinant();
}

map<int, map<int, float>> CollisionForce::dx()
{
    return map<int, map<int, float>>();
}

MatrixXf CollisionForce::dv()
{
    return MatrixXf();
}