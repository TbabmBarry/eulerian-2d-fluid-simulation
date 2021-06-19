#include "CollisionForce.h"
#include <GL/glut.h>

CollisionForce::CollisionForce(vector<Particle*> rigidbodies, float threshold) : threshold(threshold)
{
    this->setTarget(rigidbodies);
}

void CollisionForce::setTarget(vector<Particle*> rigidbodies)
{
    this->particles = rigidbodies;
}

void CollisionForce::apply(bool springsCanBreak)
{
}

void CollisionForce::draw()
{

}