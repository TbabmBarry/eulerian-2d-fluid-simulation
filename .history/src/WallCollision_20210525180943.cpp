#include "SpringForce.h"
#include "WallCollision.h"
#include <GL/glut.h>
#include <gfx/vec2.h>
#include <math.h>

WallCollision::WallCollision(vector<Particle*> particles, Vec2f g) :
  g(g) {
    this->setTarget(particles);
  }

void WallCollision::setTarget(vector<Particle*> particles)
{
    this->particles = particles;
}

void WallCollision::apply(bool springsCanBreak)
{   // force = mg
  if (this->active)
  {
    for (Particle *p : particles){
      p->m_Force += p->mass * g;
    }
  }
}

map<int, map<int, float>> WallCollision::dx()
{
    return map<int, map<int, float>>();
}

MatrixXf WallCollision::dv()
{
    return MatrixXf();
}

void WallCollision::draw()
{}
