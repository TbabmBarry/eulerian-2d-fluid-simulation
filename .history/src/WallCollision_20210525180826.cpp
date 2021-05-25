#include "SpringForce.h"
#include "WallCollision.h"
#include <GL/glut.h>
#include <gfx/vec2.h>
#include <math.h>

WallCollision::WallCollision(vector<Particle*> particles, Vec2f g) :
  g(g) {
    this->setTarget(particles);
  }

void GravityForce::setTarget(vector<Particle*> particles)
{
    this->particles = particles;
}

void GravityForce::apply(bool springsCanBreak)
{   // force = mg
  if (this->active)
  {
    for (Particle *p : particles){
      p->m_Force += p->mass * g;
    }
  }
}

map<int, map<int, float>> GravityForce::dx()
{
    return map<int, map<int, float>>();
}

MatrixXf GravityForce::dv()
{
    return MatrixXf();
}

void GravityForce::draw()
{
  // glColor3f(0.6f, 0.7f, 0.8f);
  // glBegin( GL_LINES );
  // Vec2f normDirection = g / norm(g);
  // for (Particle *p : particles)
  // {
  //   glVertex2f(p->m_Position[0], p->m_Position[1]);
  //   glVertex2f(p->m_Position[0]+normDirection[0], p->m_Position[1]+normDirection[1]);
  // }
  // glEnd();
}
