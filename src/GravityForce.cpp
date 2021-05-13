#include "SpringForce.h"
#include "GravityForce.h"
#include <GL/glut.h>
#include <gfx/vec2.h>
#include <math.h>

GravityForce::GravityForce(vector<Particle*> particles, Vec2f g) :
  g(g) {
    this->setTarget(particles);
  }

void GravityForce::setTarget(vector<Particle*> particles)
{
    this->particles = particles;
}

void GravityForce::apply(bool springsCanBreak)
{
    // Vec2f length = this->m_p1->m_Position - this->m_p2->m_Position; //l=particle p1-particle p2
    // Vec2f length_derivate = this->m_p1->m_Velocity - this->m_p2->m_Velocity; //l'=velocity p1-velocity p2
    // bool active = true;

    // force = mg
    for (Particle *p : particles)
      p->m_Force += p->mass * g;
}


void GravityForce::draw()
{
  glColor3f(0.6f, 0.7f, 0.8f);
  glBegin( GL_LINES );
  Vec2f normDirection = g / norm(g);
  for (Particle *p : particles)
  {
    glVertex2f(p->m_Position[0], p->m_Position[1]);
    glVertex2f(p->m_Position[0]+normDirection[0], p->m_Position[1]+normDirection[1]);
  }
  glEnd();
}
