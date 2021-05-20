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
{   // force = mg
  if (this->active)
  {
    for (Particle *p : particles){
      p->m_Force += p->mass * g;
    }
  }
  cout << "gravity "<<particles[0]->m_Force <<endl;
  cout << "velocity "<<particles[0]->m_Velocity <<endl;
  cout << "position "<<particles[0]->m_Position <<endl;
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
