#include "SpringForce.h"
#include "Gravity.h"
#include <GL/glut.h>
#include <gfx/vec2.h>
#include <math.h>

Gravity::Gravity(Particle *p, double mass) :
  m_p(p) {
      m_p->mass=mass;
  }

void Gravity::apply()
{
    // Vec2f length = this->m_p1->m_Position - this->m_p2->m_Position; //l=particle p1-particle p2
    // Vec2f length_derivate = this->m_p1->m_Velocity - this->m_p2->m_Velocity; //l'=velocity p1-velocity p2
    // bool active = true;

    // force = mg
    Vec2f force = m_p->mass * g;
    m_p->m_Force = force;
}

Vec2f Gravity::Jacobian() {
    return g;
}


// MatrixXf SpringForce::jv() {
//     return ks * MatrixXf::Identity(3, 3);
// }

void Gravity::draw()
{
  glBegin( GL_POINTS );
  glColor3f(0.6, 0.7, 0.8);
  glVertex2f( m_p->m_Position[0], m_p->m_Position[1] );
  glEnd();
}
