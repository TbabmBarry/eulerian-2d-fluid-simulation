#include "RodConstraint.h"
#include <GL/glut.h>
#include <math.h>

RodConstraint::RodConstraint(Particle *p1, Particle * p2, double dist) : //r=dist
  m_p1(p1), m_p2(p2), m_dist(dist) {}

float RodConstraint::C() {
  float dx = this->m_p1->m_Position[0] - this->m_p2->m_Position[0];
  float dy = this->m_p1->m_Position[1] - this->m_p2->m_Position[1];
  return pow(dx,2) + pow(dy,2) - pow(this->m_dist,2);
}

float RodConstraint::Cd() {
    Vec2f pDiff = (this->m_p1->m_Position - this->m_p2->m_Position) * 2;
    Vec2f vDiff = (this->m_p1->m_Velocity - this->m_p2->m_Velocity) * 2;
    return pDiff * vDiff;
}

// std::vector <Vec2f> RodConstraint::j() {
//     std::vector <Vec2f> j;
//     j.push_back((this->m_p1->m_Position - this->m_p2->m_Position) * 2);
//     j.push_back((this->m_p1->m_Position - this->m_p2->m_Position) * 2);
//     return j;
// }

// std::vector<Vec2f> RodConstraint::jd() {
//     std::vector <Vec2f> jd;
//     jd.push_back((this->m_p1->m_Velocity - this->m_p2->m_Velocity) * 2);
//     jd.push_back((this->m_p2->m_Velocity - this->m_p1->m_Velocity) * 2);
//     return jd;
// }

void RodConstraint::draw()
{
  glBegin( GL_LINES );
  glColor3f(0.8, 0.7, 0.6);
  glVertex2f( m_p1->m_Position[0], m_p1->m_Position[1] );
  glColor3f(0.8, 0.7, 0.6);
  glVertex2f( m_p2->m_Position[0], m_p2->m_Position[1] );
  glEnd();

}
