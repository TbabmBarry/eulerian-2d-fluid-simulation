#include "RodConstraint.h"
#include <GL/glut.h>
#include <math.h>

RodConstraint::RodConstraint(Particle *p1, Particle * p2, float dist) : //r=dist
  Constraint({p1, p2}), m_p1(p1), m_p2(p2), m_dist(dist) {}

float RodConstraint::C() {
  float dx = m_p1->m_Position[0] - m_p2->m_Position[0];
  float dy = m_p1->m_Position[1] - m_p2->m_Position[1];
  return pow(dx,2) + pow(dy,2) - pow(m_dist,2);
}

float RodConstraint::legalVelocity() {
    Vec2f pVector = (m_p1->m_Position - m_p2->m_Position) * 2;
    Vec2f vVector = (m_p1->m_Velocity - m_p2->m_Velocity) * 2;
    return pVector * vVector;
}

// Legal accelaeration is not needed to compute the constraint force
// float RodConstraint::legal_accelerate() {
//     Vec2f vVector = (m_p1->m_Velocity - m_p2->m_Velocity);
//     return 2 * vVector * vVector;
// }

std::vector <Vec2f> RodConstraint::jacobian() {
    std::vector <Vec2f> j;
    j.push_back((m_p1->m_Position - m_p2->m_Position) * 2);//for partcle 1
    j.push_back((m_p1->m_Position - m_p2->m_Position) * 2);//for partcle 2
    //but why push twice??? cuz 2 particles?
    return j;
}

std::vector<Vec2f> RodConstraint::jacobianDerivative() {
    std::vector <Vec2f> jd;
    jd.push_back(m_p1->m_Velocity * 2);
    jd.push_back(-m_p2->m_Velocity * 2);
    return jd;
}

void RodConstraint::draw()
{
  glBegin( GL_LINES );
  glColor3f(0.8, 0.7, 0.6);
  glVertex2f( m_p1->m_Position[0], m_p1->m_Position[1] );
  glColor3f(0.8, 0.7, 0.6);
  glVertex2f( m_p2->m_Position[0], m_p2->m_Position[1] );
  glEnd();

}
