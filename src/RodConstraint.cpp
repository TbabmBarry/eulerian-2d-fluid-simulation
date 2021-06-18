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
    Vector2f pVector = (m_p1->m_Position - m_p2->m_Position) * 2;
    Vector2f vVector = (m_p1->m_Velocity - m_p2->m_Velocity) * 2;
    return pVector.dot(vVector);
}

// Legal accelaeration is not needed to compute the constraint force

std::vector <Vector2f> RodConstraint::jacobian() {
    std::vector <Vector2f> j;
    j.push_back((m_p1->m_Position - m_p2->m_Position) * 2);//for partcle 1
    j.push_back(-(m_p1->m_Position - m_p2->m_Position) * 2);//for partcle 2
    return j;
}

std::vector<Vector2f> RodConstraint::jacobianDerivative() {
    std::vector <Vector2f> jd;
    jd.push_back((m_p1->m_Velocity-m_p2->m_Velocity) * 2);
    jd.push_back(-(m_p1->m_Velocity-m_p2->m_Velocity) * 2);
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
