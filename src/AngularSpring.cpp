#include "AngularSpring.h"
#include <GL/glut.h>
#include <gfx/vec2.h>
#include <math.h>
#include <assert.h>

AngularSpring::AngularSpring(Particle *p1, Particle * midpoint,Particle * p3, double angle, double ks, double kd) :
  AngularSpring({p1, midpoint, p3}, angle, m_ks, m_kd) {}

AngularSpring::AngularSpring(vector<Particle*> particles, float angle, float m_ks, float m_kd) : m_angle(angle), m_ks(m_ks), m_kd(m_kd)
{
    this->setTarget(particles);
}

void AngularSpring::setTarget(vector<Particle*> particles)
{
    assert(particles.size() == 3);
    this->particles = particles;
}

void AngularSpring::apply(bool springsCanBreak)
{
  if (this->active)
  {
    Vec2f midtoP1 = particles[0]->m_Position - particles[1]->m_Position; //l1=particle p1-particle midpoint
    Vec2f midtoP3 = particles[2]->m_Position - particles[1]->m_Position; //l2=particle p1-particle midpoint

    float current_angle = acos((midtoP1 * midtoP3)/(norm(midtoP1) * norm(midtoP3)));
    float half_current_angle = current_angle / 2;
    float half_rest_angle = m_angle / 2;

    float p1restlength=norm(midtoP1) * sin(half_current_angle) / sin(half_rest_angle);
    float p3restlength=norm(midtoP3) * sin(half_current_angle) / sin(half_rest_angle);

    Vec2f length1_derivate = particles[0]->m_Velocity - particles[1]->m_Velocity; //l'=velocity p1-velocity midpoint
    Vec2f length3_derivate = particles[2]->m_Velocity - particles[1]->m_Velocity; //l'=velocity p3-velocity midpoint

    // force = [ ks * ( |l| - r ) + kd * l' * l /|l| ] * l / |l|
    //rest length r is calculated by rest angle here
    Vec2f force1 = (m_ks*(norm(midtoP1)-p1restlength)+m_kd*((midtoP1*length1_derivate)/norm(midtoP1)))*(midtoP1/norm(midtoP1));
    Vec2f force3 = (m_ks*(norm(midtoP3)-p3restlength)+m_kd*((midtoP3*length3_derivate)/norm(midtoP3)))*(midtoP3/norm(midtoP3));
    particles[0]->m_Force += force1;
    particles[1]->m_Force += -force1;
    particles[2]->m_Force += force3;
    particles[1]->m_Force += -force3;
  }
}

MatrixXf AngularSpring::dx()
{
    return MatrixXf();
}

MatrixXf AngularSpring::dv()
{
    return m_ks * MatrixXf::Identity(2, 2);
}

void AngularSpring::draw()
{
  glBegin( GL_LINES );
  glColor3f(0.6, 0.7, 0.8);
  glVertex2f( particles[0]->m_Position[0], particles[0]->m_Position[1] );
  glColor3f(0.6, 0.7, 0.8);
  glVertex2f( particles[1]->m_Position[0], particles[1]->m_Position[1] );
  glColor3f(0.6, 0.7, 0.8);
  glVertex2f( particles[2]->m_Position[0], particles[2]->m_Position[1] );
  glEnd();
}
