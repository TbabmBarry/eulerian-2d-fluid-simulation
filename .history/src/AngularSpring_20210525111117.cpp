#include "AngularSpring.h"
#include <GL/glut.h>
#include <gfx/vec2.h>
#include <math.h>
#include "unistd.h"
#include <assert.h>

#define PI 3.14159265

AngularSpring::AngularSpring(Particle *p1, Particle * midpoint,Particle * p3, float m_angle, float m_ks, float m_kd) :
  AngularSpring({p1, midpoint, p3}, m_angle, m_ks, m_kd) {}

AngularSpring::AngularSpring(vector<Particle*> particles, float m_angle, float m_ks, float m_kd) : m_angle(m_angle), m_ks(m_ks), m_kd(m_kd)
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
    Vec2f p1toMid = particles[0]->m_Position - particles[1]->m_Position; //l1=particle p1-particle midpoint
    Vec2f midtoP3 = particles[1]->m_Position - particles[2]->m_Position; //l2=particle p1-particle midpoint
    float cos_angle = (p1toMid * midtoP3)/(norm(p1toMid) * norm(midtoP3));
    if (cos_angle > 1.0) cos_angle = 1.0;
    if (cos_angle < -1.0) cos_angle = -1.0;
    float current_angle = acos(cos_angle) * 180.0 / PI;
    // cout << "current angle: " << cos_angle << endl;
    double angle = acos(current_angle);
    Vec2f length = particles[0]->m_Position - particles[2]->m_Position;
    Vec2f velocity = particles[0]->m_Velocity - particles[2]->m_Velocity;

    // Compute spring force
    double b = norm(p1toMid);
    double c = norm(midtoP3);
    Vec2f result = -(m_ks * (norm(length) - sqrt(b * b + c * c + 2 * b * c * cos(m_angle))) + m_kd * ((length * velocity) / norm(length))) *
                   (length / norm(length));

    particles[0]->m_Force += result;
    particles[2]->m_Force -= result;
  }
}

map<int, map<int, float>> AngularSpring::dx()
{
    return map<int, map<int, float>>();
}

MatrixXf AngularSpring::dv()
{
    return MatrixXf();
}

void AngularSpring::draw()
{
  // glBegin( GL_LINES );
  // glColor3f(0.6, 0.7, 0.8);
  // glVertex2f( particles[0]->m_Position[0], particles[0]->m_Position[1] );
  // glColor3f(0.6, 0.7, 0.8);
  // glVertex2f( particles[1]->m_Position[0], particles[1]->m_Position[1] );
  // glEnd();
  // glBegin( GL_LINES );
  // glColor3f(0.6, 0.7, 0.8);
  // glVertex2f( particles[1]->m_Position[0], particles[1]->m_Position[1] );
  // glColor3f(0.6, 0.7, 0.8);
  // glVertex2f( particles[2]->m_Position[0], particles[2]->m_Position[1] );
  // glEnd();
}
