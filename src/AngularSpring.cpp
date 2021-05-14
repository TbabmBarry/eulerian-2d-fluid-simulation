#include "AngularSpring.h"
#include <GL/glut.h>
#include <gfx/vec2.h>
#include <math.h>
#include <assert.h>

AngularSpring::AngularSpring(Particle *p1, Particle * p2,Particle * p3, double angle, double ks, double kd) :
  AngularSpring({p1, p2, p3}, angle, m_ks, m_kd) {}

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
    Vec2f length = particles[0]->m_Position - particles[1]->m_Position; //l=particle p1-particle p2
    Vec2f length_derivate = particles[0]->m_Velocity - particles[1]->m_Velocity; //l'=velocity p1-velocity p2
    bool active = true;

    if(springsCanBreak && norm(length)>2*m_dist){//think of break(or not) length
        active=false;
    } else if(active){
        // force1 = [ ks * ( |l| - r ) + kd * l' * l /|l| ] * l / |l|
        Vec2f force = (m_ks*(norm(length)-m_dist)+m_kd*((length*length_derivate)/norm(length)))*(length/norm(length));
        particles[0]->m_Force += force;
        particles[1]->m_Force += -force;
    }
}

void AngularSpring::draw()
{
  glBegin( GL_LINES );
  glColor3f(0.6, 0.7, 0.8);
  glVertex2f( particles[0]->m_Position[0], particles[0]->m_Position[1] );
  glColor3f(0.6, 0.7, 0.8);
  glVertex2f( particles[1]->m_Position[0], particles[1]->m_Position[1] );
  glEnd();
}