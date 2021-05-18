#include "SpringForce.h"
#include <GL/glut.h>
#include <gfx/vec2.h>
#include <math.h>
#include <assert.h>

SpringForce::SpringForce(Particle *p1, Particle * p2, float m_dist, float m_ks, float m_kd) :
// "class : xxxxx" means pass xxxxx as parameters to define a new class
  //p1,p2 are particles&postions of them
  //m_p1(p1) means p1 is defined as private var. m_p1; p2 is m_p2; dist is m_dist ...
  //cuz SpringForce::SpringForce, :: used. If only SpringForce, not need m_p1(p1)
  SpringForce({p1, p2}, m_dist, m_ks, m_kd) {}

SpringForce::SpringForce(vector<Particle*> particles, float m_dist, float m_ks, float m_kd) : m_dist(m_dist), m_ks(m_ks), m_kd(m_kd)
{
    this->setTarget(particles);
}

void SpringForce::setTarget(vector<Particle*> particles)
{
    assert(particles.size() == 2);
    this->particles = particles;
}

void SpringForce::apply(bool springsCanBreak)
{
    // printf("ciao");
    Vec2f length = particles[0]->m_Position - particles[1]->m_Position; //l=particle p1-particle p2
    Vec2f length_derivate = particles[0]->m_Velocity - particles[1]->m_Velocity; //l'=velocity p1-velocity p2
    bool active = true;

    if(springsCanBreak && norm(length)>2*m_dist){//think of break(or not) length
        active=false;
    } else if(active){
        // force1 = [ ks * ( |l| - r ) + kd * l' * l /|l| ] * l / |l|
        // std::cout<<norm(length)<<std::endl;
        Vec2f force = (m_ks*(norm(length)-m_dist)+m_kd*((length*length_derivate)/norm(length)))*(length/norm(length));
        particles[0]->m_Force += force;
        particles[1]->m_Force += -force;
    }
}

void SpringForce::draw()
{
  glBegin( GL_LINES );
  glColor3f(0.6, 0.7, 0.8);
  glVertex2f( particles[0]->m_Position[0], particles[0]->m_Position[1] );
  glColor3f(0.6, 0.7, 0.8);
  glVertex2f( particles[1]->m_Position[0], particles[1]->m_Position[1] );
  glEnd();
}
