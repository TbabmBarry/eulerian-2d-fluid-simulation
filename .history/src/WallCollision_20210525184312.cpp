#include "SpringForce.h"
#include "WallCollision.h"
#include <GL/glut.h>
#include <gfx/vec2.h>
#include <math.h>

WallCollision::WallCollision(vector<Particle*> particles, Vec2f g) :
  g(g) {
    this->setTarget(particles);
  }

void WallCollision::setTarget(vector<Particle*> particles)
{
    this->particles = particles;
}

void WallCollision::apply(bool springsCanBreak)
{   // force = mg
  if (this->active)
  {
    for (Particle *p : particles){
        if(p->m_Position[0]<-0.9){
            printf("hello");
            p->m_Force += Vec2f(10,0);
        }
        if(p->m_Position[0]>0.9){
            printf("hi");
            p->m_Force += Vec2f(-10,0);
        }
        if(p->m_Position[1]<-0.9){
            // cout<<p->m_Position[1]<<endl;
            p->m_Position[1] = -0.5;
        }
        if(p->m_Position[1]>0.9){
            printf("hey");
            p->m_Force += Vec2f(0,-10);
        }
    }
  }
}

map<int, map<int, float>> WallCollision::dx()
{
    return map<int, map<int, float>>();
}

MatrixXf WallCollision::dv()
{
    return MatrixXf();
}

void WallCollision::draw()
{}
