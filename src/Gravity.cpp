#include "SpringForce.h"
#include "Gravity.h"
#include <GL/glut.h>
#include <gfx/vec2.h>
#include <math.h>

Gravity::Gravity(Particle *p, double mass) :
  m_p(p) {}

void Gravity::apply()
{
    Vec2f length = this->m_p1->m_Position - this->m_p2->m_Position; //l=particle p1-particle p2
    Vec2f length_derivate = this->m_p1->m_Velocity - this->m_p2->m_Velocity; //l'=velocity p1-velocity p2
    bool active = true;

    // force = mg
    Vec2f force = (this->m_ks*(norm(length)-this->m_dist)+this->m_kd*((length*length_derivate)/norm(length)))*(length/norm(length));
    this->m_p1->m_Force = force;
    this->m_p2->m_Force = -force;
}

Mat2**  SpringForce::Jacobian() {
    Mat2** Jacobian=(Mat2**)malloc(sizeof(Mat2*)*2);
    for(int i=0;i<2;++i){
      Jacobian[i]=(Mat2*)malloc(sizeof(Mat2)*2);
    }

    Mat2 I = Mat2::I();
    Vec2f length = this->m_p1->m_Position - this->m_p2->m_Position;
    Vec2f C=norm(length)-this->m_dist;
    float length_abs = norm(length);

    Mat2 force = this->m_ks*I \
                  + (this->m_ks*this->m_dist + this->m_kd*norm(this->m_p1->m_Velocity-this->m_p2->m_Velocity))/length_abs * I \ 
                  + (this->m_ks*this->m_dist + this->m_kd*norm(this->m_p1->m_Velocity-this->m_p2->m_Velocity))/pow(length_abs,3) * I;

    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            if (i!=j){
              Jacobian[i][j] = force;
            }
            else{
              Jacobian[i][j] = -force;
            }
        }
    }
    return Jacobian;
}


// MatrixXf SpringForce::jv() {
//     return ks * MatrixXf::Identity(3, 3);
// }

void SpringForce::draw()
{
  glBegin( GL_LINES );
  glColor3f(0.6, 0.7, 0.8);
  glVertex2f( m_p1->m_Position[0], m_p1->m_Position[1] );
  glColor3f(0.6, 0.7, 0.8);
  glVertex2f( m_p2->m_Position[0], m_p2->m_Position[1] );
  glEnd();
}
