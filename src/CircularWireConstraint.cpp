#include "CircularWireConstraint.h"
#include <GL/glut.h>
#include <math.h>

#define PI 3.1415926535897932384626433832795

static void draw_circle(const Vec2f & vect, float radius)
{
	glBegin(GL_LINE_LOOP);
	glColor3f(0.0,1.0,0.0); 
	for (int i=0; i<360; i=i+18)
	{
		float degInRad = i*PI/180;
		glVertex2f(vect[0]+cos(degInRad)*radius,vect[1]+sin(degInRad)*radius);
	}
	glEnd();
}

CircularWireConstraint::CircularWireConstraint(Particle *p, const Vec2f & center, const double radius) :
	m_p(p), m_center(center), m_radius(radius) {}

float CircularWireConstraint::C() {
    float dx = this->m_p->m_Position[0] - this->m_center[0];
	float dy = this->m_p->m_Position[1] - this->m_center[1];
    return pow(dx,2) + pow(dy,2) - pow(this->m_radius,2);
}

/**
 * Computes Cd of this constraint
 * @return x * xd
 */
float CircularWireConstraint::Cd() {
    Vec2f pVector = (this->m_p->m_Position - this->m_center);
    Vec2f vVector = this->m_p->m_Velocity;
    return 2 * pVector * vVector;
}

/**
 * Computes j of this constraint
 * @return
 */
// std::vector<Vec2f> CircularWireConstraint::j() {
//     std::vector<Vec2f> j;
//     j.push_back((this->m_p->m_Position - this->m_center) * 2);
//     return j;
// }

// /**
//  * Computes jd of this constraint
//  * @return
//  */
// std::vector<Vec2f> CircularWireConstraint::jd() {
//     std::vector<Vec2f> jd;
//     jd.push_back(this->m_p->m_Velocity * 2);
//     return jd;
// }

void CircularWireConstraint::draw()
{
	draw_circle(m_center, m_radius);
}
