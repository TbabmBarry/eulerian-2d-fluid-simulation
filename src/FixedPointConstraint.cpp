#include "FixedPointConstraint.h"
#include <GL/glut.h>
#include <math.h>

#define PI 3.1415926535897932384626433832795

static void draw_circle(const Vec2f & vect, float radius)
{
	glBegin(GL_LINE_LOOP);
	glColor3f(0.0,1.0,0.0); 
	for (int i = 0; i < 360; i = i + 18)
	{
		float degInRad = i * PI / 180.0f;
		glVertex2f(vect[0] + cos(degInRad) * radius, vect[1] + sin(degInRad) * radius);
	}
	glEnd();
}

FixedPointConstraint::FixedPointConstraint(Particle *p, const Vec2f & center) :
	Constraint({p}), m_p(p), m_center(center) {}

float FixedPointConstraint::C() {
    Vec2f pVector = m_p->m_Position - m_center;
    return pVector * pVector / 2;
}

/**
 * Computes Cd of this constraint
 * @return x * xd
 */
float FixedPointConstraint::legalVelocity() {//C'
    Vec2f pVector = m_p->m_Position - m_center;
    Vec2f vVector = m_p->m_Velocity;
    return pVector * vVector;
}

vector<Vec2f> FixedPointConstraint::jacobian() {
    vector<Vec2f> j;
	//J=(x-xc,y-yc)
    j.push_back(m_p->m_Position - m_center);
    return j;
}

vector<Vec2f> FixedPointConstraint::jacobianDerivative() {
    vector<Vec2f> jd;
    jd.push_back(m_p->m_Velocity);
    return jd;
}

void FixedPointConstraint::draw()
{
	draw_circle(m_center, 0.001);

	glBegin(GL_LINES);
    glColor3f(0.7,0.7,0.0);
    glVertex2f(m_p->m_Position[0], m_p->m_Position[1]);
    glVertex2f(m_center[0], m_center[1]);
    glEnd();
}
