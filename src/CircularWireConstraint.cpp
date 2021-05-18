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
		float degInRad = i * PI / 180.0f;
		glVertex2f(vect[0]+cos(degInRad)*radius,vect[1]+sin(degInRad)*radius);
	}
	glEnd();
}

CircularWireConstraint::CircularWireConstraint(Particle *p, const Vec2f & center, const float radius) :
	Constraint({p}), m_p(p), m_center(center), m_radius(radius) {}

float CircularWireConstraint::C() {
    float dx = m_p->m_Position[0] - m_center[0];
	float dy = m_p->m_Position[1] - m_center[1];
    return pow(dx,2) + pow(dy,2) - pow(m_radius,2);
}

/**
 * Computes Cd of this constraint
 * @return x * xd
 */
float CircularWireConstraint::legalVelocity() {//C'
    Vec2f pVector = m_p->m_Position - m_center;
    Vec2f vVector = m_p->m_Velocity;
    return 2 * pVector * vVector;
}

// float CircularWireConstraint::legal_accelerate() {//C''
//     Vec2f vVector = m_p->m_Velocity;
//     return 2 * vVector * vVector;
// }

// Vec2f CircularWireConstraint::ConstraintF(){
// 	return -(m_p->m_Force * m_p->m_Position/norm(m_p->m_Position))*m_p->m_Position/norm(m_p->m_Position);
// }

vector<Vec2f> CircularWireConstraint::jacobian() {
    vector<Vec2f> j;
	//J=(x-xc,y-yc)
    j.push_back((m_p->m_Position - m_center) * 2);
    return j;
}

vector<Vec2f> CircularWireConstraint::jacobianDerivative() {
    vector<Vec2f> jd;
    jd.push_back(m_p->m_Velocity * 2);
    return jd;
}

void CircularWireConstraint::draw()
{
	draw_circle(m_center, m_radius);

	glBegin(GL_LINES);
    glColor3f(0.7,0.7,0.0);
    glVertex2f(m_p->m_Position[0], m_p->m_Position[1]);
    glVertex2f(m_center[0], m_center[1]);
    glEnd();
}
