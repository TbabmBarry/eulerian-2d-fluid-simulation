#include "CircularWireConstraint.h"
#include <GL/glut.h>
#include <math.h>

#define PI 3.1415926535897932384626433832795

static void draw_circle(const Vector2f & vect, float radius)
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

CircularWireConstraint::CircularWireConstraint(Particle *p, const Vector2f & center, const float radius) :
	Constraint({p}), m_p(p), m_center(center), m_radius(radius) {}

float CircularWireConstraint::C() {
    Vector2f pVector = m_p->m_Position - m_center;
    return pVector.dot(pVector) - m_radius * m_radius;
}

/**
 * Computes Cd of this constraint
 * @return x * xd
 */
float CircularWireConstraint::legalVelocity() {//C'
    Vector2f pVector = m_p->m_Position - m_center;
    Vector2f vVector = m_p->m_Velocity;
    return 2 * pVector.dot(vVector);
}

vector<Vector2f> CircularWireConstraint::jacobian() {
    vector<Vector2f> j;
	//J=(x-xc,y-yc)
    j.push_back((m_p->m_Position - m_center) * 2);
    return j;
}

vector<Vector2f> CircularWireConstraint::jacobianDerivative() {
    vector<Vector2f> jd;
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
