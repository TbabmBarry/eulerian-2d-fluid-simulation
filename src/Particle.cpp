#include "Particle.h"
#include <GL/glut.h>

Particle::Particle(const Vec2f & ConstructPos, float mass, int index) :
	m_ConstructPos(ConstructPos), m_Position(Vec2f(0.0, 0.0)), m_Velocity(Vec2f(0.0, 0.0)), mass(mass), index(index)
{
}

Particle::~Particle(void)
{
}

void Particle::reset()
{
	m_Position = m_ConstructPos;
	m_Velocity = Vec2f(0.0, 0.0);
	m_Force = Vec2f(0.0, 0.0);
}
void Particle::draw()//draw a square
{
	const float h = 3.f;
	// std::cout<<m_Position<<std::endl;
	glColor3f(1.f, 1.f, 1.f); //rgb
	glPointSize(h);
	glBegin(GL_POINTS);
	glVertex2f(m_Position[0], m_Position[1]);
	glEnd();

	glColor3f(0.0, 0.6, 0.0);
	glBegin(GL_LINES);
	glVertex2f(m_Position[0], m_Position[1]);
	glVertex2f(m_Position[0] + m_Velocity[0] * 0.2f, m_Position[1] + m_Velocity[1] * 0.2f);
	glEnd();

	glColor3f(0.0, 0.6, 0.6);
	glBegin(GL_LINES);
	glVertex2f(m_Position[0], m_Position[1]);
	glVertex2f(m_Position[0] + m_Force[0] * 0.2f, m_Position[1] + m_Force[1] * 0.2f);
	glEnd();
}
