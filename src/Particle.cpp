#include "Particle.h"
#include <GL/glut.h>
#include <iostream>
#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

Particle::Particle(const Vector2f & ConstructPos, float mass, int index, TYPE type) :
	m_ConstructPos(ConstructPos), m_Position(Vector2f(0.0, 0.0)), m_Velocity(Vector2f(0.0, 0.0)), mass(mass), index(index), 
	type(NORMAL), MassCenter(ConstructPos), dimension(5)
{
	switch (type)
    {
    case NORMAL:
		rigid=0;
		break;
    case RIGID:
		{rigid=1;
		setBoundingBox();
        break;}
    }
}

Particle::~Particle(void)
{
}

void Particle::reset()
{
	m_Position = m_ConstructPos;
	m_Velocity = Vector2f(0.0, 0.0);
	m_Force = Vector2f(0.0, 0.0);

	//rigid specific
	I = 0.0;
	x = MassCenter;
    R = Matrix2f::Identity();
    P = Vector2f(0, 0);//M*v(t)
    L = 0.0;//I*w(t)
    v = Vector2f(0, 0);
    omega = L/I;
    torque = 0.0;
    angle = 10 * 180 / M_PI;
}

void Particle::draw()//draw a square
{
	switch (type)
    {
    case NORMAL:
		{const float h = 6.f;
		// std::cout<<m_Position<<std::endl;
		glColor3f(1.f, 1.f, 1.f); //rgb
		glPointSize(h);
		glBegin(GL_POINTS);
		glVertex2f(m_Position[0], m_Position[1]);
		glEnd();
        break;}
    case RIGID:
        break;
    }
	
}

void Particle::setBoundingBox(){
	//local positions wrt masscenter, in order to deal with rotation
	//to be changed
	corners.push_back(Vector2f(dimension/2 , dimension/2));//local topright
	corners.push_back(Vector2f(-dimension/2 , dimension/2));//local topleft
	corners.push_back(Vector2f(dimension/2 , -dimension/2));//local bottomright
	corners.push_back(Vector2f(-dimension/2 , -dimension/2));//local bottomleft
}

vector<Vector2f> Particle::getBoundingBox(){
	switch (type)
    {
    case NORMAL:
		break;
    case RIGID:
		return corners;
        break;
    }
}