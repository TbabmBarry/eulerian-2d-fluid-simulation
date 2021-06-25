#include "Particle.h"
#include <GL/glut.h>
#include <iostream>
#include <limits>
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
		reset();
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
    L = 0.0f;//I*w(t)
    v = Vector2f(0, 0);
    omega = L/(I+0.00000000001);
    torque = 0.0f;
    angle = 0.0f * M_PI / 180;
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
		glClear(GL_COLOR_BUFFER_BIT);
		glColor3f(0.0,1.0,0.0);
		glBegin(GL_POLYGON);
		glVertex2f(corners[0][0],corners[0][1]);
		glVertex2f(corners[1][0],corners[1][1]);
		glVertex2f(corners[2][0],corners[2][1]);
		glVertex2f(corners[3][0],corners[3][1]);
		glEnd();
        break;
    }
	
}

void Particle::setBoundingBox(){
	// local positions wrt masscenter, in order to deal with rotation
	// to be changed
	corners.push_back(Vector2f(dimension/2 , dimension/2));//local topright
	corners.push_back(Vector2f(-dimension/2 , dimension/2));//local topleft
	corners.push_back(Vector2f(dimension/2 , -dimension/2));//local bottomright
	corners.push_back(Vector2f(-dimension/2 , -dimension/2));//local bottomleft
	//corners rotated pos = corner pos*R + masscenter pos
	for (int k=0; k < corners.size();++k) {
		corners[k] = R * corners[k] + x;
	}
}

// Do we confine corners to be local positions or global
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

vector<Vector2f> Particle::getClosestEdge(Vector2f point)
{
	int idx = 0, n = corners.size();
	float minDist = numeric_limits<float>::max(), currDist;
	for (int i = 0; i < n; i++)
	{
		currDist = minDistance(corners[i], corners[(i+1)%n], point);
		if (minDist >= currDist)
		{
			idx = i;
			minDist = currDist;
		}
	}
	return vector<Vector2f>({corners[idx],corners[(idx+1)%n]});
}

float Particle::minDistance(Vector2f p1, Vector2f p2, Vector2f p3)
{
	Vector2f e12 = p2 - p1, e23 = p3 - p2, e13 = p3 - p1;
	float d23 = e12.dot(e23), d13 = e12.dot(e13), minDist;
	if (d23 > 0)
	{
		minDist = e23.norm();
	}
	else if (d13 < 0)
	{
		minDist = e13.norm();
	}
	else
	{
		float mod = e12.norm();
		minDist = abs(e12[0] * e13[1] - e12[1] * e13[0]) / mod;
	}
	return minDist;
}