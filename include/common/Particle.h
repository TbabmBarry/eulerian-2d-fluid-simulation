#pragma once

#include <gfx/vec2.h>
#include <vector>
#include "Eigen/Dense"

using namespace Eigen;
using namespace std;

class Particle
{
public:
	enum TYPE {
        RIGID,
        NORMAL
    };
    TYPE type;

	Particle(const Vec2f & ConstructPos, float mass, int index, TYPE type);
	~Particle(void);

	virtual void reset();
	virtual void draw();

	Vec2f m_ConstructPos;
	Vec2f m_Position;
	Vec2f m_Velocity;
	Vec2f m_Force;
	double mass;
	int index;
    bool rigid;



	//rigid body specific
	vector<Vec2f> corners;//local positions wrt masscenter, in order to deal with rotation
	void setBoundingBox();
	vector<Vec2f> getBoundingBox();//minX, minY, maxX, maxY
    // Vec2f getBodyCoordinates(Vec2f world);

    Vec2f MassCenter;
    float dimension;  //lengths of the edges

    //Constants
    double M; //totalMass
    float I;

    //State variables
    Vec2f x; //position x(t) global mass center
    float angle; //rotation R(t)
    Vec2f P; //linear momentum P(t)
    float L; //angular momentum L(t)

    //Derived quantities
    Vec2f v; //velocity v(t)
    float omega; //angular velocity omega(t)=L/I in 2D
    MatrixXf R; //rotation R(t)
    Vec2f force;
    float torque; //I*w'
};
