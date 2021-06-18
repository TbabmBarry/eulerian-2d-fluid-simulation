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

	Particle(const Vector2f & ConstructPos, float mass, int index, TYPE type);
	~Particle(void);

	virtual void reset();
	virtual void draw();

	Vector2f m_ConstructPos;
	Vector2f m_Position;
	Vector2f m_Velocity;
	Vector2f m_Force;
	double mass;
	int index;
    bool rigid;



	//rigid body specific
	vector<Vector2f> corners;//local positions wrt masscenter, in order to deal with rotation
	void setBoundingBox();
	vector<Vector2f> getBoundingBox();//minX, minY, maxX, maxY
    // Vec2f getBodyCoordinates(Vec2f world);

    Vector2f MassCenter;
    float dimension;  //lengths of the edges

    //Constants
    double M; //totalMass
    float I;

    //State variables
    Vector2f x; //position x(t) global mass center
    float angle; //rotation R(t)
    Vector2f P; //linear momentum P(t)
    float L; //angular momentum L(t)

    //Derived quantities
    Vector2f v; //velocity v(t)
    float omega; //angular velocity omega(t)=L/I in 2D
    MatrixXf R; //rotation R(t)
    Vector2f force;
    float torque; //I*w'
};
