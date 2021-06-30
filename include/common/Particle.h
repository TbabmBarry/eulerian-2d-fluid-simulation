#pragma once

#include <gfx/vec2.h>
#include <vector>
#include "Eigen/Dense"

using namespace Eigen;
using namespace std;

class Particle
{
public:
    enum TYPE
    {
        RIGID,
        FIX,
        MOVING,
        NORMAL
    };
    TYPE type;

    Particle(const Vector2f &ConstructPos, float mass, int index, TYPE type);
    ~Particle(void);

    virtual void reset();
    virtual void draw();
    virtual void drawBound();
    virtual void drawInner();

    Vector2f m_ConstructPos;
    Vector2f m_Position;
    Vector2f m_Velocity;
    Vector2f m_Force;
    float mass;
    int index;
    int rigid;

    //rigid body specific
    vector<Vector2f> corners; //local positions wrt masscenter, in order to deal with rotation
    void setBoundingBox();
    vector<Vector2f> getBoundingBox(); //minX, minY, maxX, maxY
    // Vec2f getBodyCoordinates(Vec2f world);
    vector<Vector4f> BoundingGrid(int grid_N);
    vector<Vector2i> InnerGrid(vector<Vector4f> BoundGrid);
    static bool compareVectors(Vector2i g1, Vector2i g2);

    // get the closest edge for a target point
    vector<Vector2f> getClosestEdge(Vector2f point);
    float minDistance(Vector2f p1, Vector2f p2, Vector2f p3);
    Vector2f MassCenter;
    float dimension; //lengths of the edges

    //Constants
    double M; //totalMass
    float I;  //inertia

    //State variables
    Vector2f x;  //position x(t) global mass center
    float angle; //rotation R(t)
    Vector2f P;  //linear momentum P(t)
    float L;     //angular momentum L(t)

    //Derived quantities
    float omega; //angular velocity omega(t)=L/I in 2D
    MatrixXf R;  //rotation R(t)
    Vector2f force;
    float torque; //I*w'
};
