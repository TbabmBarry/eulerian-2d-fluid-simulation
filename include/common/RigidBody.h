#include "Eigen/Dense"
#include "Particle.h"
#include "Force.h"

// using namespace Eigen;

class RigidBody : public Particle{
public:
    RigidBody(const Vec2f & MassCenter, float mass, int index, bool rigid);

    void reset();
    void draw(bool drawVelocity, bool drawForce);

    VectorXf getBoundingBox();//minX, minY, maxX, maxY
    // Vec2f getBodyCoordinates(Vec2f world);

    VectorXf getState();

    VectorXf getDerivativeState();

    void setState(VectorXf newState);

    std::vector<Particle *> particles;
    Vec2f MassCenter;
    Vec2f dimension;  //lengths of the edges

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
    Matrix2f R; //rotation R(t)
    Vec2f force;
    float torque; //I*w'

private:
    void updateForce();
    void updateTorque();
    void initializeVariables();
};