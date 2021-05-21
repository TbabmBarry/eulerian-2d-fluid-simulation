#pragma once
#include <vector>
#include <stdio.h>
#include "Eigen/Dense"
#include "Particle.h"

using namespace Eigen;
using namespace std;

class Force {
  public:
    vector<Particle*> particles;
    virtual void setTarget(vector<Particle*> particles) = 0;
    virtual void apply(bool springsCanBreak) = 0;
    virtual MatrixXf dx() = 0;
    virtual MatrixXf dv() = 0;
    virtual void draw() = 0;
    bool active = true;
    void toggle() {active = !active;}
};
