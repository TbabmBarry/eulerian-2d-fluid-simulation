#pragma once
#include "Eigen/Dense"
#include "Fluid.h"
#include "Force.h"
#include "Particle.h"
#include <map>
#include <stdio.h>

using namespace Eigen;
class GridForce : public Force
{

public:
  GridForce(vector<Particle *> particles, Fluid *fluid, int scale);
  Fluid *fluid;
  void draw() override;
  void setTarget(vector<Particle *> particles) override;
  void apply(bool springsCanBreak) override;
  map<int, map<int, float>> dx() override;
  MatrixXf dv() override;

private:
  int scale;
};
