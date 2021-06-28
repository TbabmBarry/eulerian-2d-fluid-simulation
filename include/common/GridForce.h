#pragma once
#include <stdio.h>
#include "Force.h"
#include "Particle.h"
#include "FluidSolver.h"
#include <map>
#include "Eigen/Dense"

using namespace Eigen;

class GridForce : public Force {

 public:
  GridForce(vector<Particle*> particles);

  void draw() override;
  void setTarget(vector<Particle*> particles) override;
  void applyFluidF(FluidSolver* fluid);
  void apply(bool springsCanBreak) override;
  map<int, map<int, float>> dx() override;
  MatrixXf dv() override;

 private:
};
