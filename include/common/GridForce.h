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
  void apply(FluidSolver fluid);

 private:
};
