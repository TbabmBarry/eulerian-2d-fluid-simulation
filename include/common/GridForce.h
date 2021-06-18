#pragma once
#include <stdio.h>
#include "Force.h"
#include "Particle.h"
#include <map>

class GridForce : public Force {

 public:
  GridForce(vector<Particle*> particles);

  void draw() override;
  void setTarget(vector<Particle*> particles) override;
//   void apply(bool springsCanBreak) override;

 private:
};
