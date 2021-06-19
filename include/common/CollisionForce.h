#pragma once
#include <stdio.h>
#include "Force.h"
#include "Particle.h"

class CollisionForce : public Force {

 public:
  CollisionForce(vector<Particle*> rigidbodies, float threshold);

  float const threshold; // how close two rigid bodies collide
  void setTarget(vector<Particle*> rigidbodies) override;
  void draw() override;
  void apply(bool springsCanBreak) override;
 private:
  
};
