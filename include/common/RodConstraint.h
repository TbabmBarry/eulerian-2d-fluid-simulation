#pragma once
#include "Constraint.h"
#include <gfx/vec2.h>
#include <math.h>

class RodConstraint : public Constraint {
 public:
  RodConstraint(Particle *p1, Particle * p2, float dist);
  float C() override;
  float legalVelocity() override;
  vector<Vector2f> jacobian() override;
  vector<Vector2f> jacobianDerivative() override;
  void draw() override;

 private:

  Particle * const m_p1;
  Particle * const m_p2;
  float const m_dist;
};
