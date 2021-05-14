#pragma once
#include <gfx/vec2.h>
#include <math.h>
#include "Constraint.h"

class CircularWireConstraint : public Constraint {
 public:
  CircularWireConstraint(Particle *p, const Vec2f & center, const float radius);
  float C() override;
  float legalVelocity() override;
  Vec2f ConstraintF();
  vector<Vec2f> jacobian() override;
  vector<Vec2f> jacobianDerivative() override;
  void draw() override;

 private:

  Particle * const m_p;
  Vec2f const m_center;
  float const m_radius;
};
