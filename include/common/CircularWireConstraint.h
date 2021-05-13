#pragma once
#include <gfx/vec2.h>
#include <math.h>
#include "Constraint.h"

class CircularWireConstraint : public Constraint {
 public:
  CircularWireConstraint(Particle *p, const Vec2f & center, const float radius);
  float C() override;
  float legal_velocity() override;
  float legal_accelerate() override;
  Vec2f ConstraintF();
  vector<Vec2f> Jacobian() override;
  vector<Vec2f> jd() override;
  void draw() override;

 private:

  Particle * const m_p;
  Vec2f const m_center;
  float const m_radius;
};
