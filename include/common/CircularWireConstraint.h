#pragma once
#include <vector>
#include <gfx/vec2.h>
#include <math.h>
#include "Particle.h"

class CircularWireConstraint {
 public:
  CircularWireConstraint(Particle *p, const Vec2f & center, const double radius);
  float C();
  float legal_velocity();
  float legal_accelerate();
  Vec2f ConstraintF();
  std::vector<Vec2f> Jacobian();
  std::vector<Vec2f> jd();
  void draw();

 private:

  Particle * const m_p;
  Vec2f const m_center;
  double const m_radius;
};
