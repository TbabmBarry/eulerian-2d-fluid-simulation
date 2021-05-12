#pragma once
#include <vector>
#include "Particle.h"
#include <gfx/vec2.h>
#include <math.h>

class RodConstraint {
 public:
  RodConstraint(Particle *p1, Particle * p2, double dist);
  float C();
  float legal_velocity();
  float legal_accelerate();
  std::vector<Vec2f> Jacobian();
  std::vector<Vec2f> jd();
  void draw();

 private:

  Particle * const m_p1;
  Particle * const m_p2;
  double const m_dist;
};
