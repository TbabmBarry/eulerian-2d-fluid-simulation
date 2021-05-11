#pragma once
#include <vector>
#include <gfx/vec2.h>
#include <math.h>
#include "Particle.h"

class CircularWireConstraint {
 public:
  CircularWireConstraint(Particle *p, const Vec2f & center, const double radius);
  float C();
  float Cd();
  std::vector<Vec2f> j();
  std::vector<Vec2f> jd();
  void draw();

 private:

  Particle * const m_p;
  Vec2f const m_center;
  double const m_radius;
};
