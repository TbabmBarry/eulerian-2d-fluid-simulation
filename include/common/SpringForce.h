#pragma once
#include <stdio.h>
#include <gfx/mat2.h>
#include "Particle.h"
#include <map>

class SpringForce {
 public:
  SpringForce(Particle *p1, Particle * p2, double dist, double ks, double kd);
  void apply(bool springsCanBreak);
  Mat2**  Jacobian();
  void draw();

 private:

  Particle * const m_p1;   // particle 1
  Particle * const m_p2;   // particle 2
  double const m_dist;     // rest length
  double const m_ks, m_kd; // spring strength constants
};
