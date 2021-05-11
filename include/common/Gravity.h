#pragma once
#include <stdio.h>
#include <mat2.h>
#include "Particle.h"
#include <map>

class Gravity {
 public:
  Gravity(Particle *p, double mass);
  void apply();
  Mat2**  Gravity::Jacobian();
  void draw();

 private:

  Particle * const m_p;   // particle
  Vec2f g=(0,9.8);     // rest length
};
