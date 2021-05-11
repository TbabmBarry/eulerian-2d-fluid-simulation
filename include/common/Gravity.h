#pragma once
#include <stdio.h>
#include <gfx/mat2.h>
#include "Particle.h"
#include <map>

class Gravity {
 public:
  Gravity(Particle *p, double mass);
  void apply();
  Vec2f Gravity::Jacobian();
  void draw();

 private:

  Particle * const m_p;   // particle
  Vec2f g=new Vec2f(0,9.8f);     // rest length
};
