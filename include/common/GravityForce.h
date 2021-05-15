#pragma once
#include <stdio.h>
#include <gfx/mat2.h>
#include "Particle.h"
#include "Force.h"
#include <map>

class GravityForce : public Force {
  public:
    GravityForce(vector<Particle*> particles, Vec2f g);
    void setTarget(vector<Particle*> particles) override;
    void apply(bool springsCanBreak) override;
    void draw() override;

    Vec2f g;     // gravity
    
};
