#pragma once
#include <stdio.h>
#include <gfx/mat2.h>
#include "Particle.h"
#include "Force.h"
#include <map>

class WallCollision : public Force {
  public:
    WallCollision(vector<Particle*> particles, Vec2f g);
    void setTarget(vector<Particle*> particles) override;
    void apply(bool springsCanBreak) override;
    map<int, map<int, float>> dx() override;
    MatrixXf dv() override;
    void draw() override;
  private:
    Vec2f g = Vec2f(0.0f, -9.8f);
};
