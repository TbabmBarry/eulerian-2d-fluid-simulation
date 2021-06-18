#pragma once
#include <stdio.h>
#include "Particle.h"
#include "Force.h"
#include <map>

class GravityForce : public Force {
  public:
    GravityForce(vector<Particle*> particles, Vector2f g);
    void setTarget(vector<Particle*> particles) override;
    void apply(bool springsCanBreak) override;
    map<int, map<int, float>> dx() override;
    MatrixXf dv() override;
    void draw() override;
  private:
    Vector2f g = Vector2f(0.0f, -9.8f);     // gravity
};
