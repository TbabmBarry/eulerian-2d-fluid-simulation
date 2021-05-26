#pragma once
#include <stdio.h>
#include "Force.h"
#include <gfx/mat2.h>
#include "Particle.h"
#include <map>

class ExternalForce : public Force {
public:
    ExternalForce(std::vector<Particle*> particles, float force, Vec2f direction);

    void setTarget(std::vector<Particle*> particles) override;
    void apply(bool springsCanBreak) override;
    map<int, map<int, float>> dx() override;
    MatrixXf dv() override;
    void draw() override;

    void setActive(bool state);
    void setForce(float f);
    Vec2f direction;
    float force;
};
