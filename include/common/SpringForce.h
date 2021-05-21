#pragma once
#include <stdio.h>
#include "Force.h"
#include "Particle.h"
#include <map>

class SpringForce : public Force {

 public:
  SpringForce(Particle *p1, Particle * p2, float m_dist, float m_ks, float m_kd);
  SpringForce(vector<Particle*> particles, float m_dist, float m_ks, float m_kd);

  void draw() override;
  void setTarget(vector<Particle*> particles) override;
  void apply(bool springsCanBreak) override;
  map<int, map<int, float>> dx() override;
  MatrixXf dv() override;

 private:
  float const m_dist;     // rest length
  float const m_ks, m_kd; // spring strength constants
};
