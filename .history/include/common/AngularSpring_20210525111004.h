#pragma once
#include "Particle.h"
#include "Force.h"

class AngularSpring : public Force {
    
 public:
  AngularSpring(Particle *p1, Particle * midpoint,Particle * p3, float m_angle, float m_ks, float m_kd);
  AngularSpring(vector<Particle*> particles, float m_dist, float m_ks, float m_kd);
  
  void setTarget(vector<Particle*> particles) override;
  void apply(bool springsCanBreak) override;
  map<int, map<int, float>> dx() override;
  MatrixXf dv() override;
  void draw() override;
  
 private:
  float const m_dist;         // cosine of rest angle
  float const m_ks, m_kd;      // spring strength constants
};