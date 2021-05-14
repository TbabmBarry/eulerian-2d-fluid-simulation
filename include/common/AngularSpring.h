#pragma once
#include "Particle.h"
#include "Force.h"

class AngularSpring : public Force {
    
 public:
  AngularSpring(Particle *p1, Particle * p2,Particle * p3, double angle, double ks, double kd);
  
  void setTarget(vector<Particle*> particles) override;
  void apply(bool springsCanBreak) override;
  void draw() override;

 private:
  double const m_angle;         // cosine of rest angle
  double const m_ks, m_kd;      // spring strength constants
};