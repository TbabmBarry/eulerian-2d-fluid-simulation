#pragma once
#include "Particle.h"
#include "Force.h"

class AngularSpring : public Force {
    
 public:
  AngularSpring(Particle *p1, Particle * midpoint,Particle * p3, double angle, double ks, double kd);
  AngularSpring(vector<Particle*> particles, float angle, float m_ks, float m_kd);
  
  void setTarget(vector<Particle*> particles) override;
  void apply(bool springsCanBreak) override;
  MatrixXf dx() override;
  MatrixXf dv() override;
  void draw() override;
  
 private:
  double const m_angle;         // cosine of rest angle
  double const m_ks, m_kd;      // spring strength constants
};