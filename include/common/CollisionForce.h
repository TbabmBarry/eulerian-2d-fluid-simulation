#pragma once
#include <stdio.h>
#include "Force.h"
#include "Particle.h"

class CollisionForce : public Force {

 public:
  CollisionForce(Particle* rb1, Particle* rb2, float threshold, float epsilon);
  CollisionForce(vector<Particle*> rigidbodies, float threshold, float epsilon);

  float const threshold; // how close two rigid bodies collide
  void setTarget(vector<Particle*> rigidbodies) override;
  void draw() override;
  void apply(bool springsCanBreak) override;
  map<int, map<int, float>> dx() override;
  MatrixXf dv() override;
  bool colliding(Vector2f point, Particle* rb1, Particle* rb2);
  void collision(Vector2f point, Particle* rb1, Particle* rb2);
 private:
  float const epsilon;
};
