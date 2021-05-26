#include <gfx/vec2.h>
#include <math.h>
#include "Constraint.h"

class FixedPointConstraint : public Constraint {
 public:
  FixedPointConstraint(Particle *p, const Vec2f & center);
  float C() override;
  float legalVelocity() override;
  vector<Vec2f> jacobian() override;
  vector<Vec2f> jacobianDerivative() override;
  void draw() override;

 private:

  Particle * const m_p;
  Vec2f const m_center;
};
