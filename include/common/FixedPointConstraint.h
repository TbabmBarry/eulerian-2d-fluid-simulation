#include <gfx/vec2.h>
#include <math.h>
#include "Constraint.h"

class FixedPointConstraint : public Constraint {
 public:
  FixedPointConstraint(Particle *p, const Vector2f & center);
  float C() override;
  float legalVelocity() override;
  vector<Vector2f> jacobian() override;
  vector<Vector2f> jacobianDerivative() override;
  void draw() override;

 private:

  Particle * const m_p;
  Vector2f const m_center;
};
