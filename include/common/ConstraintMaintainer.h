#ifndef MAINTAIN_CONSTRAINT_H
#define MAINTAIN_CONSTRAINT_H

#include "System.h"


class ConstraintMaintainer
{
    public:
        static void maintainConstraint(System *system, float m_ks, float m_kd);
};

#endif //MAINTAIN_CONSTRAINT_H