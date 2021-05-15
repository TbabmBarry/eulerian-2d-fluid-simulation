#pragma once
#include <Particle.h>
#include "System.h"

class System;
class Solver {
public:
    virtual void simulateStep(System* sys, float h) = 0;
}; 