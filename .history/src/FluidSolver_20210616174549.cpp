#include "FluidSolver.h"
#include "Eigen/Dense"
#include <Eigen/Sparse>
#include "Eigen/IterativeLinearSolvers"

FluidSolver::FluidSolver() {}

void FluidSolver::simulateStep(System *system, float h){
}

void add_source ( int N, float * x, float * s, float h )
{
	int i, size=(N+2)*(N+2);
	for ( i=0 ; i<size ; i++ ) x[i] += dt*s[i];
}