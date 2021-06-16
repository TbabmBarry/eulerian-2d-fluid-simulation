#include "FluidSolver.h"
#include "Eigen/Dense"
#include <Eigen/Sparse>
#include "Eigen/IterativeLinearSolvers"

#define IX(i,j) ((i)+(N+2)*(j))

FluidSolver::FluidSolver() {}

void FluidSolver::simulateStep(System *system, float h){
}

void add_source ( int N, float* x, float* s, float dt )
{
	int i, size=(N+2)*(N+2);
	for ( i=0 ; i<size ; i++ ) {
        x[i] += dt*s[i];
    }
}

void set_bnd ( int N, int b, float * x )
{
	int i;
	for ( i=1 ; i<=N ; i++ ) {
		x[IX(0  ,i)] = b==1 ? -x[IX(1,i)] : x[IX(1,i)];//if b==1, set all cells of upper boundary
		x[IX(N+1,i)] = b==1 ? -x[IX(N,i)] : x[IX(N,i)];//if b==1, set all cells of lower boundary
		x[IX(i,0  )] = b==2 ? -x[IX(i,1)] : x[IX(i,1)];//if b==2, set all cells of lhs boundary
		x[IX(i,N+1)] = b==2 ? -x[IX(i,N)] : x[IX(i,N)];//if b==2, set all cells of rhs boundary
	}
	x[IX(0  ,0  )] = 0.5f*(x[IX(1,0  )]+x[IX(0  ,1)]);//set topleft corner cell
	x[IX(0  ,N+1)] = 0.5f*(x[IX(1,N+1)]+x[IX(0  ,N)]);//set topright corner cell
	x[IX(N+1,0  )] = 0.5f*(x[IX(N,0  )]+x[IX(N+1,1)]);//set bottomleft corner cell
	x[IX(N+1,N+1)] = 0.5f*(x[IX(N,N+1)]+x[IX(N+1,N)]);//set bottomright corner cell
}

void lin_solve ( int N, int b, float * x, float * x0, float a, float c )
{
	int i, j, k;

	for ( k=0 ; k<20 ; k++ ) {
		FOR_EACH_CELL
			x[IX(i,j)] = (x0[IX(i,j)] + a*(x[IX(i-1,j)]+x[IX(i+1,j)]+x[IX(i,j-1)]+x[IX(i,j+1)]))/c;
		END_FOR
		set_bnd ( N, b, x );
	}
}

void diffuse ( int N, int b, float * x, float * x0, float diff, float dt )
{
	float a=dt*diff*N*N;
	lin_solve ( N, b, x, x0, a, 1+4*a );
}