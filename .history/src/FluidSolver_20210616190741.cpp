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
	for ( int i=1 ; i<=N ; i++ ) {
		x[IX(0  ,i)] = b==1 ? -x[IX(1,i)] : x[IX(1,i)];//if b==1, set densities of all cells of upper boundary
		x[IX(N+1,i)] = b==1 ? -x[IX(N,i)] : x[IX(N,i)];//if b==1, set densities of all cells of lower boundary
		x[IX(i,0  )] = b==2 ? -x[IX(i,1)] : x[IX(i,1)];//if b==2, set densities of all cells of lhs boundary
		x[IX(i,N+1)] = b==2 ? -x[IX(i,N)] : x[IX(i,N)];//if b==2, set densities of all cells of rhs boundary
	}
	x[IX(0  ,0  )] = 0.5f*(x[IX(1,0  )]+x[IX(0  ,1)]);//set density of topleft corner cell
	x[IX(0  ,N+1)] = 0.5f*(x[IX(1,N+1)]+x[IX(0  ,N)]);//set density of topright corner cell
	x[IX(N+1,0  )] = 0.5f*(x[IX(N,0  )]+x[IX(N+1,1)]);//set density of bottomleft corner cell
	x[IX(N+1,N+1)] = 0.5f*(x[IX(N,N+1)]+x[IX(N+1,N)]);//set density of bottomright corner cell
}

void lin_solve ( int N, int b, float * x, float * x0, float a, float c )
{
	for (int k=0 ; k<20 ; k++ ) {
		for (int i=1 ; i<=N ; i++ ) { 
            for (int j=1 ; j<=N ; j++ ) {
                //for each not boundary cell, give it back-diffusion(stable) density/...
			    x[IX(i,j)] = (x0[IX(i,j)] + a*(x[IX(i-1,j)]+x[IX(i+1,j)]+x[IX(i,j-1)]+x[IX(i,j+1)]))/c;
            }
        }
		set_bnd ( N, b, x );//set densities
	}
}

void diffuse ( int N, int b, float * x, float * x0, float diff, float dt )
{
	float a=dt*diff*N*N;//apply stable diffusion "a"
	lin_solve ( N, b, x, x0, a, 1+4*a );
}

void advect ( int N, int b, float * d, float * d0, float * u, float * v, float dt )
{
	int i, j, i0, j0, i1, j1;
	float x, y, s0, t0, s1, t1, dt0;

	dt0 = dt*N;
    for (int i=1 ; i<=N ; i++ ) { 
        for (int j=1 ; j<=N ; j++ ) {
            x = i-dt0*u[IX(i,j)]; y = j-dt0*v[IX(i,j)];
            if (x<0.5f) x=0.5f; if (x>N+0.5f) x=N+0.5f; i0=(int)x; i1=i0+1;
            if (y<0.5f) y=0.5f; if (y>N+0.5f) y=N+0.5f; j0=(int)y; j1=j0+1;
            s1 = x-i0; s0 = 1-s1; t1 = y-j0; t0 = 1-t1;
            d[IX(i,j)] = s0*(t0*d0[IX(i0,j0)]+t1*d0[IX(i0,j1)])+
                        s1*(t0*d0[IX(i1,j0)]+t1*d0[IX(i1,j1)]);
        }
    }
	set_bnd ( N, b, d );
}