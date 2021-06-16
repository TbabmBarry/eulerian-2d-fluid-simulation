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

void set_bnd ( int N, int b, float* x )
{// Gauss-Seidel relaxiation
	for ( int i=1 ; i<=N ; i++ ) {
		x[IX(0  ,i)] = b==1 ? -x[IX(1,i)] : x[IX(1,i)];//if b==1, set densities of all cells of upper boundary
		x[IX(N+1,i)] = b==1 ? -x[IX(N,i)] : x[IX(N,i)];//if b==1, set densities of all cells of lower boundary
		x[IX(i,0  )] = b==2 ? -x[IX(i,1)] : x[IX(i,1)];//if b==2, set densities of all cells of lhs boundary
		x[IX(i,N+1)] = b==2 ? -x[IX(i,N)] : x[IX(i,N)];//if b==2, set densities of all cells of rhs boundary
	}
	x[IX(0  ,0  )] = 0.5f*(x[IX(1,0)]+x[IX(0,1)]);//set density of topleft corner cell
	x[IX(0  ,N+1)] = 0.5f*(x[IX(1,N+1)]+x[IX(0,N)]);//set density of topright corner cell
	x[IX(N+1,0  )] = 0.5f*(x[IX(N,0)]+x[IX(N+1,1)]);//set density of bottomleft corner cell
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

void advect ( int N, int b, float* d, float* d0, float* u, float* v, float dt )
{
	int i0, j0, i1, j1;
	float x, y, s0, t0, s1, t1, dt0;

	dt0 = dt*N;//???why  *N? why not *2, *3 or *100? I guess randomly set *N?
    for (int i=1 ; i<=N ; i++ ) {
        for (int j=1 ; j<=N ; j++ ) {
            //assume each cell center is a "particle" representing density
            //tracing(by velocity) "density cell particle" backforward for each cell
            x = i-dt0*u[IX(i,j)];
            if (x<0.5f) x=0.5f;//top boundary limit
            if (x>N+0.5f) x=N+0.5f;//bottom boundary limit
            i0=(int)x;//get cell id of traced back x
            i1=i0+1;//get cell id of next step

            y = j-dt0*v[IX(i,j)];
            if (y<0.5f) y=0.5f;//left boundary limit
            if (y>N+0.5f) y=N+0.5f;//right boundary limit
            j0=(int)y;//get cell id of traced back y
            j1=j0+1;//get cell id of next step

            s1 = x-i0;
            s0 = 1-s1;//i0+1-x 
            t1 = y-j0;
            t0 = 1-t1;//j0+1-y
            //summary:
            //current (i,j)
            //trace back get (x,y)->(i0,j0)
            //neighbor of (i0,j0)/(x,y) are ->(s0 right ,t0 top), (s1 left ,t1 bottom), (s0,t1), (s1,t0)
            d[IX(i,j)] = s0*(t0*d0[IX(i0,j0)]+
                        t1*d0[IX(i0,j1)])+
                        s1*(t0*d0[IX(i1,j0)]+
                        t1*d0[IX(i1,j1)]);
        }
    }
	set_bnd ( N, b, d );//flag b to update advection vector/(N+2)*(N+2)matrix d
}

void project ( int N, float* u, float* v, float* p, float* div )
{
	for (int i=1 ; i<=N ; i++ ) {
        for (int j=1 ; j<=N ; j++ ) {
            div[IX(i,j)] = -0.5f*(u[IX(i+1,j)]-u[IX(i-1,j)]+v[IX(i,j+1)]-v[IX(i,j-1)])/N;
            p[IX(i,j)] = 0;
        }
    }
	set_bnd ( N, 0, div );
    set_bnd ( N, 0, p );

	lin_solve ( N, 0, p, div, 1, 4 );

	for (int i=1 ; i<=N ; i++ ) {
        for (int j=1 ; j<=N ; j++ ) {
            u[IX(i,j)] -= 0.5f*N*(p[IX(i+1,j)]-p[IX(i-1,j)]);
            v[IX(i,j)] -= 0.5f*N*(p[IX(i,j+1)]-p[IX(i,j-1)]);
        }
    }
	set_bnd ( N, 1, u );
    set_bnd ( N, 2, v );
}
