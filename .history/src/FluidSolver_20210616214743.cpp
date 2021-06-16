#include "FluidSolver.h"
#include "Eigen/Dense"
#include <Eigen/Sparse>
#include <math.h>
#include "Eigen/IterativeLinearSolvers"

#define IX(i,j) ((i)+(N+2)*(j))
#define SWAP(x0,x) {float * tmp=x0;x0=x;x=tmp;}

FluidSolver::FluidSolver(float* u,float* v) : u(u), v(v){
    for (int i = 0; i < (N+2)*(N+2); i++)
        u_previous[i] = v_previous[i] = densisty_previous[i] = 0.0;
}

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
{//velocity field, velocity conservation. 
    //velocity field = mass conservation + velocity gradient
	for (int i=1 ; i<=N ; i++ ) {
        for (int j=1 ; j<=N ; j++ ) {
            div[IX(i,j)] = -0.5f*(u[IX(i+1,j)]-u[IX(i-1,j)]+v[IX(i,j+1)]-v[IX(i,j-1)])/N;//velocity function
            p[IX(i,j)] = 0;//pressure states/field
        }
    }
	set_bnd ( N, 0, div );
    set_bnd ( N, 0, p );
	// mass conservation = implicit method, solve for pressure linear system
    //i.e P state neighbors and velocity relations
    lin_solve ( N, 0, p, div, 1, 4 );

	//velocity gradient = second derivateive of P field
    for (int i=1 ; i<=N ; i++ ) {
        for (int j=1 ; j<=N ; j++ ) {
            u[IX(i,j)] -= 0.5f*N*(p[IX(i+1,j)]-p[IX(i-1,j)]);
            v[IX(i,j)] -= 0.5f*N*(p[IX(i,j+1)]-p[IX(i,j-1)]);
        }
    }
	set_bnd ( N, 1, u );//set velocity field
    set_bnd ( N, 2, v );//set velocity field
}

void FluidSolver::vorticity_confinement(int N, float dt){
    // int curl = this->dens_previous.slice();
    int n = N+2;

    //compute vorticity
    //totalDens = 0;
    float x,y,z;
    for (int i = 1; i <= N; i++) {
        for (int j = 1; j <= N; j++) {
            //totalDens += this.dens[IX(i,j)];
            // curlx = dw/dy - dv/dz
            x = (u[IX(i+1,j)] - u[IX(i-1,j)]) / 2;
            // curly = du/dz - dw/dx
            y = (v[IX(i, j+1)] - v[IX(i, j-1)]) / 2;
            // curlz = dv/dx - du/dy
            z = 0;

            // curl = |curl|
            curl[IX(i,j)] = sqrt(x*x+y*y+z*z);
        }
    }
    //add vorticity confinement
    for (int i = 1; i <= N; i++) {
        for (int j = 1; j < N; j++) {
            float Nx = (curl[IX(i+1, j)] - curl[IX(i-1, j)]) * 0.5;
            float Ny = (curl[IX(i, j+1)] - curl[IX(i, j-1)]) * 0.5;

            //normalize
            float len1 = 1/(sqrt(Nx*Nx + Ny*Ny) + 0.0000000000000000001);
            Nx *= len1;
            Ny *= len1;
            u[IX(i,j)] += Nx*u_prev[IX(i,j)];
            v[IX(i,j)] += Ny*v_prev[IX(i,j)];
        }
    }
}

void dens_step ( int N, float* x, float* x0, float* u, float* v, float diff, float dt )
{
	add_source ( N, x, x0, dt );
	SWAP ( x0, x );
    diffuse ( N, 0, x, x0, diff, dt );
	SWAP ( x0, x );
    advect ( N, 0, x, x0, u, v, dt );
}

void vel_step ( int N, float * u, float * v, float * u0, float * v0, float visc, float dt )
{
	add_source ( N, u, u0, dt );
    add_source ( N, v, v0, dt );
	SWAP ( u0, u );
    diffuse ( N, 1, u, u0, visc, dt );
	SWAP ( v0, v );
    diffuse ( N, 2, v, v0, visc, dt );
	project ( N, u, v, u0, v0 );
	SWAP ( u0, u );
    SWAP ( v0, v );
	advect ( N, 1, u, u0, u0, v0, dt );
    advect ( N, 2, v, v0, u0, v0, dt );
	project ( N, u, v, u0, v0 );
}
