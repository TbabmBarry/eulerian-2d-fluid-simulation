#include "Fluid.h"
#include "Eigen/Dense"
#include <Eigen/Sparse>
#include <math.h>
#include "Eigen/IterativeLinearSolvers"

#define IX(i, j) ((i) + (N + 2) * (j))
#define SWAP(x0, x)      \
    {                    \
        float *tmp = x0; \
        x0 = x;          \
        x = tmp;         \
    }

Fluid::Fluid() {}

void Fluid::add_source(int N, float *x, float *s, float dt)
{
    h = dt;
    int i, size = (N + 2) * (N + 2);
    for (i = 0; i < size; i++)
    {
        x[i] += dt * s[i];
    }
}

void Fluid::set_bnd(int N, int b, float *x)
{ // Gauss-Seidel relaxiation
    for (int i = 1; i <= N; i++)
    {
        x[IX(0, i)] = b == 1 ? -x[IX(1, i)] : x[IX(1, i)];     //if b==1, set densities of all cells of upper boundary
        x[IX(N + 1, i)] = b == 1 ? -x[IX(N, i)] : x[IX(N, i)]; //if b==1, set densities of all cells of lower boundary
        x[IX(i, 0)] = b == 2 ? -x[IX(i, 1)] : x[IX(i, 1)];     //if b==2, set densities of all cells of lhs boundary
        x[IX(i, N + 1)] = b == 2 ? -x[IX(i, N)] : x[IX(i, N)]; //if b==2, set densities of all cells of rhs boundary
    }
    x[IX(0, 0)] = 0.5f * (x[IX(1, 0)] + x[IX(0, 1)]);                 //set density of topleft corner cell
    x[IX(0, N + 1)] = 0.5f * (x[IX(1, N + 1)] + x[IX(0, N)]);         //set density of topright corner cell
    x[IX(N + 1, 0)] = 0.5f * (x[IX(N, 0)] + x[IX(N + 1, 1)]);         //set density of bottomleft corner cell
    x[IX(N + 1, N + 1)] = 0.5f * (x[IX(N, N + 1)] + x[IX(N + 1, N)]); //set density of bottomright corner cell

    //internal boundary to deal with rigidbodies->fluid
    for (Particle *rigidBody : rigidbodies)
    {
        // cout<<"rigid_corners"<< rigidBody->corners[0]<<" "<< rigidBody->corners[1]<<" "<< rigidBody->corners[2]<<" "<< rigidBody->corners[3]<<" "<<endl;
        vector<Vector4f> boundgrids = rigidBody->BoundingGrid(N);
        vector<Vector2i> innergrids = rigidBody->InnerGrid(boundgrids);
        // cout << " innersize " << innergrids.size() << endl;
        // cout<< "new bound"<<endl;
        // for (int a=0; a<boundgrids.size();a++){
        //     cout<< "boundgrids:" << boundgrids[a][0] << " "<< boundgrids[a][1] <<endl;
        // }
        //here we discuss 3 fluid cases wrt a rigidbody:
        //1. velocity case
        //2. density case
        //3. anything inside rigid body

        for (int i = 0; i < boundgrids.size(); i++)
        {
            //if case1: first flip fluid value(Gauss-Seidel relaxiation),
            //then if rigid grid is on upper/lower canvas boundary, fluid value is changed based on x velocity field
            //if rigid grid is on lhs/rhs canvas boundary, fluid value is changed based on y velocity field
            if (b == 1 || b == 2)
            {
                x[IX((int)boundgrids[i][0], (int)boundgrids[i][1])] *= -1; //flip velocity
                Vector2f vel = rigidBody->m_Velocity;
                float factor = h * 0.05;
                //if the grid is also on canvas boundary
                if (b == 1)
                {
                    x[IX((int)boundgrids[i][0], (int)boundgrids[i][1])] += vel[0] * factor;
                }
                else if (b == 2)
                {
                    x[IX((int)boundgrids[i][0], (int)boundgrids[i][1])] += vel[1] * factor;
                }
            }
            else
            {
                //if case2: Assign average value of fluid neigboring cells(grids) to current cell
                //Value in x field around current cell
                // cout<<"before: "<<x[IX((int)boundgrids[i][0], (int)boundgrids[i][1])]<<endl;
                float before = (checkinner((int)boundgrids[i][0] - 1, (int)boundgrids[i][1], innergrids) == 1) ? 0 : 1;
                float after = (checkinner((int)boundgrids[i][0] + 1, (int)boundgrids[i][1], innergrids) == 1) ? 0 : 1;
                float above = (checkinner((int)boundgrids[i][0], (int)boundgrids[i][1] + 1, innergrids) == 1) ? 0 : 1;
                float below = (checkinner((int)boundgrids[i][0], (int)boundgrids[i][1] - 1, innergrids) == 1) ? 0 : 1;
                float x_before = (before == 1) ? x[IX((int)boundgrids[i][0] - 1, (int)boundgrids[i][1])] : 0;
                float x_after = (after == 1) ? x[IX((int)boundgrids[i][0] + 1, (int)boundgrids[i][1])] : 0;
                float x_above = (above == 1) ? x[IX((int)boundgrids[i][0], (int)boundgrids[i][1] + 1)] : 0;
                float x_below = (below == 1) ? x[IX((int)boundgrids[i][0], (int)boundgrids[i][1] - 1)] : 0;
                // cout<<(x_before + x_after + x_above + x_below) / x[IX((int)boundgrids[i][0], (int)boundgrids[i][1])]<<endl;
                // x[IX((int)boundgrids[i][0], (int)boundgrids[i][1])] *= 1;
                x[IX((int)boundgrids[i][0], (int)boundgrids[i][1])] = (x_before + x_after + x_above + x_below) / (before + after + above + below);
                // cout<<"after: "<<x[IX((int)boundgrids[i][0], (int)boundgrids[i][1])]<<endl;
            }
        }
        //if case3: for all fluid grids inside rigid body(i.e not boundary grids), assign them=0
        for (int i = 0; i < innergrids.size(); i++)
        {
            x[IX((int)innergrids[i][0], (int)innergrids[i][1])] = 0;
        }
    }
}

int Fluid::checkinner(int i, int j, vector<Vector2i> pos)
{
    for (int idx = 0; idx < pos.size(); idx++)
    {
        if (i == pos[idx][0] && j == pos[idx][1])
        {
            return 1;
        }
    }
    return 0;
}

void Fluid::lin_solve(int N, int b, float *x, float *x0, float a, float c)
{
    for (int k = 0; k < 20; k++)
    {
        for (int i = 1; i <= N; i++)
        {
            for (int j = 1; j <= N; j++)
            {
                //for each not boundary cell, give it back-diffusion(stable) density/...
                x[IX(i, j)] = (x0[IX(i, j)] + a * (x[IX(i - 1, j)] + x[IX(i + 1, j)] + x[IX(i, j - 1)] + x[IX(i, j + 1)])) / c;
            }
        }
        set_bnd(N, b, x); //set densities
    }
}

void Fluid::diffuse(int N, int b, float *x, float *x0, float diff, float dt)
{
    float a = dt * diff * N * N; //apply stable diffusion "a"
    lin_solve(N, b, x, x0, a, 1 + 4 * a);
}

void Fluid::advect(int N, int b, float *d, float *d0, float *u, float *v, float dt)
{
    int i0, j0, i1, j1;
    float x, y, s0, t0, s1, t1, dt0;

    dt0 = dt * N; //???why  *N? why not *2, *3 or *100? I guess randomly set *N?
    for (int i = 1; i <= N; i++)
    {
        for (int j = 1; j <= N; j++)
        {
            //assume each cell center is a "particle" representing density
            //tracing(by velocity) "density cell particle" backforward for each cell
            x = i - dt0 * u[IX(i, j)];
            if (x < 0.5f)
                x = 0.5f; //top boundary limit
            if (x > N + 0.5f)
                x = N + 0.5f; //bottom boundary limit
            i0 = (int)x;      //get cell id of traced back x
            i1 = i0 + 1;      //get cell id of next step

            y = j - dt0 * v[IX(i, j)];
            if (y < 0.5f)
                y = 0.5f; //left boundary limit
            if (y > N + 0.5f)
                y = N + 0.5f; //right boundary limit
            j0 = (int)y;      //get cell id of traced back y
            j1 = j0 + 1;      //get cell id of next step

            s1 = x - i0;
            s0 = 1 - s1; //i0+1-x
            t1 = y - j0;
            t0 = 1 - t1; //j0+1-y
            //summary:
            //current (i,j)
            //trace back get (x,y)->(i0,j0)
            //neighbor of (i0,j0)/(x,y) are ->(s0 right ,t0 top), (s1 left ,t1 bottom), (s0,t1), (s1,t0)
            d[IX(i, j)] = s0 * (t0 * d0[IX(i0, j0)] +
                                t1 * d0[IX(i0, j1)]) +
                          s1 * (t0 * d0[IX(i1, j0)] +
                                t1 * d0[IX(i1, j1)]);
        }
    }
    set_bnd(N, b, d); //flag b to update advection vector/(N+2)*(N+2)matrix d
}

void Fluid::project(int N, float *u, float *v, float *p, float *div)
{ //velocity field, velocity conservation.
    //velocity field = mass conservation + velocity gradient
    for (int i = 1; i <= N; i++)
    {
        for (int j = 1; j <= N; j++)
        {
            div[IX(i, j)] = -0.5f * (u[IX(i + 1, j)] - u[IX(i - 1, j)] + v[IX(i, j + 1)] - v[IX(i, j - 1)]) / N; //velocity function
            p[IX(i, j)] = 0;                                                                                     //pressure states/field
        }
    }
    set_bnd(N, 0, div);
    set_bnd(N, 0, p);
    // mass conservation = implicit method, solve for pressure linear system
    //i.e P state neighbors and velocity relations
    lin_solve(N, 0, p, div, 1, 4);

    //velocity gradient = second derivateive of P field
    for (int i = 1; i <= N; i++)
    {
        for (int j = 1; j <= N; j++)
        {
            u[IX(i, j)] -= 0.5f * N * (p[IX(i + 1, j)] - p[IX(i - 1, j)]);
            v[IX(i, j)] -= 0.5f * N * (p[IX(i, j + 1)] - p[IX(i, j - 1)]);
        }
    }
    set_bnd(N, 1, u); //set velocity field
    set_bnd(N, 2, v); //set velocity field
}

void Fluid::vorticity_confinement(int N, float dt, float *d0, float *u, float *v, float *u0, float *v0)
{
    float *curl = d0;
    //compute vorticity
    //totalDens = 0;
    setDensity(density, d0, N);
    setVelocity(u, v, u0, v0, N);
    float x, y, z;
    for (int i = 1; i <= N; i++)
    {
        for (int j = 1; j <= N; j++)
        {
            //totalDens += this.dens[IX(i,j)];
            // curlx = dw/dy - dv/dz
            x = (u[IX(i + 1, j)] - u[IX(i - 1, j)]) * 0.5;
            // curly = du/dz - dw/dx
            y = (v[IX(i, j + 1)] - v[IX(i, j - 1)]) * 0.5;
            // curlz = dv/dx - du/dy
            z = 0;
            // curl = |curl|
            curl[IX(i, j)] = sqrt(x * x + y * y + z * z);
        }
    }
    //add vorticity confinement
    float Nx, Ny, len;
    for (int i = 1; i <= N; i++)
    {
        for (int j = 1; j < N; j++)
        {
            Nx = (curl[IX(i + 1, j)] - curl[IX(i - 1, j)]) * 0.5;
            Ny = (curl[IX(i, j + 1)] - curl[IX(i, j - 1)]) * 0.5;
            //normalize
            len = 1 / (sqrt(Nx * Nx + Ny * Ny) + 0.0000000000000000001);
            Nx *= len;
            Ny *= len;
            u[IX(i, j)] += dt * Nx * u0[IX(i, j)];
            v[IX(i, j)] += dt * Ny * v0[IX(i, j)];
        }
    }
}

void Fluid::setDensity(float *d, float *dprev, int inputN)
{
    // cout<<"hello1 "<<dprev[IX(64, 82)]<<endl;
    density = d;
    density_previous = dprev;
    // cout<<"hello2 "<<density_previous[IX(64, 82)]<<endl;
    N = inputN;
};

float Fluid::getDensity(int i, int j)
{
    int idx = i + (N + 2) * j;
    return *(density + idx);
};

void Fluid::setVelocity(float *xu, float *xv, float *xuprev, float *xvprev, int inputN)
{
    u = xu;
    v = xv;
    u_previous = xuprev;
    v_previous = xvprev;
    N = inputN;
};

float Fluid::getXVelocity(int i, int j)
{
    int idx = i + (N + 2) * j;
    return *(u + idx);
};

float Fluid::getYVelocity(int i, int j)
{
    int idx = i + (N + 2) * j;
    return *(v + idx);
};

void Fluid::dens_step(int N, float *x, float *x0, float *u, float *v, float diff, float dt)
{
    // cout<<"hello "<<x0[IX(64, 82)]<<endl;
    setDensity(x, x0, N);
    add_source(N, x, x0, dt);
    SWAP(x0, x);
    diffuse(N, 0, x, x0, diff, dt);
    SWAP(x0, x);
    advect(N, 0, x, x0, u, v, dt);
    setDensity(x, x0, N);
}

void Fluid::vel_step(int N, float *u, float *v, float *u0, float *v0, float visc, float dt)
{
    setVelocity(u, v, u0, v0, N);
    add_source(N, u, u0, dt);
    add_source(N, v, v0, dt);
    SWAP(u0, u);
    diffuse(N, 1, u, u0, visc, dt);
    SWAP(v0, v);
    diffuse(N, 2, v, v0, visc, dt);
    project(N, u, v, u0, v0);
    SWAP(u0, u);
    SWAP(v0, v);
    advect(N, 1, u, u0, u0, v0, dt);
    advect(N, 2, v, v0, u0, v0, dt);
    project(N, u, v, u0, v0);
    setVelocity(u, v, u0, v0, N);
}
