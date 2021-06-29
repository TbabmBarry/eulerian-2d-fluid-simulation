#include "FluidField.h"
#include <GL/glut.h>
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

FluidField::FluidField(System *sys, int grid_N) : sys(sys), N(grid_N), dt(0.1f), diff(0.0f), visc(0.0f)
{
    allocate();
    int i, j, size = (grid_N + 2) * (grid_N + 2);
    for (i = 0; i < size; i++)
    {
        u[i] = v[i] = density[i] = 0.0f;
    }
}

void FluidField::simulationStep()
{
    velStep();
    densStep();
    vorticityConfinement();
}

void FluidField::addSource(float *x, float *s)
{
    int i, size = (N + 2) * (N + 2);
    for (i = 0; i < size; i++)
    {
        x[i] += dt * s[i];
    }
}

void FluidField::setBnd(int b, float *x)
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
    // for (Particle *rigidBody : sys->rigidbodies)
    // {
    //     vector<Vector4f> boundgrids = rigidBody->BoundingGrid(N);
    //     vector<Vector2i> innergrids = rigidBody->InnerGrid(boundgrids);
    //     //here we discuss 3 fluid cases wrt a rigidbody:
    //     //1. grid is boundary of rigidbody && the grid is on canvas boundary
    //     //2. grid is boundary of rigidbody && the grid is not on canvas boundary
    //     //3. grid inside boundary of rigidbody
    //     for (int i = 0; i < boundgrids.size(); i++)
    //     {
    //         //if case1: first flip fluid value(Gauss-Seidel relaxiation),
    //         //then if rigid grid is on upper/lower canvas boundary, fluid value is changed based on x velocity field
    //         //if rigid grid is on lhs/rhs canvas boundary, fluid value is changed based on y velocity field
    //         if (b == 1 || b == 2)
    //         {
    //             x[IX((int)boundgrids[i][0], (int)boundgrids[i][1])] *= -1; //flip value according to above(Gauss-Seidel relaxiation)
    //             //Find rigid body
    //             // Vector2f vel = Vector2f(getXVelocity(rigidBody->x[0], rigidBody->x[1]), getYVelocity(rigidBody->x[0], rigidBody->x[1]));
    //             float factor = dt * 1.1;
    //             if (b == 1)
    //             {
    //                 x[IX((int)boundgrids[i][0], (int)boundgrids[i][1])] += rigidBody->m_Velocity[0] * factor;
    //             }
    //             else if (b == 2)
    //             {
    //                 x[IX((int)boundgrids[i][0], (int)boundgrids[i][1])] += rigidBody->m_Velocity[1] * factor;
    //             }
    //         }
    //         else
    //         {
    //             //if case2: Assign average value of fluid neigboring cells(grids) to current cell
    //             //Value in x field around current cell
    //             float x_before = x[IX((int)boundgrids[i][0] - 1, (int)boundgrids[i][1])];
    //             float x_after = x[IX((int)boundgrids[i][0] + 1, (int)boundgrids[i][1])];
    //             float x_above = x[IX((int)boundgrids[i][0], (int)boundgrids[i][1] + 1)];
    //             float x_below = x[IX((int)boundgrids[i][0], (int)boundgrids[i][1] - 1)];
    //             x[IX((int)boundgrids[i][0], (int)boundgrids[i][1])] = (x_before + x_after + x_above + x_below) / x[IX((int)boundgrids[i][0], (int)boundgrids[i][1])];
    //         }
    //     }
    //     //if case3: for all fluid grids inside rigid body(i.e not boundary grids), assign them=0
    //     for (int i = 0; i < innergrids.size(); i++)
    //     {
    //         x[IX((int)innergrids[i][0], (int)innergrids[i][1])] = 0.0f;
    //     }
    // }
}

void FluidField::linSolve(int b, float *x, float *x0, float a, float c)
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
        setBnd(b, x); //set densities
    }
}

void FluidField::diffuse(int b, float *x, float *x0, float diff)
{
    float a = dt * diff * N * N; //apply stable diffusion "a"
    linSolve(b, x, x0, a, 1 + 4 * a);
}

void FluidField::advect(int b, float *d, float *d0, float *u, float *v)
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
    setBnd(b, d); //flag b to update advection vector/(N+2)*(N+2)matrix d
}

void FluidField::project(float *u, float *v, float *p, float *div)
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
    setBnd(0, div);
    setBnd(0, p);
    // mass conservation = implicit method, solve for pressure linear system
    //i.e P state neighbors and velocity relations
    linSolve(0, p, div, 1, 4);

    //velocity gradient = second derivateive of P field
    for (int i = 1; i <= N; i++)
    {
        for (int j = 1; j <= N; j++)
        {
            u[IX(i, j)] -= 0.5f * N * (p[IX(i + 1, j)] - p[IX(i - 1, j)]);
            v[IX(i, j)] -= 0.5f * N * (p[IX(i, j + 1)] - p[IX(i, j - 1)]);
        }
    }
    setBnd(1, u); //set velocity field
    setBnd(2, v); //set velocity field
}

void FluidField::vorticityConfinement()
{
    float *curl = density_previous;
    //compute vorticity
    //totalDens = 0;
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
            u[IX(i, j)] += Nx * u_previous[IX(i, j)];
            v[IX(i, j)] += Ny * v_previous[IX(i, j)];
        }
    }
}

// void FluidField::densStep( int N, float* x, float* x0, float* u, float* v, float diff, float dt )
// {
// 	add_source (x, x0);
// 	SWAP ( x0, x );
//     diffuse (0, x, x0, diff);
// 	SWAP ( x0, x );
//     advect (0, x, x0, u, v);
// }

void FluidField::densStep()
{
    addSource(density, density_previous);
    SWAP(density_previous, density);
    diffuse(0, density, density_previous, diff);
    SWAP(density_previous, density);
    advect(0, density, density_previous, u, v);
}

// void FluidField::velStep( int N, float * u, float * v, float * u0, float * v0, float visc, float dt )
// {
// 	add_source (u, u0);
//     add_source (v, v0);
// 	SWAP ( u0, u );
//     diffuse (1, u, u0, visc);
// 	SWAP ( v0, v );
//     diffuse (2, v, v0, visc);
// 	project (u, v, u0, v0);
// 	SWAP ( u0, u );
//     SWAP ( v0, v );
// 	advect (1, u, u0, u0, v0);
//     advect (2, v, v0, u0, v0);
// 	project (u, v, u0, v0);
// }

void FluidField::velStep()
{
    addSource(u, u_previous);
    addSource(v, v_previous);
    SWAP(u_previous, u);
    diffuse(1, u, u_previous, visc);
    SWAP(v_previous, v);
    diffuse(2, v, v_previous, visc);
    project(u, v, u_previous, v_previous);
    SWAP(u_previous, u);
    SWAP(v_previous, v);
    advect(1, u, u_previous, u_previous, v_previous);
    advect(2, v, v_previous, u_previous, v_previous);
    project(u, v, u_previous, v_previous);
}

void FluidField::drawVelocity()
{
    int i, j;
    float x, y, h;

    h = 2.0f / N;

    glColor3f(1.0f, 1.0f, 1.0f);
    glLineWidth(1.0f);

    glBegin(GL_LINES);

    for (i = 1; i <= N; i++)
    {
        x = (i - 0.5f) * h;
        x = -1 + x;
        for (j = 1; j <= N; j++)
        {
            y = (j - 0.5f) * h;
            y = -1 + y;
            glVertex2f(x, y);
            glVertex2f(x + u[IX(i, j)], y + v[IX(i, j)]);
        }
    }

    glEnd();
}

void FluidField::drawDensity()
{
    int i, j;
    float x, y, h, d00, d01, d10, d11;

    h = 2.0f / N;

    glBegin(GL_QUADS);

    for (i = 0; i <= N; i++)
    {
        x = (i - 0.5f) * h;
        x = -1 + x;
        for (j = 0; j <= N; j++)
        {
            y = (j - 0.5f) * h;
            y = -1 + y;

            d00 = density[IX(i, j)];
            d01 = density[IX(i, j + 1)];
            d10 = density[IX(i + 1, j)];
            d11 = density[IX(i + 1, j + 1)];

            glColor3f(d00, d00, d00);
            glVertex2f(x, y);
            glColor3f(d10, d10, d10);
            glVertex2f(x + h, y);
            glColor3f(d11, d11, d11);
            glVertex2f(x + h, y + h);
            glColor3f(d01, d01, d01);
            glVertex2f(x, y + h);
        }
    }

    glEnd();
}

bool FluidField::allocate()
{
    int size = (N + 2) * (N + 2);

    u = (float *)malloc(size * sizeof(float));
    v = (float *)malloc(size * sizeof(float));
    u_previous = (float *)malloc(size * sizeof(float));
    v_previous = (float *)malloc(size * sizeof(float));
    density = (float *)malloc(size * sizeof(float));
    density_previous = (float *)malloc(size * sizeof(float));

    if (!u || !v || !u_previous || !v_previous || !density || !density_previous)
    {
        fprintf(stderr, "cannot allocate data\n");
        return (0);
    }

    return (1);
}

void FluidField::freeData()
{
    // for gird based system
    if (u)
        free(u);
    if (v)
        free(v);
    if (u_previous)
        free(u_previous);
    if (v_previous)
        free(v_previous);
    if (density)
        free(density);
    if (density_previous)
        free(density_previous);
}

void FluidField::reset()
{
    // for gird based system
    int i, size = (N + 2) * (N + 2);

    for (i = 0; i < size; i++)
    {
        u[i] = v[i] = u_previous[i] = v_previous[i] = density[i] = density_previous[i] = 0.0f;
    }
}