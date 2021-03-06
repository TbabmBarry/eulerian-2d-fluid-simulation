// ParticleToy.cpp : Defines the entry point for the console application.
//

#include "Particle.h"
#include "DragForce.h"
#include "GravityForce.h"
#include "SpringForce.h"
#include "AngularSpring.h"
#include "RodConstraint.h"
#include "CircularWireConstraint.h"
#include "System.h"
#include "EulerSolver.h"
#include "imageio.h"
#include "GravityForce.h"
#include "ExternalForce.h"
#include "MidpointSolver.h"
#include "RungeSovler.h"
#include "Fluid.h"

#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <GL/glut.h>
#include <gfx/vec2.h>

#include "unistd.h"
#include "Mode.h"

/* macros */

#define IX(i, j) ((i) + (grid_N + 2) * (j))
/* external definitions (from solver) */
// extern void simulation_step( std::vector<Particle*> pVector, float dt );
// extern void dens_step ( int N, float * x, float * x0, float * u, float * v, float diff, float dt );
// extern void vel_step ( int N, float * u, float * v, float * u0, float * v0, float visc, float dt );

/* global variables */
static bool sys_type = false; // false = particle, true = fluid
static bool mouse_inrigid = false;
// for particle based system
static int N;
static float dt, d;
static int dsim;
static int dump_frames;
static int frame_number;

static System *sys;
static Mode *mode;
static ExternalForce *mouseForce;

static int win_id;
static int win_x, win_y;
static int mouse_down[3];
static int mouse_release[3];
static int mouse_shiftclick[3];
static int omx, omy, mx, my;
static int hmx, hmy;
static float external_force;
static int mode_index;

// for grid based system
static int grid_N;
static float diff, visc;
static float force, source;
static int dvel;

static float *u, *v, *u_prev, *v_prev;
static float *dens, *dens_prev;
static int gomx, gomy, gmx, gmy;
Fluid *fluid = new Fluid();
/*
----------------------------------------------------------------------
free/clear/allocate simulation data
----------------------------------------------------------------------
*/

static void free_data(void)
{
	// for particle based system
	sys->free();

	// for gird based system
	if (u)
		free(u);
	if (v)
		free(v);
	if (u_prev)
		free(u_prev);
	if (v_prev)
		free(v_prev);
	if (dens)
		free(dens);
	if (dens_prev)
		free(dens_prev);
}

static void clear_data(void)
{
	// for particle based system
	sys->reset();

	// for gird based system
	int i, size = (grid_N + 2) * (grid_N + 2);

	for (i = 0; i < size; i++)
	{
		u[i] = v[i] = u_prev[i] = v_prev[i] = dens[i] = dens_prev[i] = 0.0f;
	}
}

static void init_system(void)
{
	const double dist = 0.2;
	const Vec2f center(0.0, 0.0);
	const Vec2f offset(dist, 0.0);
	sys = new System(new EulerSolver(EulerSolver::SEMI), fluid);
	mode = new Mode();
}

static int allocate_data(void)
{
	int size = (grid_N + 2) * (grid_N + 2);

	u = (float *)malloc(size * sizeof(float));
	v = (float *)malloc(size * sizeof(float));
	u_prev = (float *)malloc(size * sizeof(float));
	v_prev = (float *)malloc(size * sizeof(float));
	dens = (float *)malloc(size * sizeof(float));
	dens_prev = (float *)malloc(size * sizeof(float));

	if (!u || !v || !u_prev || !v_prev || !dens || !dens_prev)
	{
		fprintf(stderr, "cannot allocate data\n");
		return (0);
	}

	return (1);
}
/*
----------------------------------------------------------------------
OpenGL specific drawing routines
----------------------------------------------------------------------
*/

static void pre_display(void)
{
	glViewport(0, 0, win_x, win_y);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(-1.0, 1.0, -1.0, 1.0);
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT);
}

static void post_display(void)
{
	// Write frames if necessary.
	if (dump_frames)
	{
		const int FRAME_INTERVAL = 4;
		if ((frame_number % FRAME_INTERVAL) == 0)
		{
			const unsigned int w = glutGet(GLUT_WINDOW_WIDTH);
			const unsigned int h = glutGet(GLUT_WINDOW_HEIGHT);
			unsigned char *buffer = (unsigned char *)malloc(w * h * 4 * sizeof(unsigned char));
			if (!buffer)
				exit(-1);
			// glRasterPos2i(0, 0);
			glReadPixels(0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, buffer);
			static char filename[80];
			sprintf(filename, "../snapshots/img%.5i.png", frame_number / FRAME_INTERVAL);
			printf("Dumped %s.\n", filename);
			saveImageRGBA(filename, buffer, w, h); //buffer img-->filename(xxx.png)

			free(buffer);
		}
	}
	frame_number++;

	glutSwapBuffers();
}

static void draw_velocity(void)
{
	int i, j;
	float x, y, h;

	h = 2.0f / grid_N;

	glColor3f(1.0f, 1.0f, 1.0f);
	glLineWidth(1.0f);

	glBegin(GL_LINES);

	for (i = 1; i <= grid_N; i++)
	{
		x = (i - 0.5f) * h;
		x = -1 + x;
		for (j = 1; j <= grid_N; j++)
		{
			y = (j - 0.5f) * h;
			y = -1 + y;
			glVertex2f(x, y);
			glVertex2f(x + u[IX(i, j)], y + v[IX(i, j)]);
		}
	}

	glEnd();
}

static void draw_density(void)
{
	int i, j;
	float x, y, h, d00, d01, d10, d11;

	h = 2.0f / grid_N;

	glBegin(GL_QUADS);

	for (i = 0; i <= grid_N; i++)
	{
		x = (i - 0.5f) * h;
		x = -1 + x;
		for (j = 0; j <= grid_N; j++)
		{
			y = (j - 0.5f) * h;
			y = -1 + y;

			d00 = dens[IX(i, j)];
			d01 = dens[IX(i, j + 1)];
			d10 = dens[IX(i + 1, j)];
			d11 = dens[IX(i + 1, j + 1)];

			glColor3f(d00, d00, d00);
			glVertex2f(x, y);
			glColor3f(d10, d10, d10);
			glVertex2f(x + h, y);
			glColor3f(d11, d11, d11);
			glVertex2f(x + h, y + h);
			glColor3f(d01, d01, d01);
			glVertex2f(x, y + h);
			// cout << "draw" << x << y << endl;
		}
	}

	glEnd();
}

/*
----------------------------------------------------------------------
relates mouse movements to particle toy construction
----------------------------------------------------------------------
*/

static void get_from_UI_particle()
{
	int i, j;
	int hi, hj;
	if (!mouse_down[0] && !mouse_down[2] && !mouse_release[0] && !mouse_shiftclick[0] && !mouse_shiftclick[2])
		return;

	i = (int)((mx / (float)win_x) * N);			  //detect mouse x-axis not outside a range?
	j = (int)(((win_y - my) / (float)win_y) * N); //detect mouse y-axis not outside a range?
	//my<0-->j>N;my-->winy or my>winy =>j<1

	if (i < 1 || i > N || j < 1 || j > N)
		return;

	if (mouse_down[0])
	{
	}

	if (mouse_down[2])
	{
	}

	hi = (int)((hmx / (float)win_x) * N);
	hj = (int)(((win_y - hmy) / (float)win_y) * N);

	if (mouse_release[0])
	{
	}

	omx = mx;
	omy = my;
}

static void get_from_UI_grid(float *d, float *u, float *v)
{
	int i, j, size = (grid_N + 2) * (grid_N + 2);

	for (i = 0; i < size; i++)
	{
		u[i] = v[i] = d[i] = 0.0f;
	}

	if (!mouse_down[0] && !mouse_down[2])
		return;

	// cout<< "in_UI" <<endl;
	// cout << "gomx " << gomx <<"gomy" << gomy << endl;
	i = (int)((gmx / (float)win_x) * grid_N + 1);
	j = (int)(((win_y - gmy) / (float)win_y) * grid_N + 1);

	// cout << "i " << i <<"j" << j << endl;
	if (i < 1 || i > grid_N || j < 1 || j > grid_N)
		return;

	if (mouse_down[0] && mouse_inrigid == false)
	{
		u[IX(i, j)] = force * (gmx - gomx);
		v[IX(i, j)] = force * (gomy - gmy);
	}

	if (mouse_down[2] && mouse_inrigid == false)
	{
		d[IX(i, j)] = source;
	}

	gomx = gmx;
	gomy = gmy;

	return;
}

static void remap_GUI()
{
	for (int i = 0; i < sys->particles.size(); i++)
	{
		sys->particles[i]->m_Position[0] = sys->particles[i]->m_ConstructPos[0];
		sys->particles[i]->m_Position[1] = sys->particles[i]->m_ConstructPos[1];
	}
}

/*
----------------------------------------------------------------------
GLUT callback routines
----------------------------------------------------------------------
*/

static void key_func(unsigned char key, int x, int y)
{
	switch (key)
	{
	case '1':
		sys_type = false;
		clear_data();
		if (dsim)
			dsim = !dsim;
		init_system();
		sys->dt = 0.001;
		external_force = 0.1f;
		mode_index = 7;
		sys->solver = new EulerSolver(EulerSolver::SEMI);
		mode->FluidCloth(sys);
		break;

	case '2':
		sys_type = false;
		clear_data();
		if (dsim)
			dsim = !dsim;
		sys->dt = 0.001;
		external_force = 0.1f;
		init_system();
		sys->solver = new EulerSolver(EulerSolver::SEMI);
		mode->RigidBody(sys, fluid);
		break;

	case '3':
		sys_type = false;
		clear_data();
		if (dsim)
			dsim = !dsim;
		sys->dt = 0.001;
		external_force = 0.1f;
		init_system();
		sys->solver = new EulerSolver(EulerSolver::SEMI);
		mode->RigidBodyCollision(sys, fluid);
		break;

	case '4':
		sys_type = false;
		clear_data();
		if (dsim)
			dsim = !dsim;
		sys->dt = 0.001;
		external_force = 0.1f;
		init_system();
		sys->solver = new EulerSolver(EulerSolver::SEMI);
		mode->Fix(sys, fluid);
		break;

	case '5':
		sys_type = false;
		clear_data();
		if (dsim)
			dsim = !dsim;
		sys->dt = 0.001;
		external_force = 0.1f;
		init_system();
		sys->solver = new EulerSolver(EulerSolver::SEMI);
		mode->Move(sys, fluid);
		break;

	case 'c':
	case 'C':
		clear_data();
		break;

	case 'd':
	case 'D':
		free_data();
		dump_frames = !dump_frames; //dump-frames initially=0,not save img.
		exit(0);
		//post_display
		break;

	case 'v':
	case 'V':
		dvel = !dvel;
		break;

	case 'q':
	case 'Q':
		free_data();
		exit(0);
		break;

	case ' ':
		dsim = !dsim;
		if (dsim)
			sys->reset();
		break;

	case 'p':
	case 'P':
		sleep(100);
	}
}

static void mouse_func(int button, int state, int x, int y)
{

	if (mouse_down[button])
		mouse_release[button] = state == GLUT_UP;
	if (mouse_down[button])
		mouse_shiftclick[button] = glutGetModifiers() == GLUT_ACTIVE_SHIFT;
	mouse_down[button] = state == GLUT_DOWN;

	// cout<< mouse_inrigid<<endl;

	if (state == GLUT_UP)
	{
		mouseForce->setActive(false);
		mouse_inrigid = false;
	}
	else
	{
		float mouse_x = (x - int(win_x / 2));
		float mouse_y = (int(win_y / 2) - y);
		mouse_x = mouse_x / (win_x / 2);
		mouse_y = mouse_y / (win_y / 2);
		// cout << "mouse_func: " << mouse_x << " " << mouse_y << endl;
		// cout << "x,y: " << x << " " << y << endl;
		Particle *closestParticle;
		for (int i = 0; i < sys->rigidbodies.size(); i++)
		{
			Vector2f position = sys->rigidbodies[i]->x;
			float r = sys->rigidbodies[i]->dimension;
			r = r / 2 * 1.1;

			double distance = sqrt((mouse_x - position[0]) * (mouse_x - position[0]) + (mouse_y - position[1]) * (mouse_y - position[1]));
			if (distance < r)
			{
				// cout<<"r "<<r<<" "<<"distance "<<distance<<endl;
				mouse_inrigid = true;
				closestParticle = sys->rigidbodies[i];
			}
		}
		mouseForce = new ExternalForce({closestParticle}, external_force, Vector2f(0.0f, 0.0f));
		sys->addRigidForce(mouseForce);
	}

	gomx = x;
	gomy = y;
	gmx = x;
	gmy = y;
}

static void motion_func(int x, int y)
{
	float fx = x - (win_x / 2);
	float fy = (win_y / 2) - y;

	// cout << "mx,my"<< fx << " " <<fy << endl;

	fx = fx / (win_x / 2);
	fy = fy / (win_y / 2);

	// cout << "after fx,fy"<< fx << " " <<fy << endl;

	if (mouse_inrigid == true)
	{
		Vector2f position = mouseForce->particles[0]->x;
		mouseForce->direction = 10000.0f * Vector2f(fx - position[0], fy - position[1]);
	}

	gmx = x;
	gmy = y;
	// cout << "gmx " << gmx <<"gmy" << gmy << endl;
	// cout << "mouseForce" << mouseForce->direction[0] << " " << mouseForce->direction[1] << endl;;
}

static void reshape_func(int width, int height)
{
	glutSetWindow(win_id);
	glutReshapeWindow(width, height);

	win_x = width;
	win_y = height;
}

static void idle_func(void)
{
	if (sys_type == false)
	{
		fluid->rigidbodies = sys->rigidbodies;
		if (dsim)
			sys->simulationStep();
		else
		{
			get_from_UI_particle();
			remap_GUI();
		}
		get_from_UI_grid(dens_prev, u_prev, v_prev);
		fluid->vel_step(grid_N, u, v, u_prev, v_prev, visc, dt);
		fluid->dens_step(grid_N, dens, dens_prev, u, v, diff, dt);
		fluid->vorticity_confinement(grid_N, dt, dens_prev, u, v, u_prev, v_prev);
	}
	else if (sys_type == true)
	{
		fluid->rigidbodies = sys->rigidbodies;
		get_from_UI_grid(dens_prev, u_prev, v_prev);
		fluid->vel_step(grid_N, u, v, u_prev, v_prev, visc, dt);
		fluid->dens_step(grid_N, dens, dens_prev, u, v, diff, dt);
		fluid->vorticity_confinement(grid_N, dt, dens_prev, u, v, u_prev, v_prev);
	}

	glutSetWindow(win_id);
	glutPostRedisplay();
}

static void display_func(void)
{
	if (sys_type == false)
	{
		pre_display();
		if (dvel)
			draw_velocity();
		else
			draw_density();
		sys->drawSystem();
		post_display(); //frame,img
	}
	else if (sys_type == true)
	{
		pre_display();
		if (dvel)
			draw_velocity();
		else
			draw_density();
		post_display();
	}
}

/*
----------------------------------------------------------------------
open_glut_window --- open a glut compatible window and set callbacks
----------------------------------------------------------------------
*/

static void open_glut_window(void)
{
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE); //init display mode:show colors+double buffer window

	glutInitWindowPosition(0, 0);
	glutInitWindowSize(win_x, win_y);
	win_id = glutCreateWindow("Particletoys!");

	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT);
	glutSwapBuffers();
	glClear(GL_COLOR_BUFFER_BIT);
	glutSwapBuffers();

	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_POLYGON_SMOOTH);

	pre_display();

	glutKeyboardFunc(key_func);
	glutMouseFunc(mouse_func);
	glutMotionFunc(motion_func);
	glutReshapeFunc(reshape_func);
	glutIdleFunc(idle_func);
	glutDisplayFunc(display_func);
}

/*
----------------------------------------------------------------------
main --- main routine
----------------------------------------------------------------------
*/

int main(int argc, char **argv)
{
	glutInit(&argc, argv);

	if (argc == 1)
	{

		// for particle based system
		N = 64;
		dt = 0.1f;
		d = 5.f;
		fprintf(stderr, "Using defaults : N=%d dt=%g d=%g\n",
				N, dt, d);

		// for grid based system
		grid_N = 128; //number of grid
		diff = 0.0f;
		visc = 0.0f;
		force = 5.0f;
		source = 100.0f;
		fprintf(stderr, "Using defaults : grid_N=%d dt=%g diff=%g visc=%g force = %g source=%g\n",
				grid_N, dt, diff, visc, force, source);
	}
	else
	{
		N = atoi(argv[1]);
		dt = atof(argv[2]);
		d = atof(argv[3]);
	}

	printf("\n\nHow to use this application:\n");
	printf("\t Toggle construction/simulation display with the spacebar key\n");
	printf("\t Dump frames by pressing the 'd' key\n");
	printf("\t Quit by pressing the 'q' key\n");
	printf("\t Switch view between density and verlocity by pressing the 'v' key\n");
	printf("\t key '1' for cloth on fluid\n");
	printf("\t key '2' for rigid body + fluid\n");
	printf("\t key '3' for rigid body collision + fluid\n");
	printf("\t key '4' for fix boundary + fluid\n");
	printf("\t key '5' for move boundary + fluid\n");

	// for particle based system
	dsim = 0;
	dump_frames = 0;
	frame_number = 0;
	init_system();

	// for grid based system
	dvel = 0;

	if (!allocate_data())
		exit(1);
	clear_data();

	win_x = 1024;
	win_y = 1024;
	open_glut_window(); //open window-->pre-display;glutKeyboardFunc ( key_func );glutDisplayFunc ( display_func );
	//displayfunc-->post-display i.e img frame...

	glutMainLoop();

	exit(0);
}
