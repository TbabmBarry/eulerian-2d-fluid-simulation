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

#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <GL/glut.h>
#include <gfx/vec2.h>

#include "unistd.h"
#include "Mode.h"

/* macros */

/* external definitions (from solver) */
// extern void simulation_step( std::vector<Particle*> pVector, float dt );

/* global variables */

static int N;
static float dt, d;
static int dsim;
static int dump_frames;
static int frame_number;

static System* sys;
static Mode* mode;
static ExternalForce* mouseForce;


static int win_id;
static int win_x, win_y;
static int mouse_down[3];
static int mouse_release[3];
static int mouse_shiftclick[3];
static int omx, omy, mx, my;
static int hmx, hmy;
static float external_force;
static int mode_index;


/*
----------------------------------------------------------------------
free/clear/allocate simulation data
----------------------------------------------------------------------
*/

static void free_data ( void )
{
	sys->free ();
}

static void clear_data ( void )
{
	sys->reset();
}

static void init_system(void)
{
	const double dist = 0.2;
	const Vec2f center(0.0, 0.0);
	const Vec2f offset(dist, 0.0);
	sys = new System(new EulerSolver(EulerSolver::SEMI));
	mode = new Mode();
}

/*
----------------------------------------------------------------------
OpenGL specific drawing routines
----------------------------------------------------------------------
*/

static void pre_display ( void )
{
	glViewport ( 0, 0, win_x, win_y );
	glMatrixMode ( GL_PROJECTION );
	glLoadIdentity ();
	gluOrtho2D ( -1.0, 1.0, -1.0, 1.0 );
	glClearColor ( 0.0f, 0.0f, 0.0f, 1.0f );
	glClear ( GL_COLOR_BUFFER_BIT );
}

static void post_display ( void )
{
	// Write frames if necessary.
	if (dump_frames) {
		const int FRAME_INTERVAL = 4;
		if ((frame_number % FRAME_INTERVAL) == 0) {
			const unsigned int w = glutGet(GLUT_WINDOW_WIDTH);
			const unsigned int h = glutGet(GLUT_WINDOW_HEIGHT);
			unsigned char * buffer = (unsigned char *) malloc(w * h * 4 * sizeof(unsigned char));
			if (!buffer)
				exit(-1);
			// glRasterPos2i(0, 0);
			glReadPixels(0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, buffer);
			static char filename[80];
			sprintf(filename, "../snapshots/img%.5i.png", frame_number / FRAME_INTERVAL);
			printf("Dumped %s.\n", filename);
			saveImageRGBA(filename, buffer, w, h);//buffer img-->filename(xxx.png)
			
			free(buffer);
		}
	}
	frame_number++;
	
	glutSwapBuffers ();
}

static void draw_particles ( void )
{
	sys->drawParticles();
}

static void draw_forces ( void )
{
	sys->drawForces();
}

static void draw_constraints ( void )
{
	sys->drawConstraints();
}


/*
----------------------------------------------------------------------
relates mouse movements to particle toy construction
----------------------------------------------------------------------
*/

static void get_from_UI ()
{
	int i, j;
	int hi, hj;
	if ( !mouse_down[0] && !mouse_down[2] && !mouse_release[0] 
	&& !mouse_shiftclick[0] && !mouse_shiftclick[2] ) return;

	i = (int)((       mx /(float)win_x)*N);//detect mouse x-axis not outside a range?
	j = (int)(((win_y-my)/(float)win_y)*N);//detect mouse y-axis not outside a range?
	//my<0-->j>N;my-->winy or my>winy =>j<1

	if ( i<1 || i>N || j<1 || j>N ) return;

	if ( mouse_down[0] ) {

	}

	if ( mouse_down[2] ) {
	}

	hi = (int)((       hmx /(float)win_x)*N);
	hj = (int)(((win_y-hmy)/(float)win_y)*N);

	if( mouse_release[0] ) {
	}

	omx = mx;
	omy = my;
}

static void remap_GUI()
{
	for(int i=0; i < sys->particles.size(); i++)
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

static void key_func ( unsigned char key, int x, int y )
{
	switch ( key )
	{
	case '1':
		if (dsim)
			dsim = !dsim;
		init_system();
		sys->dt=0.05;
		external_force = 0.008f;
		mode_index = 1;
		mode->Spring(sys);
		break;

	case '2':
		if (dsim)
			dsim = !dsim;
		init_system();
		sys->dt=0.01;
		external_force = 0.1f;
		mode_index = 2;
		mode->Gravity(sys);
		break;

	case '3':
		if (dsim)
			dsim = !dsim;
		init_system();
		sys->dt=0.05;
		external_force = 0.001f;
		mode_index = 3;
		mode->SpringRod(sys);
		break;		
	
	case '4':
		if (dsim)
			dsim = !dsim;
		init_system();
		sys->dt=0.001;
		external_force = 0.065f;
		sys->solver = new RungeSovler();
		mode_index = 4; 
		mode->CircularGravity(sys);
		break;
	
	case '5':
		if (dsim)
			dsim = !dsim;
		init_system();
		sys->dt=0.001;
		external_force = 1.0f;
		sys->solver = new RungeSovler(); 
		mode_index = 5;
		mode->CircularGravityRod(sys);
		break;
	
	case '6':
		if (dsim)
			dsim = !dsim;
		init_system();
		sys->dt=0.01;
		external_force = 0.05f;
		sys->solver = new EulerSolver(EulerSolver::SEMI); 
		mode_index = 6;
		mode->hair(sys);
		break;

	case '7':
		if (dsim)
			dsim = !dsim;
		init_system();
		sys->dt=0.001;
		external_force = 0.1f;
		mode_index = 7;
		sys->solver = new RungeSovler();
		mode->Cloth(sys);
		break;

	case '8':
		if (dsim)
			dsim = !dsim;
		init_system();
		sys->dt=0.001;
		external_force = 0.1f;
		mode_index = 7;
		sys->solver = new RungeSovler();
		mode->CircularCloth(sys);
		break;
	
	case 'w':
	case 'W':
		if (dsim)
			dsim = !dsim;
		sys->reset();	
		printf("Using SEMI Euler\n");
		sys->solver = new EulerSolver(EulerSolver::SEMI); 
		break;

	case 'e':
	case 'E':
		if (dsim)
			dsim = !dsim;
		sys->reset();
		if (mode_index != 6) {
			printf("Using Explicit Euler\n");
        	sys->solver = new EulerSolver(EulerSolver::EXPLICIT);
		} else {
			printf("Only SEMI\n");
		}
		
		break;

	case 'r':
	case 'R':
		if (dsim)
			dsim = !dsim;
		sys->reset();
		if (mode_index != 6) {
			printf("Using Implicit Euler\n");
        	sys->solver = new EulerSolver(EulerSolver::IMPLICIT);
		} else {
			printf("Only SEMI\n");
		}
		 
		break;

	case 't':
	case 'T':
		if (dsim)
			dsim = !dsim;
		sys->reset();
		if (mode_index != 6) {
			printf("Using Midpoint Solver\n");
        	sys->solver = new MidpointSolver();
		} else {
			printf("Only SEMI\n");
		}
		 
		break;

	case 'y':
	case 'Y':
		if (dsim)
			dsim = !dsim;
		sys->reset();
		if (mode_index != 6) {
			printf("Using Runge4 Solver\n");
       	 	sys->solver = new RungeSovler();
		} else {
			printf("Only SEMI\n");
		}				 
		break;

	case 'c':
	case 'C':
		free_data();
		clear_data ();
		break;

	case 'd':
	case 'D':
		free_data();
		dump_frames = !dump_frames;//dump-frames initially=0,not save img.
		//post_display
		break;

	case 'q':
	case 'Q':
		free_data ();
		exit ( 0 );
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

static void mouse_func ( int button, int state, int x, int y )
{
	omx = mx = x;
	omx = my = y;
	if(!mouse_down[0]){hmx=x; hmy=y;}
	if(mouse_down[button]) mouse_release[button] = state == GLUT_UP;
	if(mouse_down[button]) mouse_shiftclick[button] = glutGetModifiers()==GLUT_ACTIVE_SHIFT;
	mouse_down[button] = state == GLUT_DOWN;

	if (state == GLUT_UP){
		mouseForce->setActive(false);
	} else {

		int mouse_x = x - int(win_x/2);
		int mouse_y = int(win_y/2) - y;
		Particle *closestParticle;
		double closestDist = 100000;
		for (int i = 0; i < sys->particles.size(); i++) {
			Vec2f position = sys->particles[i]->m_Position;
            double distance = sqrt(pow(mouse_x - (position[0]*(win_x/2)),2) + pow(mouse_y - (position[1]*(win_y/2)),2));
			if (distance < closestDist) {
                closestDist = distance;
                closestParticle = sys->particles[i];
            }
		}

		mouseForce = new ExternalForce({closestParticle}, external_force, Vec2f(0.0f,0.0f));
		sys->addForce(mouseForce);
		
	}
}

static void motion_func ( int x, int y )
{
	mx = x - int(win_x/2);
	my = int(win_y/2) - y;

	Vec2f position = mouseForce->particles[0]->m_Position;
	mouseForce->direction = 3.0f * Vec2f(mx-position[0]*(win_x/2), my-position[1]*(win_y/2));
}

static void reshape_func ( int width, int height )
{
	glutSetWindow ( win_id );
	glutReshapeWindow ( width, height );

	win_x = width;
	win_y = height;
}

static void idle_func ( void )
{
	if ( dsim ) sys->simulationStep();
	else        {get_from_UI();remap_GUI();}

	glutSetWindow ( win_id );
	glutPostRedisplay ();
}

static void display_func ( void )
{
	pre_display ();
	draw_particles();
	draw_forces();
	draw_constraints();
	post_display ();//frame,img
}


/*
----------------------------------------------------------------------
open_glut_window --- open a glut compatible window and set callbacks
----------------------------------------------------------------------
*/

static void open_glut_window ( void )
{
	glutInitDisplayMode ( GLUT_RGBA | GLUT_DOUBLE );//init display mode:show colors+double buffer window

	glutInitWindowPosition ( 0, 0 );
	glutInitWindowSize ( win_x, win_y );
	win_id = glutCreateWindow ( "Particletoys!" );

	glClearColor ( 0.0f, 0.0f, 0.0f, 1.0f );
	glClear ( GL_COLOR_BUFFER_BIT );
	glutSwapBuffers ();
	glClear ( GL_COLOR_BUFFER_BIT );
	glutSwapBuffers ();

	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_POLYGON_SMOOTH);

	pre_display ();

	glutKeyboardFunc ( key_func );
	glutMouseFunc ( mouse_func );
	glutMotionFunc ( motion_func );
	glutReshapeFunc ( reshape_func );
	glutIdleFunc ( idle_func );
	glutDisplayFunc ( display_func );
	
}


/*
----------------------------------------------------------------------
main --- main routine
----------------------------------------------------------------------
*/

int main ( int argc, char ** argv )
{
	glutInit ( &argc, argv );

	if ( argc == 1 ) {
		N = 64;
		dt = 0.1f;
		d = 5.f;
		fprintf ( stderr, "Using defaults : N=%d dt=%g d=%g\n",
			N, dt, d );
	} else {
		N = atoi(argv[1]);
		dt = atof(argv[2]);
		d = atof(argv[3]);
	}

	printf ( "\n\nHow to use this application:\n" );
	printf ( "\t Toggle construction/simulation display with the spacebar key\n" );
	printf ( "\t Dump frames by pressing the 'd' key\n" );
	printf ( "\t Quit by pressing the 'q' key\n" );
	printf ( "\t key '1' for Spring force (default: Semi)\n" );
	printf ( "\t key '2' for Gravity (default: Semi)\n" );
	printf ( "\t key '3' for Spring force + Rod Constraint (default: Semi)\n" );
	printf ( "\t key '4' for Circular Wire Constraint + Gravity (default: Runge4)\n" );
	printf ( "\t key '5' for Circular Wire Constraint + Spring force + Rod Constraint + Gravity (default: Runge4)\n" );
	printf ( "\t key '6' for hair (Only Semi)\n");
	printf ( "\t key '7' for cloth (default: Implicit)\n");
	printf ( "\t key 'w' turn to Semi Euler solver\n");
	printf ( "\t key 'e' turn to Explicit Euler solver\n");
	printf ( "\t key 'r' turn to Implicit Euler solver\n");
	printf ( "\t key 't' turn to Midpoint solver\n");
	printf ( "\t key 'y' turn to Runge4 solver\n");

	dsim = 0;
	dump_frames = 0;
	frame_number = 0;
	init_system();
	
	win_x = 1024;
	win_y = 1024;
	open_glut_window ();//open window-->pre-display;glutKeyboardFunc ( key_func );glutDisplayFunc ( display_func );
	//displayfunc-->post-display i.e img frame...
	
	glutMainLoop ();

	exit ( 0 );
}

