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

// static Particle *pList;
// static std::vector<Particle*> pVector;
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

// static SpringForce * delete_this_dummy_spring = NULL;
// static RodConstraint * delete_this_dummy_rod = NULL;
// static CircularWireConstraint * delete_this_dummy_wire = NULL;


/*
----------------------------------------------------------------------
free/clear/allocate simulation data
----------------------------------------------------------------------
*/

static void free_data ( void )
{
	// pVector.clear();
	// if (delete_this_dummy_rod) {
	// 	delete delete_this_dummy_rod;
	// 	delete_this_dummy_rod = NULL;
	// }
	// if (delete_this_dummy_spring) {
	// 	delete delete_this_dummy_spring;
	// 	delete_this_dummy_spring = NULL;
	// }
	// if (delete_this_dummy_wire) {
	// 	delete delete_this_dummy_wire;
	// 	delete_this_dummy_wire = NULL;
	// }
	sys->free ();
}

static void clear_data ( void )
{
	// int ii, size = pVector.size();

	// for(ii=0; ii<size; ii++){
	// 	pVector[ii]->reset();
	// }
	sys->reset();
}

static void init_system(void)
{
	const double dist = 0.2;
	const Vec2f center(0.0, 0.0);
	const Vec2f offset(dist, 0.0);
	// const Vec2f offset(0.0, dist);
	sys = new System(new EulerSolver(EulerSolver::SEMI));
	mode = new Mode();
	// Create three particlereConstraint(sys->particles[0], center, dist));
}

/*
----------------------------------------------------------------------
OpenGL specific drawing routines
----------------------------------------------------------------------
*/

static void pre_display ( void )
{
	glViewport ( 0, 0, win_x, win_y );
	// printf("hello"); recursively running
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
	// int size = pVector.size();

	// for(int ii=0; ii< size; ii++)
	// {
	// 	pVector[ii]->draw();
	// }
	sys->drawParticles();
}

static void draw_forces ( void )
{
	// change this to iteration over full set
	// if (delete_this_dummy_spring)
	// 	delete_this_dummy_spring->draw();
	sys->drawForces();
}

static void draw_constraints ( void )
{
	// change this to iteration over full set
	// if (delete_this_dummy_rod)
	// 	delete_this_dummy_rod->draw();
	// if (delete_this_dummy_wire)
	// 	delete_this_dummy_wire->draw();
	sys->drawConstraints();
}

// static System* Hair() {
//     System* sys = new System(new EulerSolver(EulerSolver::SEMI));

//     const int num_particles = 100;
//     const float deltay = 3.0f/num_particles;
//     const int numHairs = 8;
// 	const float ks = 50.0f;
//     const float kd = 1.0f;

//     for (int i = 0; i < numHairs; i++) {
//         // Initialize particles
//         for (int j = 0; j < num_particles; j++) {
//             sys->addParticle(new Particle(Vec2f(-0.5f + 0.03f*i, 0.5f - j * deltay), 0.2f, i * num_particles + j));
//         }

//         // for (int j = 0; j < num_particles - 1; j++) {
//         //     sys->addForce(new SpringForce(sys->particles[i * num_particles + j],
//         //                                   sys->particles[i * num_particles + j + 1],
//         //                                   deltay, ks, kd));
//         // }
//         for (int j = 2; j < num_particles - 2; j++) {
//             sys->addForce(new AngularSpring(sys->particles[i * num_particles + j],
// 											sys->particles[i * num_particles + j + 1],
// 											sys->particles[i * num_particles + j + 2],
// 											120, ks, kd));
//         }

//         float radius = 0.05f;
//         sys->addConstraint(new CircularWireConstraint(sys->particles[i * num_particles],
//                                                       Vec2f(0.0f,0.0f) + Vec2f(-radius, 0.f),//暂时定义为(0,0)+(-r,0)
//                                                       radius));
//     }
//     // Add gravity and drag to all particles
//     sys->addForce(new GravityForce(sys->particles, Vec2f(0, -9.81f)));
//     return sys;
// }



/*
----------------------------------------------------------------------
relates mouse movements to particle toy construction
----------------------------------------------------------------------
*/

static void get_from_UI ()
{
	int i, j;
	// int size, flag;
	int hi, hj;
	// float x, y;
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
		init_system();
		// free_data();
		sys->dt=0.1;
		mode->Spring(sys);
		break;

	case '2':
		init_system();
		sys->dt=0.001;
		mode->SpringRod(sys);
		break;		
	
	case '3':
		init_system();
		sys->dt=0.001;
		mode->SpringCircular(sys);
		break;
	
	case '4':
		init_system();
		sys->dt=0.001;
		mode->Rod(sys);
		break;

	case '5':
		init_system();
		sys->dt=0.01;
		mode->Gravity(sys);
		break;

	case '6':
		init_system();
		mode->test(sys);
		break;
	
	case '7':
		init_system();
		sys->dt=0.01;
		mode->hair(sys);
		break;

	case '8':
		init_system();
		sys->dt=0.001;
		mode->cloth(sys);
		break;
	
	case 'c':
	case 'C':
		clear_data ();
		break;

	case 'd':
	case 'D':
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
		// std::cout << "GLUT_UP   x" << x << "   y" << y << std::endl;	
	} else {

		GLdouble modelMatrix[16];
        glGetDoublev(GL_MODELVIEW_MATRIX, modelMatrix);
        GLdouble projectionMatrix[16];
        glGetDoublev(GL_PROJECTION_MATRIX, projectionMatrix);
        GLint viewMatrix[4];
        glGetIntegerv(GL_VIEWPORT, viewMatrix);

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

		mouseForce = new ExternalForce({closestParticle}, Vec2f(0.0f,0.0f));
		sys->addForce(mouseForce);
		
	}
}

static void motion_func ( int x, int y )
{
	mx = x - int(win_x/2);
	my = int(win_y/2) - y;

	Vec2f position = mouseForce->particles[0]->m_Position;
	//mode 3 8 use 5.0, others 0.005 
	mouseForce->direction = 5.0f * Vec2f(mx-position[0]*(win_x/2), my-position[1]*(win_y/2));
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

	// sleep(0.5);
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

	printf ( "\n\nHow to use this application:\n\n" );
	printf ( "\t Toggle construction/simulation display with the spacebar key\n" );
	printf ( "\t Dump frames by pressing the 'd' key\n" );
	printf ( "\t Quit by pressing the 'q' key\n" );
	printf ( "\t key '1' for Spring force\n" );
	printf ( "\t key '2' for Spring force + Rod Constraint\n" );
	printf ( "\t key '3' for Spring force + Circular Wire Constraint\n" );
	printf ( "\t key '4' for Circular Wire Constraint + Spring force + Rod Constraint\n" );
	printf ( "\t key '5' for Circular Wire Constraint + Spring force + Rod Constraint + Gravity force\n" );
	printf ( "\t key '6' for test\n");
	printf ( "\t key '7' for hair\n");
	printf ( "\t key '8' for cloth\n");

	dsim = 0;
	dump_frames = 0;
	frame_number = 0;
	init_system();
	// sys = Cloth();
	//+3 new Particles
	//-->springforce;-->circularwireconstraint;-->rodconstraint
	
	win_x = 1024;
	win_y = 1024;
	open_glut_window ();//open window-->pre-display;glutKeyboardFunc ( key_func );glutDisplayFunc ( display_func );
	//displayfunc-->post-display i.e img frame...
	
	glutMainLoop ();

	exit ( 0 );
}

