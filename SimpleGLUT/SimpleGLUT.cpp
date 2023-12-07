#include "stdafx.h"

// standard
#include <assert.h>
#include <math.h>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

// glut
#include "stdafx.h"
#include <GL/glut.h>

#include "mylib.hpp"



//================================
// global variables
//================================
// screen size
int g_screenWidth  = 0;
int g_screenHeight = 0;


// frame index
int g_frameIndex = 0;

// angle for rotation
int g_angle = 0;

float x_angle = 0;
float y_angle = 1.0;
float z_angle = 0;

float x_trans = 0;
float y_trans = 0;
float z_trans = 0;

Matrix4f model;

GLfloat mat[16];

float ground = -5;

int timecount = 0;

vector<Vector3f> trace;

//a series of rigid bodys
vector<rigidBody> rigidbodys;

//================================
// init
//================================
void init( void ) {
	// init something before main loop...

	rigidBody body1;
	body1.init();
		
	rigidBody body2;
	body2.init();	
		
	rigidBody body3;
	body3.init();
		
	rigidbodys.push_back(body1);
	rigidbodys.push_back(body2);
	rigidbodys.push_back(body3);
}

//================================
// update
//================================
void update( void ) {
	// do something before rendering...

	//Check collide for each pair
	

	//Update motion for each object
	for (auto& obj : rigidbodys) {
		obj.initAcc();
	}
	universalGravitation(rigidbodys);
	for (auto& obj : rigidbodys) {
		obj.move();
		trace.push_back(obj.transform.location);
	}
}


//================================
// render
//================================
void render( void ) {
	// clear buffer
	glClearColor (0.035, 0.0, 0.136, 0.0);
	glClearDepth (1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);	
	
	// render state
	glEnable(GL_DEPTH_TEST);
	glShadeModel(GL_SMOOTH);

	// enable lighting
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	// light source attributes
	GLfloat LightAmbient[]	= { 0.4f, 0.4f, 0.4f, 1.0f };
	GLfloat LightDiffuse[]	= { 0.3f, 0.3f, 0.3f, 1.0f };
	GLfloat LightSpecular[]	= { 0.4f, 0.4f, 0.4f, 1.0f };
	GLfloat LightPosition[] = { 5.0f, 5.0f, 5.0f, 1.0f };

	glLightfv(GL_LIGHT0, GL_AMBIENT , LightAmbient );
	glLightfv(GL_LIGHT0, GL_DIFFUSE , LightDiffuse );
	glLightfv(GL_LIGHT0, GL_SPECULAR, LightSpecular);
	glLightfv(GL_LIGHT0, GL_POSITION, LightPosition);

	// surface material attributes
	GLfloat material_Ka[]	= { 1.0f, 0.5f, 0.0f, 1.0f };
	GLfloat material_Kd[]	= { 0.43f, 0.47f, 0.54f, 1.0f };
	GLfloat material_Ks[]	= { 0.0f, 0.0f, 0.0f, 1.0f };
	GLfloat material_Ke[]	= { 1.0f , 0.5f , 0.0f , 1.0f };
	GLfloat material_Se		= 10;

	glMaterialfv(GL_FRONT, GL_AMBIENT	, material_Ka);
	glMaterialfv(GL_FRONT, GL_DIFFUSE	, material_Kd);
	glMaterialfv(GL_FRONT, GL_SPECULAR	, material_Ks);
	glMaterialfv(GL_FRONT, GL_EMISSION	, material_Ke);
	glMaterialf (GL_FRONT, GL_SHININESS	, material_Se);

	// modelview matrix
	glMatrixMode( GL_MODELVIEW );
	glLoadIdentity();

	//Set position of camera
	Vector3f camera(0,5,100);
	Matrix4f view = translate(-camera(0), -camera(1), -camera(2));

	// render objects
	for (auto &obj : rigidbodys) {
		//glLoadIdentity();
		model = modelMat(obj.transform);

		Matrix4f modelview = model * view;
		

		for (int i = 0; i < 16; i++) {
			mat[i] = modelview(i);
		}
		glLoadMatrixf(mat);
		glutSolidSphere(obj.r, 32, 32);
	}
	

	// disable lighting
	glDisable(GL_LIGHT0);
	glDisable(GL_LIGHTING);


	for (int i = 0; i < 16; i++) {
		mat[i] = view(i);
	}
	
	glLoadIdentity();
	glLoadMatrixf(mat);
	glBegin(GL_POINTS);
	
	for (auto point : trace) {
		
		Vector4f ver(point(0), point(1), point(2), 1);
		ver = view * ver;
		glVertex2f(point(0), point(1));
	}
	glEnd();
	// swap back and front buffers
	glutSwapBuffers();
}

//================================
// keyboard input
//================================
void keyboard( unsigned char key, int x, int y ) {
}

//================================
// reshape : update viewport and projection matrix when the window is resized
//================================
void reshape( int w, int h ) {
	// screen size
	g_screenWidth  = w;
	g_screenHeight = h;	
	
	// viewport
	glViewport( 0, 0, (GLsizei)w, (GLsizei)h );

	// projection matrix
	glMatrixMode( GL_PROJECTION );
	glLoadIdentity();
	gluPerspective(45.0, (GLfloat)w/(GLfloat)h, 1.0, 2000.0);
}


//================================
// timer : triggered every 16ms ( about 60 frames per second )
//================================
void timer( int value ) {	
	// increase frame index
	g_frameIndex++;

	//check timer control of keyframe
	timecount += 16;

	update();
	
	// render
	glutPostRedisplay();

	// reset timer
	// 16 ms per frame ( about 60 frames per second )
	glutTimerFunc( 16, timer, 0 );
}

//================================
// main
//================================
int main( int argc, char** argv ) {
	// create opengL window
	glutInit( &argc, argv );
	glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGB |GLUT_DEPTH );
	glutInitWindowSize( 1800, 800 ); 
	glutInitWindowPosition( 0, 0 );
	glutCreateWindow("Computer Animation Lab3-three bodys");

	//Init series of rigid bodys with random attribute
	

	// init
	init();
	
	// set callback functions
	glutDisplayFunc( render );
	glutReshapeFunc( reshape );
	glutKeyboardFunc( keyboard );
	glutTimerFunc( 16, timer, 0 );
	
	// main loop
	glutMainLoop();

	return 0;
}