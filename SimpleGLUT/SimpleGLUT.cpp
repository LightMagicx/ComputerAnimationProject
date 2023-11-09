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

GLfloat mat[16];

int timecount = 0;

//animation sequence
Sequence myAnime;

//================================
// init
//================================
void init( void ) {
	// init something before main loop...
}

//================================
// update
//================================
void update( void ) {
	// do something before rendering...

	// rotation angle
	//g_angle = ( g_angle + 5 ) % 360;

	//setting transform
	if (g_frameIndex < myAnime.sequence.size()) {

		GLfloat* pMat = modelMat(myAnime.sequence[g_frameIndex]);
		for (int i = 0; i < 16; i++) {
			mat[i] = pMat[i];
		}
		/*
		Matrix4f matTrans;
		Matrix4f matRotate;
		Matrix4f matModel;

		

		matTrans = translate(x_trans, y_trans, z_trans);
		matRotate = quatRotate(g_angle, x_angle, y_angle, z_angle);
		matModel = matRotate * matTrans;

		
		for (int i = 0; i < 16; i++) {
			mat[i] = matModel(i);
		}	

		*/

		x_trans = myAnime.sequence[g_frameIndex-1].location(0);
		y_trans = myAnime.sequence[g_frameIndex-1].location(1);
		z_trans = myAnime.sequence[g_frameIndex-1].location(2);

		g_angle = acos(myAnime.sequence[g_frameIndex-1].quat.w);
		x_angle = myAnime.sequence[g_frameIndex-1].quat.x;
		y_angle = myAnime.sequence[g_frameIndex-1].quat.y;
		z_angle = myAnime.sequence[g_frameIndex-1].quat.z;
		
	}
}


//================================
// render
//================================
void render( void ) {
	// clear buffer
	glClearColor (0.0, 0.0, 0.0, 0.0);
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
	GLfloat material_Ka[]	= { 0.11f, 0.06f, 0.11f, 1.0f };
	GLfloat material_Kd[]	= { 0.43f, 0.47f, 0.54f, 1.0f };
	GLfloat material_Ks[]	= { 0.33f, 0.33f, 0.52f, 1.0f };
	GLfloat material_Ke[]	= { 0.1f , 0.0f , 0.1f , 1.0f };
	GLfloat material_Se		= 10;

	glMaterialfv(GL_FRONT, GL_AMBIENT	, material_Ka);
	glMaterialfv(GL_FRONT, GL_DIFFUSE	, material_Kd);
	glMaterialfv(GL_FRONT, GL_SPECULAR	, material_Ks);
	glMaterialfv(GL_FRONT, GL_EMISSION	, material_Ke);
	glMaterialf (GL_FRONT, GL_SHININESS	, material_Se);

	// modelview matrix
	glMatrixMode( GL_MODELVIEW );
	glLoadIdentity();

	


	//glTranslatef (0.0, 0.0, -5.0);
	
	GLfloat modelMatrix[16];
	glTranslatef(x_trans, y_trans, z_trans);

	glGetFloatv(GL_MODELVIEW_MATRIX, modelMatrix);

	glRotated(g_angle, x_angle, y_angle, z_angle);

	glGetFloatv(GL_MODELVIEW_MATRIX, modelMatrix);
	
	//glLoadMatrixf(modelMatrix);
	
	glLoadMatrixf(mat);

	// render objects
	glutSolidTeapot(0.5);
	//glutWireTeapot(0.5);
	//glutSolidSphere(0.5,32,32);
	

	// disable lighting
	glDisable(GL_LIGHT0);
	glDisable(GL_LIGHTING);

	

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
	glutInitWindowSize( 600, 600 ); 
	glutInitWindowPosition( 100, 100 );
	glutCreateWindow("Computer Animation Lab1");

	//setup k-frame sequence
	Sequence kframes;
	
	for (int i = 0; i < 10; i++) {
		myTransform kframe;

		kframe.location << (rand() % 4) - 2, (rand() % 4) - 2, -5;
		
		kframe.rotation << (rand() % 360)-180, (rand() % 360) - 180, (rand() % 360) - 180;
		kframes.sequence.push_back(kframe);
	}

	/*
	myTransform frame1, frame2, frame3, frame4;
	frame1.location << 0, 0, -5;
	frame2.location << 1, 0, -5;
	frame3.location << 2, 0, -5;
	frame4.location << 3, 0, -5;

	frame1.rotation << 0, 0, 0;
	frame2.rotation << 0, 90, 0;
	frame3.rotation << 0, 180, 0;
	frame4.rotation << 45, 45, 0;

	kframes.sequence.push_back(frame1);
	kframes.sequence.push_back(frame2);
	kframes.sequence.push_back(frame3);
	kframes.sequence.push_back(frame4);
	*/
	
	
	myAnime = Catmall_Rom(kframes,false);
	//myAnime = Bspline(kframes, false);

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