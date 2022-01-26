#include <iostream>
#include <fstream>
#include <cstring>
#include <vector>
#include <Eigen/Dense>
#ifdef __APPLE__
#include <GLUT/glut.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#ifndef M_PI
#define	M_PI		3.14159265358979323846	/* pi */
#endif

double size = 0.01;

class Light {
public:
  GLfloat ambient[4]; // ambient color, RGBA
  GLfloat diffuse[4]; // diffuse color, RGBA
  GLfloat specular[4]; // specular color, RGBA
  GLfloat pos[4]; // light position, XYZW
  GLenum id; // light identifier
  Light() {}; // constructor
  void apply() {
	glLightfv(id, GL_AMBIENT, ambient);
	glLightfv(id, GL_DIFFUSE, diffuse);
	glLightfv(id, GL_SPECULAR, specular);
	glLightfv(id, GL_POSITION, pos);
	glEnable(id);
  }
};

class Fish {
public:
  Eigen::Vector3d pos, vel;
  void draw();
};

struct Frame {
  std::vector<Fish> fish;
  std::vector<Eigen::Vector3d> food;
};

// Global Variables
std::vector<Frame> frames;
std::vector<Light> lights;
int vWidth, vHeight;
int frame;

bool readAnimation(char *fname, std::vector<Frame> &frames) {
  int nframes, nfish, nfood;
  char junk;

  std::ifstream in(fname, std::ios::in);

  in>>nframes;
  frames.resize(nframes);
  for (int i=0; i<nframes; i++) {
	in>>nfish;
	for (unsigned int j=0; j<nfish; j++) {
	  Fish f;
	  in >> junk >> f.pos[0] >> junk >> f.pos[1] >> junk >> f.pos[2] >> junk;
	  in >> junk >> f.vel[0] >> junk >> f.vel[1] >> junk >> f.vel[2] >> junk;
	  frames[i].fish.push_back(f);
	}
	in>>nfood;
	for (unsigned int j=0; j<nfood; j++) {
	  Eigen::Vector3d x;
	  in >> junk >> x[0] >> junk >> x[1] >> junk >> x[2] >> junk;
	  frames[i].food.push_back(x);
	}
  }
  return true;
}

///////////////////////////////////////////////////
// Begin Class Function Definitions
///////////////////////////////////////////////////
void Fish::draw() {
  glPushMatrix();
  GLfloat c[4] = {0.0, 1.0, 0.0, 1.0};
  glTranslated(pos[0], pos[1], pos[2]);
  glMaterialfv(GL_FRONT, GL_AMBIENT, c);
  glMaterialfv(GL_FRONT, GL_DIFFUSE, c);
  glMaterialfv(GL_FRONT, GL_SPECULAR, c);
  Eigen::Vector3d axis = vel;
  double m = axis.squaredNorm();
  if (m > 1e-4) {
	axis /= sqrt(m);
	glRotated(-acos(axis[2])*180 / M_PI, axis[1], -axis[0], 0.0);
  }
  glutSolidCone(size, 10.0*sqrt(sqrt(m))*size, 10, 10);
  glutSolidSphere(size, 10, 10);
  glPopMatrix();
}

///////////////////////////////////////////////////
// End Class Definitions
///////////////////////////////////////////////////

void myReshape(int w, int h) {
  glViewport (0,0,w,h);
  vWidth = w;
  vHeight = h;
}

void myDisplay() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glEnable(GL_LIGHTING);
  glEnable(GL_DEPTH_TEST); /* enable depth testing; required for z-buffer */
  glEnable(GL_CULL_FACE); /* enable polygon face culling */ 
  glCullFace(GL_BACK); /* tell opengl to cull the back face*/ 
  glEnable(GL_NORMALIZE);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(60, ((float)vWidth)/vHeight, 0.75, 2.5);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  gluLookAt(0.0, 0.0, 1.0, 
	  0.0, 0.0, 0.0,
	  0.0, 1.0, 0.0);
  for (int i=0; i<lights.size(); i++) lights[i].apply();
  for (int i=0; i<frames[frame].fish.size(); i++) frames[frame].fish[i].draw();

  GLfloat c[4] = {0.0, 0.0, 1.0, 1.0};
  glMaterialfv(GL_FRONT, GL_AMBIENT, c);
  glMaterialfv(GL_FRONT, GL_DIFFUSE, c);
  glMaterialfv(GL_FRONT, GL_SPECULAR, c);
  glPointSize(5.0f);
  glBegin(GL_POINTS);
  std::vector<Eigen::Vector3d> &food = frames[frame].food;
  for (unsigned int i=0; i<food.size(); i++) glVertex3f(food[i][0], food[i][1], food[i][2]);
  glEnd();
  
  
  glutSwapBuffers();
}

void myKeyboard(unsigned char key, int x, int y) {
  switch(key) {
  case 'q':
  case 'Q':
	exit(0);
	break;
  default:
	std::cerr<<"key "<<key<<" not supported"<<std::endl;
  }
  glutPostRedisplay();
}

void myTimerFunc(int id) {
  frame++;
  if (frame > frames.size()-1) frame = 0;
  glutPostRedisplay();
  glutTimerFunc(33, myTimerFunc, 0);
}

int main(int argc, char *argv[]) {
  glutInit(&argc, argv);
  
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
  glutInitWindowSize(600, 400);
  glutInitWindowPosition(0,0);
  glutCreateWindow(argv[0]);
  
  glutDisplayFunc(myDisplay);
  glutReshapeFunc(myReshape);
  glutKeyboardFunc(myKeyboard);
  glutTimerFunc(1000, myTimerFunc, 0);
  
  for (int i=0; i<1; i++) {
	Light l;
	l.ambient[0] = 0.2; l.ambient[1] = 0.2; l.ambient[2] = 0.2; l.ambient[3] = 0.2;
	l.diffuse[0] = 0.7; l.diffuse[1] = 0.7; l.diffuse[2] = 0.7; l.diffuse[3] = 0.7;
	l.specular[0] = 0.4; l.specular[1] = 0.4; l.specular[2] = 0.4; l.specular[3] = 0.4;
	switch(i) {
	case 0:
	  l.pos[0] = 0.0; l.pos[1] = 1.0; l.pos[2] = 0.0; l.pos[3] = 1.0;
	  break;
	case 1:
	  l.pos[0] = 1.0; l.pos[1] = 0.0; l.pos[2] = 0.0; l.pos[3] = 1.0;
	  break;
	case 2:
	  l.pos[0] = -1.0; l.pos[1] = 0.0; l.pos[2] = 0.0; l.pos[3] = 1.0;
	  break;
	case 3:
	  l.pos[0] = 0.0; l.pos[1] = 0.0; l.pos[2] = -1.0; l.pos[3] = 1.0;
	  break;
	case 4:
	  l.pos[0] = 0.0; l.pos[1] = 0.0; l.pos[2] = 1.0; l.pos[3] = 1.0;
	  break;
	}
	l.id = GL_LIGHT0+i;
	lights.push_back(l);
  }
  
  
  frame = 0;
  readAnimation(argv[1], frames);

  glutMainLoop();
  
  return 0;
}

