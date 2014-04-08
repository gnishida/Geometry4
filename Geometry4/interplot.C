#include "kdtree.h"
#include "plot.h"

LineSegments lineSegments;
LineSegment lineSegment;
bool intersected = false;

void graphics_init_aux () {}

void menu_init ()
{
  glutCreateMenu(menu);
  glutAddMenuEntry("clear", 1);
  glutAddMenuEntry("read points", 2);
  glutAddMenuEntry("write points", 3);
  glutAddMenuEntry("exit", 4);
  glutAttachMenu(GLUT_RIGHT_BUTTON);
}

void menu (int value)
{
  switch (value) {
  case 1:
	clear_lines(lineSegments);
    break;
  case 2:
    //read_points(inpoints);
    break;
  case 3:
    //writePoints(cerr, inpoints);
    break;
  case 4:
    exit(0);
  }
  glutPostRedisplay();
}

void mouse (int button, int buttonState, int x, int y)
{
  mouse_generic(button, buttonState, x, y, lineSegments, lineSegment);
  if( buttonState == GLUT_UP) {
    
    intersected = intersects(lineSegments, lineSegment);
  }
  glutPostRedisplay();
}

void display ()
{
  glutSetWindowTitle("convex hull");
  float white[3] = {1.0f, 1.0f, 1.0f};
  clear_screen(white);
  glColor3f(0.0f, 0.0f, 0.0f);
  draw_lineSegments(lineSegments);

  if (lineSegment.p1 != 0) {
    if (intersected) {
      glColor3f(1.0f, 0.0f, 0.0f);
	} else {
      glColor3f(0.0f, 0.0f, 1.0f);
	}
	draw_line(lineSegment.p0, lineSegment.p1);
  }
  glFlush();
}
