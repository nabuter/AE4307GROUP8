/***************************************************************************
                          DisplayInterface.cxx  -  description
                             -------------------
    begin                : Wed Sep 25 2002
    modified             : Tuesday, 22 Oct 2002
    copyright            : (C) 2002 by Mark Mulder
    email                : ma.mulder@lr.tudelft.nl
 ***************************************************************************/

#include "DisplayInterface.hxx"
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cmath>

// constructor
DisplayInterface::DisplayInterface():
  DuecaGLWindow("OpenGL window", true, true, true, true),
  // Defining the OpenGL window and viewport //
  window_width(800),  //[pixels]
  window_height(600),  //[pixels]
  square_size(0.0f),
  night(false)
{
  setWindow(0, 0, window_width, window_height);
}

// destructor
DisplayInterface::~DisplayInterface()
{
  // nothing needs to be done explicitly here
}

void DisplayInterface::setAmplitude(float amp)
{
  if (amp > 1.0f) {
    cerr << "DisplayInterface::setAmplitude input too large" << std::endl;
    square_size = 1.0f;
  }
  else if (amp < 0.0f) {
    cerr << "DisplayInterface::setAmplitude input too small" << std::endl;
    square_size = 0.0f;
  }
  else {
    square_size = amp;
  }
}

void DisplayInterface::adjust(bool night)
{
  this->night = night;
}

// openGL initialisation
void DisplayInterface::initGL()
{
  // get Viewport size in pixels
  GLint viewport_width = window_width;
  GLint viewport_height = window_height;

  // set viewport
  glViewport(0, 0, (GLsizei) viewport_width, (GLsizei) viewport_height);

  // Select the projection matrices for the following operation,
  glMatrixMode(GL_PROJECTION);

  //define 2-D projection matrix
  glLoadIdentity(); 
  gluOrtho2D(-1.0, 1.0, -1.0, 1.0);
  
  // at this point we have a 2d coordinate system with 0, 0 in the center
  // of the screen, lower left corner is -1,-1 and upper right corner
  // is 1,1

  //set matrix mode to modelview
  glMatrixMode(GL_MODELVIEW);
}

// openGL window reshape function
void DisplayInterface::reshape(int w, int h)
{
  window_width = w;
  window_height = h;

  // call initGL now the width and height are changed
  initGL();
}

// openGL drawing code
void DisplayInterface::display()
{
  glClearColor(0.0, 0.0, 0.0, 0.0);

  // clear screen
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  { 
    // save the transformation matrix
    glPushMatrix();
    
    // do a scaling transformation
    glScalef(square_size, square_size, 1.0f);
    
    // take a nice color
    if (night) {
      glColor3f(0.5f*square_size, 0.4, 0.5f*(1.0f - square_size));
    }
    else { // day
      glColor3f(square_size, 0.8, 1.0f - square_size);
    }      

    { // make a strip of two triangles
      glBegin(GL_TRIANGLE_STRIP);
      glVertex3f(-1.0f, -1.0f, 0.0f);
      glVertex3f( 1.0f, -1.0f, 0.0f);
      glVertex3f(-1.0f,  1.0f, 0.0f);
      glVertex3f( 1.0f,  1.0f, 0.0f);
      glEnd();
    }

    glPopMatrix();
  }

  glFlush();
  swapBuffers();
}

