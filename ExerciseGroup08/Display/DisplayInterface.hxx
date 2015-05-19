/***************************************************************************
                          DisplayInterface.hxx  -  description
                             -------------------
    begin                : Wed Sep 25 2002
    copyright            : (C) 2002 by Mark Mulder
    email                : ma.mulder@lr.tudelft.nl
 ***************************************************************************/


#ifndef DISPLAYINTERFACE_HXX
#define DISPLAYINTERFACE_HXX

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <DuecaGLWindow.hxx>

using namespace std;

/**
   @author Mark Mulder

   An OpenGL interface class that paints a simple square on the
   screen. The size of the square can be changed with the setAmplitude
   call. 
*/
class DisplayInterface : public dueca::DuecaGLWindow
{
public:

  /** Constructor, for creation of a new DisplayInterface. */
  DisplayInterface();

  /** Destructor, deletes the interface when we are done. */
  ~DisplayInterface();

  /** With this function you can change the size of the square. */
  void setAmplitude(float amp);

  /** Adjust colours. */
  void adjust(bool night);

public: 
  /** These functions are "implementations" of virtual function of the
      base class, DuecaGLWindow. */
  
  /** This function should contain the OpenGL code to draw the
      display. */
  void display();

  /** This function should makes adjustments when the display is
      re-sized. */
  void reshape(int, int);

  /** This function is called once, to set up the OpenGL environment. */
  void initGL();

private: // OpenGL variables
  // Defining the OpenGL window and viewport
  GLint window_width;  //  [pixels]
  GLint window_height;  // [pixels]

  /** The size of the square */
  float square_size;

  /** Night options. */
  bool night;

};

#endif
