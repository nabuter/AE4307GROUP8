/* ------------------------------------------------------------------   */
/*      item            : F16HUDWindow.cxx
        made by         : DASTUDostroosma
        date            : Wed Mar 30 10:55:46 2011
	category        : body file 
        description     : 
	changes         : Wed Mar 30 10:55:46 2011 first version
        language        : C++
*/

static const char c_id[] =
"$Id: F16HUDWindow.cxx,v 1.1 2015/04/14 09:22:43 ostroosma Exp $";

#define F16HUDWindow_cxx
// include the definition of the module class
#include "F16HUDWindow.hxx"

// constructor
F16HUDWindow::F16HUDWindow():
  DuecaGLWindow("F16 HUD"),
  width(1024), height(768),
  hud(width, height, 20.0)
{
}

F16HUDWindow::~F16HUDWindow()
{
}

void F16HUDWindow::reshape(int w, int h)
{
  // remember
  width = w; height = h;

  glViewport(0, 0, w, h);
}

void F16HUDWindow::display(void)
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  hud.DrawGL();

  swapBuffers(); // from DuecaGLCanvas
}

void F16HUDWindow::initGL()
{
  // initialize the drawing routines
  hud.InitGL();

  // set up correct viewport
  reshape(width, height);
}
