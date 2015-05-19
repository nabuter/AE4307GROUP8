/* ------------------------------------------------------------------   */
/*      item            : F16HUDWindow.hxx
        made by         : DASTUDostroosma
        date            : Wed Mar 30 10:55:46 2011
	category        : header file 
        description     : 
	changes         : Wed Mar 30 10:55:46 2011 first version
        language        : C++
*/

#ifndef F16HUDWindow_hxx
#define F16HUDWindow_hxx

// DUECA stuff
#include <DuecaGLWindow.hxx> // from DUECA
USING_DUECA_NS;

// class containing the actual drawing routines
#include "HUD.hxx"

class F16HUDWindow: public DuecaGLWindow
{
public: 
  /** Constructor. */
  F16HUDWindow();

  /** Destructor. */
  ~F16HUDWindow();

public:
  /** Virtual functions inherited from DuecaGLCanvas through DuecaGLWindow.*/

  /** This is called if the size of the window is changed. You might
      need to update the image set-up for a different screen
      format. */
  virtual void reshape(int x, int y);
  
  /** This is called whenever the display needs to be redrawn. When
      called, the appropriate window has been made current. */
  virtual void display(); // pure virtual in DuecaGLCanvas itself

  /** This is called when the window is ready, for first-time
      set-up. DO NOT CALL THIS FUNCTION YOURSELF! Override this
      function, and when it is called, you can assume the gl code is
      possible. So creating viewports, GL lists, allocating textures
      etc. can be done in initGL. */
  virtual void initGL();

private:
  /** Width and height of the window. */
  int width, height;

  /** Drawing class. */
  HUD hud;

public:
  inline HUD& getHUD(){return hud;}

};

#endif
