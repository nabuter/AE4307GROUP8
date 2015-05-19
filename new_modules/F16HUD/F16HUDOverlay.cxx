/* ------------------------------------------------------------------   */
/*      item            : F16HUDOverlay.cxx
        made by         : rvanpaassen
	from template   : DuecaModuleTemplate.cxx
        template made by: Rene van Paassen
        date            : Fri Mar 19 17:06:10 2010
	category        : body file 
        description     : 
	changes         : Fri Mar 19 17:06:10 2010 first version
	template changes: 030401 RvP Added template creation comment
        language        : C++
*/

#ifdef HAVE_WORLDVIEW

static const char c_id[] =
"$Id: F16HUDOverlay.cxx,v 1.1 2015/04/14 09:22:43 ostroosma Exp $";

#define F16HUDOverlay_cxx
// include the definition of the helper class
#include "F16HUDOverlay.hxx"

// include the debug writing header, by default, write warning and 
// error messages
#define D_MOD
#define I_MOD
#define W_MOD
#define E_MOD
#include <debug.h>

// include additional files needed for your calculation here
#include <GL/glu.h>

#define DO_INSTANTIATE
#include <VarProbe.hxx>
#include <MemberCall.hxx>
#include <MemberCall2Way.hxx>
#include <CoreCreator.hxx>

USING_DUECA_NS;
using namespace std;

// Parameters to be inserted
const ParameterTable* F16HUDOverlay::getParameterTable()
{
  static const ParameterTable parameter_table[] = {

    /* You can extend this table with labels and MemberCall or
       VarProbe pointers to perform calls or insert values into your
       class objects. Please also add a description (c-style string). */
    /*    { "strings",
      new VarProbe<F16HUDOverlay, vector<string> >
      (&F16HUDOverlay::messages), 
      "enter one or more strings with messages you want to display" },*/
      
    /* The table is closed off with NULL pointers for the variable
       name and MemberCall/VarProbe object. The description is used to
       give an overall description of the module. */
    { NULL, NULL, "F16 HUD in PLib Overlay form"} };

  return parameter_table;
}

// constructor
F16HUDOverlay::F16HUDOverlay() :
  ScriptCreatable(),
  PlibObject(),
  hud(800, 600, 20.0)
{
  
}

bool F16HUDOverlay::complete()
{
  /* All your parameters have been set. You may do extended
     initialisation here. Return false if something is wrong. */
  return true;
}



// destructor
F16HUDOverlay::~F16HUDOverlay()
{
  //
}

void F16HUDOverlay::draw(int viewno)
{ 
  // I am breaking in on the PLIB GL work here. Be very careful, reset
  // everything after doing this draw routine
  glMatrixMode(GL_PROJECTION);
  glPushMatrix(); // saves current (PLIB's)

  // define new 2d Projection matrix
  glLoadIdentity();
  gluOrtho2D(0.0, 500.0, 0.0, 500.0);
  
  // define identity model matrix
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix(); // saves current (PLIB's)
  glLoadIdentity();  

  // save attributes, a bit coarse, might save space and time by
  // selecting exactly which attributes I change here!
  glPushAttrib(GL_ALL_ATTRIB_BITS);
  
  glDisable(GL_COLOR_MATERIAL);
  glDisable(GL_FOG);
  glDisable(GL_COLOR_LOGIC_OP);
  glDisable(GL_LIGHTING);
  glDisable(GL_DEPTH_TEST);
  glDisable(GL_TEXTURE_2D);
  glDisable(GL_COLOR_SUM);
  glDisable(GL_CULL_FACE);
  
  // turn alpha blending on
  /*  glDisable(GL_LIGHTING);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_BLEND);
  glDisable(GL_DEPTH_TEST);


  // alpha blending off
  glDisable(GL_BLEND);*/

  // draw stuff
  // HUD actually makes its own Matrices
  hud.DrawGL();
  
  // restore the attributes
  glPopAttrib();

  // restore the projection and model matrices
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
};


// script access macro
SCM_FEATURES_NOIMPINH(F16HUDOverlay, ScriptCreatable, "f16-hud-overlay");

// Make a CoreCreator object for this module, the CoreCreator
// will check in with the scheme-interpreting code, and enable the
// creation of objects of this type
static CoreCreator<F16HUDOverlay> a(F16HUDOverlay::getParameterTable());

#endif
