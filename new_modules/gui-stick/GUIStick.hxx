/* ------------------------------------------------------------------   */
/*      item            : GUIStick.hxx
        made by         : tux
	from template   : DuecaModuleTemplate.hxx
        template made by: Rene van Paassen
        date            : Wed Apr  2 15:45:37 2014
	category        : header file 
        description     : 
	changes         : Wed Apr  2 15:45:37 2014 first version
	template changes: 030401 RvP Added template creation comment
        language        : C++
*/

#ifndef GUIStick_hxx
#define GUIStick_hxx

// include the dusime header
#include <dueca.h>
USING_DUECA_NS;

// This includes headers for the objects that are sent over the channels
#include "comm-objects.h"

// include headers for functions/classes you need in the module
#include <GtkGladeWindow.hxx>

/** A simulation module.
    
    The instructions to create an module of this class from the Scheme
    script are:

    \verbinclude gui-stick.scm
 */
class GUIStick: public Module
{
private: // simulation data
  // declare the data you need in your simulation
  GtkGladeWindow myPCWindow, mySCWindow, myPSWindow, mySSWindow;

  // configuration
  bool use_pc, use_sc, use_ps, use_ss;

  // calibration
  double ux_gain, ux_offset;
  double uy_gain, uy_offset;
  double uz_gain, uz_offset;
  double uc_gain, uc_offset;

  // widgets
  GtkWidget *pc_ux, *pc_uy, *pc_uz, *pc_uc;
  GtkWidget *pc_ux_label, *pc_uy_label, *pc_uz_label, *pc_uc_label;
  GtkWidget *sc_sb, *sc_thrl, *sc_thrr, *sc_lock; 
  GtkWidget *sc_f0, *sc_f1, *sc_f5, *sc_f15, *sc_f20, *sc_f25, *sc_f30;
  GtkWidget *ps_xmit, *ps_trup, *ps_trdwn, *ps_apd;
  GtkWidget *ss_atd, *ss_ignl, *ss_ignr, *ss_up, *ss_off, *ss_dwn;

  // strings
  char ux_string[32];
  char uy_string[32];
  char uz_string[32];
  char uc_string[32];
  
private: // channel access
  // declare StreamChannelReadToken<Type>, StreamChannelWriteToken<Type>
  // EventChannelReadToken<Type>, EventChannelWriteToken<Type>
  // access tokens for all the channels you read and write
  // example
  // StreamChannelReadToken<MyData>  my_token;
  StreamChannelWriteToken<PrimaryControls> *pc_token;
  StreamChannelWriteToken<SecondaryControls> *sc_token;
  StreamChannelWriteToken<PrimarySwitches> *ps_token;
  StreamChannelWriteToken<SecondarySwitches> *ss_token;

private: // activity allocation
  /** You might also need a clock. Don't mis-use this, because it is
      generally better to trigger on the incoming channels */
  PeriodicAlarm        myclock;

  /** Callback object for simulation calculation. */
  Callback<GUIStick>  cb1;

  /** Activity for simulation calculation. */
  ActivityCallback      do_calc;
  
public: // class name and trim/parameter tables
  /** Name of the module. */
  static const char* const           classname;

  /** Return the parameter table. */
  static const ParameterTable*       getMyParameterTable();
  
public: // construction and further specification
  /** Constructor. Is normally called from scheme/the creation script. */
  GUIStick(Entity* e, const char* part, const PrioritySpec& ts);

  /** Continued construction. This is called after all script
      parameters have been read and filled in, according to the
      parameter table. Your running environment, e.g. for OpenGL
      drawing, is also prepared. Any lengty initialisations (like
      reading the 4 GB of wind tables) should be done here.
      Return false if something in the parameters is wrong (by
      the way, it would help if you printed what!) May be deleted. */
  bool complete();

  /** Destructor. */
  ~GUIStick();

  // add here the member functions you want to be called with further 
  // parameters. These are then also added in the parameter table
  // The most common one (addition of time spec) is given here. 
  // Delete if not needed!

  /** Specify a time specification for the simulation activity. */
  bool setTimeSpec(const TimeSpec& ts);

  /** Request check on the timing. */
  bool checkTiming(const vector<int>& i);

public: // member functions for cooperation with DUECA
  /** indicate that everything is ready. */
  bool isPrepared();

  /** start responsiveness to input data. */
  void startModule(const TimeSpec &time);
  
  /** stop responsiveness to input data. */
  void stopModule(const TimeSpec &time);

public: // the member functions that are called for activities
  /** the method that implements the main calculation. */
  void doCalculation(const TimeSpec& ts);

public: // GTK callbacks
  void throttleChanged(GtkWidget *w, gpointer gp);
};

#endif
