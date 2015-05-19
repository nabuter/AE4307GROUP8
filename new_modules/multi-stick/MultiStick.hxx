/* ------------------------------------------------------------------   */
/*      item            : MultiStick.hxx
        made by         : repa
	from template   : DusimeModuleTemplate.hxx
        template made by: Rene van Paassen
        date            : Fri Dec  5 12:52:42 2003
	category        : header file 
        description     : 
	changes         : Fri Dec  5 12:52:42 2003 first version
	template changes: 030401 RvP Added template creation comment
        language        : C++
*/

#ifndef MultiStick_hxx
#define MultiStick_hxx

#ifdef MultiStick_cxx
static const char h_id[] =
"$Id: MultiStick.hxx,v 1.1 2015/04/14 09:22:46 ostroosma Exp $";
#endif

// include the dusime header
#include <dusime.h>

// This includes headers for the objects that are sent over the channels
#include "comm-objects.h"

// include headers for functions/classes you need in the module
#include <vector>
#include <list>
#include "MultiStickValue.hxx"

/** Multiple, joystick-based, versatile input module.
    This module accepts multiple joystick devices as inputs. 
    Then the connection between the buttons and axes of the joystick
    device may be supplied through "stick-value" objects, that
    optionally may carry scaling/stepping function to modify their
    output.
    
    \verbinclude multi-stick.scm

    The module is configured in the scheme script with the following
    steps:
    <ol>
    <li> Specify the usual, time specification, priority etc. 
    <li> Specify which joystick device files you want to read. You can
    supply multiple device files, all axes and buttons are seen as two
    large arrays of axes and buttons, numbering starts at 0,0 for the
    first stick, and simply continues for following sticks.
    <li> Specify how axes and buttons are turned into values on the
    output channel by supplying the module with stick value
    objects. See the MultiStickValue code for their description. The
    names of the stick value objects need to correspond to the names
    of the outputs, otherwise they are not used. Available names are
    <ul>
    <li> For PrimaryControls, "ux", "uy", "uz", "uc", "test_button"

    <li> For SecondaryControls, "fbrake_left", "fbrake_right",
    "throttle_left", "throttle_right", "speedbrake", "flap_setting"

    <li> For PrimarySwitches, "xmit", "trim_up", "trim_down",
    "AP_disconnect".

    <li> For SecondarySwitches, "AT_disconnect", "ignition_left",
    "ignition_right", "gear_up", "gear_neutral", "gear_down".
    </ul>
    </ol>

    A typical example is given here:

    \verbinclude multi-stick-example.mod
*/
class MultiStick: public SimulationModule
{
private: // simulation data

  /** struct that gives the necessary data to read one (Linux)
      joystick device. */
  struct JoystickDevice 
  {
    /** File descriptor. */
    int fd;
    /** Axis numbering offset, nonzero for 2nd and further joystick. */
    int offset_axis;
    /** Button numbering offset */
    int offset_button;
  };

  /** List of file descriptors/information for joystick devices */
  std::vector<JoystickDevice> device;

  /** A check on successful opening of all devices. */
  bool all_devices_open;

  /** Holds the raw data (-1 to 1) for all available axes. */
  std::vector<float> axis;

  /** Holds the raw data for all available buttons. */
  std::vector<bool>  button;

  /** For each of the outputs, a placeholder for an object that links
      the measured axes/buttons to the output values. */
  MultiStickValue* ux;
  MultiStickValue* uy;
  MultiStickValue* uz;
  MultiStickValue* uc;
  MultiStickValue* fbrake_left;
  MultiStickValue* fbrake_right;
  MultiStickValue* throttle_left;
  MultiStickValue* throttle_right;
  MultiStickValue* speedbrake;
  MultiStickValue* flap_setting;

  MultiStickValue* xmit;
  MultiStickValue* trim_up;
  MultiStickValue* trim_down;
  MultiStickValue* AP_disconnect;
  MultiStickValue* AT_disconnect;
  MultiStickValue* ignition_left;
  MultiStickValue* ignition_right;
  MultiStickValue* gear_up;
  MultiStickValue* gear_neutral;
  MultiStickValue* gear_down;
  MultiStickValue* test_button;

  /** A list of all supplied (and therefore available) MultiStickValue
      objects. */
  std::list<MultiStickValue*>  known_values;

private: // trim calculation data
  // declare the trim calculation data needed for your simulation

private: // snapshot data
  // declare, if you need, the room for placing snapshot data

private: // channel access
  /** Channel access tokens. */
  StreamChannelWriteToken<PrimaryControls>         *controls;
  StreamChannelWriteToken<SecondaryControls>       *secondary;
  StreamChannelWriteToken<PrimarySwitches>         *primary_switches;
  StreamChannelWriteToken<SecondarySwitches>       *secondary_switches;

private: // activity allocation
  /** Callback object for simulation calculation. */
  Callback<MultiStick>  cb1;

  /** Activity for simulation calculation. */
  ActivityCallback      do_calc;
  
public: // class name and trim/parameter tables
  /** Name of the module. */
  static const char* const           classname;

  /** Return the initial condition table. */
  static const IncoTable*            getMyIncoTable();

  /** Return the parameter table. */
  static const ParameterTable*       getMyParameterTable();
  
public: // construction and further specification
  /** Constructor. Is normally called from scheme/the creation script. */
  MultiStick(Entity* e, const char* part, const PrioritySpec& ts);

  /** Continued construction. This is called after all script
      parameters have been read and filled in, according to the
      parameter table. Your running environment, e.g. for OpenGL
      drawing, is also prepared. Any lengty initialisations (like
      reading the 4 GB of wind tables) should be done here.
      Return false if something in the parameters is wrong (by
      the way, it would help if you printed what!) May be deleted. */
  bool complete();

  /** Destructor. */
  ~MultiStick();

  // add here the member functions you want to be called with further 
  // parameters. These are then also added in the parameter table
  // The most common one (addition of time spec) is given here. 
  // Delete if not needed!

private:
  /** Specify a time specification for the simulation activity. */
  bool setTimeSpec(const TimeSpec& ts);

  /** Request check on the timing. */
  bool checkTiming(const vector<int>& i);

  /** enable output of PrimaryControls data */
  bool usePrimary(const bool& b);

  /** enable output of SecondaryControls data */
  bool useSecondary(const bool& b);

  /** enable output of PrimarySwitches data */
  bool usePrimarySwitches(const bool& b);

  /** enable output of SecondarySwitches data */
  bool useSecondarySwitches(const bool& b);

  /** Open a joystick device */
  bool setDevice(const vstring& dev);

  /** Supply a new MultiStickValue object. */
  bool addLink(ScriptCreatable& o, bool in);

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
};

#endif
