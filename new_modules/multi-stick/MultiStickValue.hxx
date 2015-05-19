/* ------------------------------------------------------------------   */
/*      item            : MultiStickValue.hxx
        made by         : Rene van Paassen
        date            : 031205
	category        : header file 
        description     : 
	changes         : 031205 first version
        language        : C++
*/

#ifndef MultiStickValue_hxx
#define MultiStickValue_hxx

#ifdef MultiStickValue_cxx
static const char h_id[] =
"$Id: MultiStickValue.hxx,v 1.1 2015/04/14 09:22:46 ostroosma Exp $";
#endif

#include "MuStValueHolder.hxx"
#include <ScriptCreatable.hxx>
#include <StepsN.hxx>
#include <PolynomialN.hxx>
#include <SimpleFunction.hxx>
#include <stringoptions.h>
#include <ParameterTable.hxx>
#include <dueca_ns.h>

USING_DUECA_NS;

/** A class definition for a scheme-creatable object. These objects
    should help the MultiStick class obtain a very flexible
    programming of the reading of axes and buttons from a (for now)
    joystick device, or also several joystick devices, and using these
    to drive the typical aircraft controls.
    
    A MultiStickValue object may for example simply read a joystick
    axis, and then use a simple polynomial to scale the output.
    
    More complicated uses might be implementing a counter driven by
    buttons, or axis values, and scaling its output with a tabulated
    function. This might be used to get a 0, 1, 5, 10, 20, 30, 40
    sequence from buttons controlling flap setting. 

    The description for creating a MultiStickValue from a script:

    \verbinclude stick-value.scm

    Note that the MultiStick module will provide values between -1 and
    1 for the axes of a calibrated joystick (check out jscal on
    linux). So implement any scaling polynomials or stepping function
    to convert the range of -1 to 1 into the values you need. 
*/
class MultiStickValue: public ScriptCreatable
{
  /** Object that knows how to obtain and hold the value from an array
      of axes and an array of buttons offered. Knows the indexing, and
      what to do. Some value holders can count. */
  MuStValueHolder *value_holder;

  /** Object that knows how to transform the value from the value
      holder, to obtain a final product. */
  SimpleFunction *calibrator;

  /** Name of this MultiStickValue. */
  vstring name;

public:
  /** Constructor. */
  MultiStickValue();
  
  /** Destructor. */
  ~MultiStickValue();
  
  /** Complete method. */
  bool complete();

  /** Obtain a pointer to the parameter table. */
  static const ParameterTable* getParameterTable();

public:
  /** Default script linkage. */
  SCM_FEATURES_DEF;

private:
  /** install an axis reader */
  bool setReadAxis(const int& ax);
  
  /** install a button reader */
  bool setReadButton(const int& but);
  
  /** install a "criterion" reader, uses an axis value and converts
      this to a button if within a specific range, supply the axis no,
      min and max value. */
  bool setReadCriterion(const vector<float>& vec);
  
  /** Install a counter, up to four integers, max value, center value,
      initial repeat, subsequent repeat. */
  bool setCounter(const vector<int>& ivec);

  /** Specify the up button for the installed counter, counter must be
      previously specified. */
  bool setCounterUpInput(ScriptCreatable& up, bool in_out);

  /** Specify the down button for the installed counter, counter must be
      previously specified. */
  bool setCounterDownInput(ScriptCreatable& up, bool in_out);

  /** Specify a criterion for counting up. */
  bool setCounterCenterInput(ScriptCreatable& up, bool in_out);

  /** Set up a polynomial calibrator. */
  bool setPolynomial(const vector<double>& c);
  
  /** Set up stepped discriminator. */
  bool setSteps(const vector<double>& c);

public: 
  
  /** Access the object that does the actual bookkeeping. */
  inline void update(const std::vector<float>& axis, 
		     const std::vector<bool>& button, 
		     int elapsed) 
  { value_holder->updateValue(axis, button, elapsed); }

  /** Obtain a floating value. */
  double getDouble() const;

  /** Obtain a truth value. */
  bool getBool() const;

  /** Find out the name. */
  inline const vstring getName() const { return name; }
};

#endif
