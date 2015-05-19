/* ------------------------------------------------------------------   */
/*      item            : MultiStickValue.cxx
        made by         : Rene' van Paassen
        date            : 031205
	category        : body file 
        description     : 
	changes         : 031205 first version
	                  040319 RvP made it possible to create a 
			  constant-value object, by allowing null
			  value_holder. 
        language        : C++
*/

const static char c_id[] =
"$Id: MultiStickValue.cxx,v 1.1 2015/04/14 09:22:46 ostroosma Exp $";

#define MultiStickValue_cxx
#include "MultiStickValue.hxx"

#define W_CNF
#define E_CNF
#include <debug.h>
#include <cmath>
#include <memory>

#define DO_INSTANTIATE
#include <VarProbe.hxx>
#include <MemberCall.hxx>
#include <MemberCall2Way.hxx>
#include <CoreCreator.hxx>
USING_DUECA_NS;

const ParameterTable* MultiStickValue::getParameterTable()
{
  static const ParameterTable table[] = {
    { "name", new VarProbe<MultiStickValue,vstring>
      (REF_MEMBER(&MultiStickValue::name)), 
      "identifying name for the stick value reader" },

    /* creation of different sub-types */
    { "read-axis", new MemberCall<MultiStickValue,int>
      (&MultiStickValue::setReadAxis),
      "read off a value from one of the joystick axes, and use it as a\n"
      "continuous input. Give the axis number as argument" },
    { "read-button", new MemberCall<MultiStickValue,int>
      (&MultiStickValue::setReadButton),
      "read off a button from the joystick, and use it as a truth value\n"
      "Give the button number as argument" },
    { "read-criterion", new MemberCall<MultiStickValue,vector<float> >
      (&MultiStickValue::setReadCriterion),
      "read a value from one of the joystick axes (scaled -1 to 1), and test\n"
      "it against minimum and maximum values to convert it to a truth value\n"
      "needs three arguments; the axis number, min criterion, and max\n"
      "criterion, true if min < val < max"},
    { "create-counter", new MemberCall<MultiStickValue,vector<int> >
      (&MultiStickValue::setCounter),
      "create a counting device. Supply the following integer values:\n"
      " * a (positive) integer number for the maximum count\n"
      " * a (nonnegative) integer number for center (init) value (optional)\n"
      " * a time value (in DUECA increments) for initial repeat action (opt)\n"
      " * a time value for subsequent repeat actions (opt)\n"
      "You must also indicate how the counter is supposed to be driven,\n"
      "by creating stick-value objects that are set up with\n"
      "either read-button or read-criterion objects, attach them with\n"
      "'set-counter-up, 'set-counter-down and optionally 'center-counter." },

    /* specification for the inputs of the criterion. */
    { "set-counter-up", new MemberCall2Way<MultiStickValue,ScriptCreatable>
      (&MultiStickValue::setCounterUpInput), 
      "provide a logical stick value that drives up counting" },
    { "set-counter-down", new MemberCall2Way<MultiStickValue,ScriptCreatable>
      (&MultiStickValue::setCounterDownInput), 
      "provide a logical stick value that drives down counting" },
    { "center-counter", new MemberCall2Way<MultiStickValue,ScriptCreatable>
      (&MultiStickValue::setCounterUpInput), 
      "provide a logical stick value that controls centering (zeroing)\n"
      "of a counter" },

    /* calibration objects. */
    { "calibration-polynomial", new MemberCall<MultiStickValue,vector<double> >
      (&MultiStickValue::setPolynomial),
      "provide a polynomial for calibration, coefficients in increasing\n"
      "of power; a0 + a1 x + a2 x^2 etc." },
    { "calibration-steps", new MemberCall<MultiStickValue,vector<double> >
      (&MultiStickValue::setSteps),
      "provide 'choice' steps for calibration, give input, output pairs\n"
      "u0, y0, u1, y1 etc. Finds u closest to input value, returns\n"
      "corresponding output value" }, 
    { NULL, NULL, 
      "A helper object for the MultiStick class, connects joystick inputs\n"
      "in various ways to the produced outputs" }
  };
  return table;
}

MultiStickValue::MultiStickValue() :
  value_holder(NULL),
  calibrator(NULL),
  name()
{
  // nothing drastic yet
}

MultiStickValue::~MultiStickValue()
{
  // delete children
  delete value_holder;
  delete calibrator;
}

bool MultiStickValue::complete()
{
  return true;
}

bool MultiStickValue::setReadAxis(const int& ax)
{
  // check for double trouble
  if (value_holder) {
    E_CNF("attempting to re-define stick value");
    return false;
  }
  
  // check the axis value is non-negative
  if (ax < 0) {
    E_CNF("supply an axis number >= 0");
    return false;
  }

  // create the appropriate reader object
  value_holder = new MuStAxisReader(ax);
  return true;
}

bool MultiStickValue::setReadButton(const int& but)
{
  // check for double trouble
  if (value_holder) {
    E_CNF("attempting to re-define stick value");
    return false;
  }
  
  // check the axis value is non-negative
  if (but < 0) {
    E_CNF("supply a button number >= 0");
    return false;
  }

  // create the appropriate reader object
  value_holder = new MuStButtonReader(but);
  return true;
}

bool MultiStickValue::setReadCriterion(const vector<float>& vec)
{
  // check for double trouble
  if (value_holder) {
    E_CNF("attempting to re-define stick value");
    return false;
  }
  
  // check that 1: three values are given, first one is integer, and
  // non-negative (axis number), and second one smaller than third one
  if (vec.size() != 3 || rintf(vec[0]) != vec[0] || vec[2] < vec[1]) {
    E_CNF("supply three values: (1) an axis number, (2) min and (3) max");
    return false;
  }

  // create the "criterion" object
  value_holder = new MuStCriterion(int(vec[0]), vec[1], vec[2]);
  return true;
}

bool MultiStickValue::setCounter(const vector<int>& ivec)
{
  // check for double trouble
  if (value_holder) {
    E_CNF("attempting to re-define stick value");
    return false;
  }
  
  // check that 1 at least size ivec is one, 2 a possible center value
  // is within 0 and maximum value, 3, repeat counts are realistic
  if (ivec.size() < 1 ||
      ivec[0] <= 0 ||
      (ivec.size() >= 2 && (ivec[1] < 0 || ivec[1] > ivec[0])) ||
      (ivec.size() >= 3 && (ivec[2] < 0)) ||
      (ivec.size() == 4 && (ivec[3] < 0)) ||
      ivec.size() > 4) {
    E_CNF("supply 1 to 4 values: maxcount, center value, initial repeat"
	  "next repeat");
    return false;
  }

  int center = (ivec.size() > 1) ? ivec[1] : 0;
  int repeat0 = (ivec.size() > 2) ? ivec[2] : 0;
  int repeat1 = (ivec.size() > 3) ? ivec[3] : repeat0;
  
  // create the "counter" object
  value_holder = new MuStCounter(ivec[0], center, repeat0, repeat1);
  return true;
}


bool MultiStickValue::setCounterUpInput(ScriptCreatable& up, bool in)
{
  // check direction
  if (!in) return false;

  // check that we already have a value holder, and that this is
  // indeed a counter!
  if (!value_holder || !dynamic_cast<MuStCounter*> (value_holder)) {
    E_CNF("no counter yet");
    return false;
  }

  // try a dynamic cast
  MultiStickValue* value = dynamic_cast<MultiStickValue*> (&up);
  if (value == NULL) {
    E_CNF("must supply a stick-value");
    return false;
  }

  // now make sure Scheme does not clean the helper from right
  // under our noses:
  scheme_id.addReferred(value->scheme_id.getSCM());

  // this has to be a multistickvalue, so use it, returns true if OK
  return dynamic_cast<MuStCounter*> (value_holder)->setUpCounter(value);
}

bool MultiStickValue::setCounterDownInput(ScriptCreatable& up, bool in)
{
  // check direction
  if (!in) return false;

  // check that we already have a value holder, and that this is
  // indeed a counter!
  if (!value_holder || !dynamic_cast<MuStCounter*> (value_holder)) {
    E_CNF("no counter yet");
    return false;
  }

  // try a dynamic cast
  MultiStickValue* value = dynamic_cast<MultiStickValue*> (&up);
  if (value == NULL) {
    E_CNF("must supply a stick-value");
    return false;
  }

  // now make sure Scheme does not clean the helper from right
  // under our noses:
  scheme_id.addReferred(value->scheme_id.getSCM());

  // this has to be a multistickvalue, so use it, returns true if OK
  return dynamic_cast<MuStCounter*> (value_holder)->setDownCounter(value);
}

bool MultiStickValue::setCounterCenterInput(ScriptCreatable& up, bool in)
{
  // check direction
  if (!in) return false;

  // check that we already have a value holder, and that this is
  // indeed a counter!
  if (!value_holder || !dynamic_cast<MuStCounter*> (value_holder)) {
    E_CNF("no counter yet");
    return false;
  }

  // try a dynamic cast
  MultiStickValue* value = dynamic_cast<MultiStickValue*> (&up);
  if (value == NULL) {
    E_CNF("must supply a stick-value");
    return false;
  }

  // now make sure Scheme does not clean the helper from right
  // under our noses:
  scheme_id.addReferred(value->scheme_id.getSCM());

  // this has to be a multistickvalue, so use it, returns true if OK
  return dynamic_cast<MuStCounter*> (value_holder)->setResetter(value);
}

bool MultiStickValue::setPolynomial(const vector<double>& c)
{
  // no calibrator yet?
  if (calibrator) {
    E_CNF("Already specified calibrator");
    return false;
  }

  // need at least 1 value (silly!)
  if (c.size() < 1) {
    E_CNF("Specify polynomial coefficients");
    return false;
  }

  // for cleannes, copy
  double a[c.size()]; 
  copy(c.begin(), c.end(), raw_storage_iterator<double*, double>(a));

  // create the polynomial
  calibrator = new PolynomialN(c.size() - 1, a);
  return true;
}

bool MultiStickValue::setSteps(const vector<double>& c)
{
  // no calibrator yet?
  if (calibrator) {
    E_CNF("Already specified calibrator");
    return false;
  }

  // need at least 1 value (silly!)
  if (c.size() % 2 == 1 || c.size() < 2) {
    E_CNF("Specify polynomial coefficients");
    return false;
  }

  // create & fill arrays 
  double ui[c.size()/2], yi[c.size()/2];
  for (int ii = c.size() / 2; ii--; ) {
    ui[ii] = c[2*ii]; yi[ii] = c[2*ii+1];
  }
  
  // create the thing
  calibrator = new StepsN(c.size()/2, yi, ui);
  return true;
}


double MultiStickValue::getDouble() const
{
  double raw = value_holder ? value_holder->getValue() : 0.0;
  if (calibrator) return (*calibrator)(raw);
  return raw;
}

bool MultiStickValue::getBool() const
{
  bool raw = value_holder ? value_holder->getTruth() : false;
  if (calibrator) return (*calibrator)(raw ? 1.0 : 0.0) != 0.0;
  return raw;
}

// script access macro
SCM_FEATURES_NOIMPINH(MultiStickValue, ScriptCreatable, "stick-value");

// create a static object that declares access to the script language
CoreCreator<MultiStickValue> m(MultiStickValue::getParameterTable());
