/* ------------------------------------------------------------------   */
/*      item            : MuStValueHolder.hxx
        made by         : Rene van Paassen
        date            : 031205
	category        : header file 
        description     : 
	changes         : 031205 first version
        language        : C++
*/

#ifndef MuStValueHolder_hxx
#define MuStValueHolder_hxx

#ifdef MuStValueHolder_cxx
static const char h_id[] =
"$Id: MuStValueHolder.hxx,v 1.1 2015/04/14 09:22:46 ostroosma Exp $";
#endif

#include <vector>
class MultiStickValue;

/** Virtual base class for objects that can read and keep a value. */
class MuStValueHolder
{
protected:
  /** Index of either the button or axis needed for reading the
      value. */
  unsigned int idx;

public:
  /** Constructor, only needs the index. */
  MuStValueHolder(unsigned int idx);

  /** Destructor. */
  virtual ~MuStValueHolder();

  /** Update the value, from either the axis array or the button
      array, depending on the child.
      \param axis     Array with values for the (joystick) axes, scaled
                      -1 to 1.
      \param button   Array with boolean values for the buttons.
      \param elapsed  The elapsed time simce the last update. If 0, 
                      this means a reset. */
  virtual void updateValue(const std::vector<float>& axis, 
			   const std::vector<bool>& button, 
			   int elapsed) = 0;

  /** Return a true or not. */
  virtual bool getTruth() = 0;
  
  /** Return a value */
  virtual float getValue() = 0;
};

/** An object reading a joystick axis. */
class MuStAxisReader: public MuStValueHolder
{
  /** Value place. */
  float value;

public:
  /** Constructor. */
  MuStAxisReader(unsigned int idx);

  /** Destructor */
  virtual ~MuStAxisReader();

  /** Return true if the value is not equal to 0 */
  bool getTruth();
  
  /** Return the value. */
  float getValue();

  /** Refresh the value. */
  void updateValue(const std::vector<float>& axis, 
		   const std::vector<bool>& button, 
		   int elapsed);
};

/** An object reading a joystick button. */
class MuStButtonReader: public MuStValueHolder
{
protected:
  /** Value place. */
  bool value;

public:
  /** Constructor. */
  MuStButtonReader(unsigned int idx);

  /** Destructor */
  virtual ~MuStButtonReader();

  /** Return true if the value is not equal to 0 */
  bool getTruth();
  
  /** Return the value. */
  float getValue();

  /** Refresh the value. */
  virtual void updateValue(const std::vector<float>& axis, 
			   const std::vector<bool>& button, 
			   int elapsed);
};

/** An object reading a joystick axis value, and determining based on
    a minimum and maximum criterion, whether to give a true or false
    signal. The counter in principle counts from 0 to max_value
    (inclusive). Combine with a StepsN object to convert such a range
    to a different set of values. */
class MuStCriterion: public MuStButtonReader
{
  /** Minimum value for criterion. */
  float true_min;

  /** Minimum value for criterion. */
  float true_max;

public:
  /** Constructor. */
  MuStCriterion(unsigned int idx, float true_min, float true_max);

  /** Destructor */
  virtual ~MuStCriterion();

  /** Refresh the value. */
  void updateValue(const std::vector<float>& axis, 
		   const std::vector<bool>& button, 
		   int elapsed);
};

/** An object counting up or down, integer value output. Fellow
    MuStValueHolder objects, which are tested for their truth value,
    are supplied to determine the up or down counting. */
class MuStCounter: public MuStValueHolder
{
  /** Value place. */
  int value;

  /** Maximum value, prevent over feeding. */
  int max_value;

  /** Initial value. */
  int initial_value;

  /** Up counter */
  MultiStickValue* upcounter;
  
  /** Down counter */
  MultiStickValue* downcounter;

  /** Reset button. */
  MultiStickValue* resetter;

  /** Remember which of the counters is pressed. */
  int current_pressed;
  
  /** Repeater, initial delay */
  int repeat_initial;
  
  /** Repeater, following delays */
  int repeat_following;

  /** Repeater status. */
  int repeat_count;

public:
  /** Constructor. */
  MuStCounter(int maxvalue, int initial_value,
	      int repeat_initial, int repeat_following);

  /** set the up counter. */
  bool setUpCounter(MultiStickValue* upcounter);

  /** Set the down counter. */
  bool setDownCounter(MultiStickValue* downc);

  /** Set the resetter. */
  bool setResetter(MultiStickValue* resetter);
  
  /** Destructor */
  virtual ~MuStCounter();

  /** Return true if the value is not equal to 0 */
  bool getTruth();
  
  /** Return the value. */
  float getValue();

  /** Refresh the value. */
  void updateValue(const std::vector<float>& axis, 
		   const std::vector<bool>& button, 
		   int elapsed);
};


/* something to throw when unhappy */
class MultiStickValueDimension: public std::exception
{
public:
  /** Constructor. */
  MultiStickValueDimension() {}

  /** Destructor. */
  virtual ~MultiStickValueDimension() throw() {}

  /** Print out the reason for throwing this. */
  const char* what() const throw() { return "axis/button not available";}
};

#endif
