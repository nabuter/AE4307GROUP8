/* ------------------------------------------------------------------   */
/*      item            : MuStValueHolder.cxx
        made by         : Rene' van Paassen
        date            : 031205
	category        : body file 
        description     : 
	changes         : 031205 first version
        language        : C++
*/

const static char c_id[] =
"$Id: MuStValueHolder.cxx,v 1.1 2015/04/14 09:22:45 ostroosma Exp $";

#define MuStValueHolder_cxx
#include "MuStValueHolder.hxx"
#include "MultiStickValue.hxx"

/* Base value holder class ------------------------------------------ */
MuStValueHolder::MuStValueHolder(unsigned int idx) :
  idx(idx)
{
  //
}

MuStValueHolder::~MuStValueHolder()
{
  //
}

/* Axis reader class, passes on a value read from a js axis --------- */
MuStAxisReader::MuStAxisReader(unsigned int idx) :
  MuStValueHolder(idx),
  value(0.0)
{
  //
}

MuStAxisReader::~MuStAxisReader()
{
  //
}

bool MuStAxisReader::getTruth()
{
  return value != 0.0f;
}
  
float MuStAxisReader::getValue()
{
  return value;
}

void MuStAxisReader::updateValue(const std::vector<float>& axis, 
				 const std::vector<bool>& button, 
				 int elapsed)
{
  if (idx >= axis.size()) {
    throw MultiStickValueDimension();
  }
  value = axis[idx];
}

/* Button reader class, passes on a true/false from a button --------- */
MuStButtonReader::MuStButtonReader(unsigned int idx) :
  MuStValueHolder(idx),
  value(false)
{
  //
}

MuStButtonReader::~MuStButtonReader()
{
  //
}

bool MuStButtonReader::getTruth()
{
  return value;
}
  
float MuStButtonReader::getValue()
{
  return value? 1.0f : 0.0f;
}

void MuStButtonReader::updateValue(const std::vector<float>& axis, 
				   const std::vector<bool>& button, 
				   int elapsed)
{
  if (idx > button.size()) {
    throw MultiStickValueDimension();
  }
  value = button[idx];
}

MuStCriterion::MuStCriterion(unsigned int idx, 
			     float true_min, float true_max) :
  MuStButtonReader(idx),
  true_min(true_min), 
  true_max(true_max)
{
  //
}

MuStCriterion::~MuStCriterion()
{
  //
}

void MuStCriterion::updateValue(const std::vector<float>& axis, 
				const std::vector<bool>& button, 
				int elapsed)
{
  if (idx > axis.size()) {
    throw MultiStickValueDimension();
  }
  value = axis[idx] >= true_min && axis[idx] < true_max;
}
  
MuStCounter::MuStCounter(int maxvalue, int initial_value,
			 int repeat_initial, int repeat_following) :
  MuStValueHolder(0),
  value(initial_value),
  max_value(maxvalue),
  initial_value(initial_value),
  upcounter(NULL), 
  downcounter(NULL),
  resetter(NULL),
  current_pressed(0),
  repeat_initial(repeat_initial),
  repeat_following(repeat_following),
  repeat_count(0)
{
  //
}
  
MuStCounter::~MuStCounter()
{
  delete upcounter;
  delete downcounter;
  delete resetter;
}

bool MuStCounter::getTruth()
{
  return value != 0;
}
  
float MuStCounter::getValue()
{
  return float(value);
}

void MuStCounter::updateValue(const std::vector<float>& axis, 
			      const std::vector<bool>& button, 
			      int elapsed)
{
  // update the children's representation
  if (upcounter) upcounter->update(axis, button, elapsed);
  if (downcounter) downcounter->update(axis, button, elapsed);
  if (resetter) resetter->update(axis, button, elapsed);

  if (elapsed == 0) {
    
    // for a reset, initialize
    current_pressed = 0;
    value = initial_value;
  }
  
  if (upcounter != NULL && upcounter->getBool()) {
    if (current_pressed == 1) {
      repeat_count -= elapsed;
      if (repeat_initial && repeat_count <= 0) {
	value++; repeat_count += repeat_following;
      }
    } else {
      value++;
      current_pressed = 1;
      repeat_count = repeat_initial;
    }
  } 
  else if (downcounter != NULL && downcounter->getBool()) {
    if (current_pressed == 2) {
      repeat_count -= elapsed;
      if (repeat_initial && repeat_count <= 0) {
	value--; repeat_count += repeat_following;
      }
    } else {
      value--;
      current_pressed = 2;
      repeat_count = repeat_initial;
    }
  }
  else if (resetter != NULL && resetter->getBool()) {
    value = initial_value;
    current_pressed = 3;
  }
  else {
    current_pressed = 0;
  }
  value = max(min(value, max_value), 0);
}
      
bool MuStCounter::setUpCounter(MultiStickValue* upc)
{
  if (upcounter != NULL) return false;
  upcounter = upc;
  return true;
}

bool MuStCounter::setDownCounter(MultiStickValue* downc)
{
  if (downcounter != NULL) return false;
  downcounter = downc;
  return true;
}

bool MuStCounter::setResetter(MultiStickValue* res)
{
  if (resetter != NULL) return false;
  resetter = res;
  return true;
}
