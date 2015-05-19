/* ------------------------------------------------------------------   */
/*      item            : SignalGenerator.cxx
        made by         : nabutergrebe
	from template   : DusimeModuleTemplate.cxx
        template made by: Rene van Paassen
        date            : Thu May  7 13:57:33 2015
	category        : body file 
        description     : 
	changes         : Thu May  7 13:57:33 2015 first version
	template changes: 030401 RvP Added template creation comment
	                  060512 RvP Modified token checking code
        language        : C++
*/

#define SignalGenerator_cxx
// include the definition of the module class
#include "SignalGenerator.hxx"

// include the debug writing header, by default, write warning and 
// error messages
#define W_MOD
#define E_MOD
#define D_MOD
#include <debug.h>

// include additional files needed for your calculation here

// the standard package for DUSIME, including template source
#define DO_INSTANTIATE
#include <dusime.h>

// class/module name
const char* const SignalGenerator::classname = "signal-generator";

// initial condition/trim table
const IncoTable* SignalGenerator::getMyIncoTable()
{
  static IncoTable inco_table[] = {
    // enter pairs of IncoVariable and VarProbe pointers (i.e. 
    // objects made with new), in this table.
    // For example
//    {(new IncoVariable("example", 0.0, 1.0, 0.01))
//     ->forMode(FlightPath, Constraint)
//     ->forMode(Speed, Control),
//     new VarProbe<SignalGenerator,double>
//       (REF_MEMBER(&SignalGenerator::i_example))}
    
    // always close off with:
    { NULL, NULL} };

  return inco_table;
}

// parameters to be inserted
const ParameterTable* SignalGenerator::getMyParameterTable()
{
  static const ParameterTable parameter_table[] = {
    { "set-timing", 
      new MemberCall<SignalGenerator,TimeSpec>
        (&SignalGenerator::setTimeSpec), set_timing_description },

    { "check-timing", 
      new MemberCall<SignalGenerator,vector<int> >
      (&SignalGenerator::checkTiming), check_timing_description },

    /* You can extend this table with labels and MemberCall or
       VarProbe pointers to perform calls or insert values into your
       class objects. Please also add a description (c-style string).  

       Note that for efficiency, set_timing_description and
       check_timing_description are pointers to pre-defined strings,
       you can simply enter the descriptive strings in the table. */

    /* The table is closed off with NULL pointers for the variable
       name and MemberCall/VarProbe object. The description is used to
       give an overall description of the module. */
	{"Amplitude",
	new VarProbe<SignalGenerator, float >
	(&SignalGenerator::A),
	"Amplitude of the oscillation"
	},
	{"Frequency",
	new VarProbe<SignalGenerator, float >
	(&SignalGenerator::f),
	"Frequency of the oscillation"
	},



    { NULL, NULL, "please give a description of this module"} };

  return parameter_table;
}

// constructor
SignalGenerator::SignalGenerator(Entity* e, const char* part, const
		       PrioritySpec& ps) :
  /* The following line initialises the SimulationModule base class. 
     You always pass the pointer to the entity, give the classname and the 
     part arguments.
     If you give a NULL pointer instead of the inco table, you will not be 
     called for trim condition calculations, which is normal if you for 
     example implement logging or a display.
     If you give 0 for the snapshot state, you will not be called to 
     fill a snapshot, or to restore your state from a snapshot. Only 
     applicable if you have no state. */
  SimulationModule(e, classname, part, getMyIncoTable(), 0),

  // initialize the data you need in your simulation
mySine(0),t(0),
myColourCounter(0),
  // initialize the data you need for the trim calculation

  // initialize the channel access tokens
  // example
  // my_token(getId(), NameSet(getEntity(), "MyData", part)),




  // activity initialization
  SSwritestreamtoken(getId(), NameSet(getEntity(), "SignalStream", part)),
  DCEwriteeventtoken(getId(), NameSet(getEntity(), "DisplayColourEvent", part)),
  myclock(),
  cb1(this, &SignalGenerator::doCalculation),
  do_calc(getId(), "generates and send signal", &cb1, ps)
{
  // do the actions you need for the simulation

  // connect the triggers for simulation
  do_calc.setTrigger( myclock );
  
  // connect the triggers for trim calculation. Leave this out if you 
  // don not need input for trim calculation 
  //trimCalculationCondition(/* fill in your trim triggering channels */);
}

bool SignalGenerator::complete()
{
  /* All your parameters have been set. You may do extended
     initialisation here. Return false if something is wrong. */
  return true;
}

// destructor
SignalGenerator::~SignalGenerator()
{
  //
}

// as an example, the setTimeSpec function
bool SignalGenerator::setTimeSpec(const TimeSpec& ts)
{
  // a time span of 0 is not acceptable
  if (ts.getValiditySpan() == 0) return false;

  // specify the timespec to the activity
  do_calc.setTimeSpec(ts);
  // or do this with the clock if you have it (don't do both!)
  // myclock.changePeriodAndOffset(ts);

  // do whatever else you need to process this in your model
  // hint: ts.getDtInSeconds()

  // return true if everything is acceptable
  return true;
}

// and the checkTiming function
bool SignalGenerator::checkTiming(const vector<int>& i)
{
  if (i.size() == 3) {
    new TimingCheck(do_calc, i[0], i[1], i[2]);
  }
  else if (i.size() == 2) {
    new TimingCheck(do_calc, i[0], i[1]);
  }
  else {
    return false;
  }
  return true;
}

// tell DUECA you are prepared
bool SignalGenerator::isPrepared()
{
  bool res = true;

  // Example checking a token:
  // CHECK_TOKEN(w_somedata);
  CHECK_TOKEN(SSwritestreamtoken);
  CHECK_TOKEN(DCEwriteeventtoken);
  // Example checking anything
  // CHECK_CONDITION(myfile.good());

  // return result of checks
  return res;
}

// start the module
void SignalGenerator::startModule(const TimeSpec &time)
{
  do_calc.switchOn(time);
}

// stop the module
void SignalGenerator::stopModule(const TimeSpec &time)
{
  do_calc.switchOff(time);
}

// fill a snapshot with state data. You may remove this method (and the
// declaration) if you specified to the SimulationModule that the size of 
// state snapshots is zero
void SignalGenerator::fillSnapshot(const TimeSpec& ts, 
                              Snapshot& snap, bool from_trim)
{
  // The most efficient way of filling a snapshot is with an AmorphStore
  // object.
  AmorphStore s(snap.accessData(), snap.getDataSize());

  if (from_trim) {
    // use packData(s, trim_state_variable1); ... to pack your state into 
    // the snapshot
  }
  else {
    // this is a snapshot from the running simulation. Dusime takes care 
    // that no other snapshot is taken at the same time, so you can safely
    // pack the data you copied into (or left into) the snapshot state 
    // variables in here
    // use packData(s, snapshot_state_variable1); ...
  }
}

// reload from a snapshot. You may remove this method (and the
// declaration) if you specified to the SimulationModule that the size of 
// state snapshots is zero
void SignalGenerator::loadSnapshot(const TimeSpec& t, const Snapshot& snap)
{
  // access the data in the snapshot with an AmorphReStore object
  AmorphReStore s(snap.data, snap.data_size);

  // use unPackData(s, real_state_variable1 ); ... to unpack the data 
  // from the snapshot. 
  // You can safely do this, while snapshot loading is going on the 
  // simulation is in HoldCurrent or the activity is stopped. 
}

// this routine contains the main simulation process of your module. You 
// should read the input channels here, and calculate and write the 
// appropriate output
void SignalGenerator::doCalculation(const TimeSpec& ts)
{
  // check the state we are supposed to be in
  switch (getAndCheckState(ts)) {
  case SimulationState::HoldCurrent: {
    // only repeat the output, do not change the model state
	D_MOD("this is debug output(generatorHold)");

	
	D_MOD("this is debug output(generatorHold)");

	
    break;
    }

  case SimulationState::Replay:
  case SimulationState::Advance: {
	D_MOD("this is debug output(generatoradvance)");

    // access the input 
    // example:
    // try {
    //   StreamReader<MyInput> u(input_token, ts);
    //   throttle = u.data().throttle;    
    //   de = u.data().de; ....
    // } 
    // catch(Exception& e) {
    //   // strange, there is no input. Should I try to continue or not?
    // }
    /* The above piece of code shows a block in which you try to catch 
       error conditions (exceptions) to handle the case in which the input
       data is lost. This is not always necessary, if you normally do not
       foresee such a condition, and you don t mind being stopped when
       it happens, forget about the try/catch blocks. */

    	// do the simulation calculations, one step
	t = t + ts.getDtInSeconds();
	mySine = A * sin (2*M_PI*f*t);

	myColourCounter ++;
	if (myColourCounter == 1000 || 2000){
		EventWriter<DisplayColourEvent> myColourEvent(DCEwriteeventtoken, ts);
	
		if (myColourCounter == 1000) {
	  		myColourEvent.data().dayornight =true;
			}
		else {
	  		myColourEvent.data().dayornight = false;
	  		myColourCounter = 0;
		}
	}
	
    break;
    }
  default:
    // other states should never be entered for a SimulationModule, 
    // HardwareModules on the other hand have more states. Throw an
    // exception if we get here, 
    throw CannotHandleState(getId(),GlobalId(), "state unhandled");
  }

  // DUECA applications are data-driven. From the time a module is switched
  // on, it should produce data, so that modules "downstreams" are 
  // activated
  // access your output channel(s)
  // example
  // StreamWriter<MyOutput> y(output_token, ts);
  StreamWriter<SignalStream> mySignalWriter(SSwritestreamtoken, ts);
  mySignalWriter.data().signal = fabs(mySine);

  // write the output into the output channel, using the stream writer
  // y.data().var1 = something; ...

  if (snapshotNow()) {
    // keep a copy of the model state. Snapshot sending is done in the 
    // sendSnapshot routine, later, and possibly at lower priority
    // e.g. 
    // snapshot_state_variable1 = state_variable1; ...
    // (or maybe if your state is very large, there is a cleverer way ...)
  }
} 

void SignalGenerator::trimCalculation(const TimeSpec& ts, const TrimMode& mode)
{
  // read the event equivalent of the input data
  // example
  // EventReader u(i_input_token, ts);

  // using the input, and the data put into your trim variables,
  // calculate the derivative of the state. DO NOT use the state
  // vector of the normal simulation here, because it might be that
  // this is done while the simulation runs!  
  // Some elements in this state derivative are needed as target, copy
  // these out again into trim variables (see you TrimTable

  // trim calculation
  switch(mode) {
  case FlightPath: {
    // one type of trim calculation, find a power setting and attitude
    // belonging to a flight path angle and speed
  }
  break;

  case Speed: {
    // find a flightpath belonging to a speed and power setting (also
    // nice for gliders)
  }
  break;

  case Ground: {
    // find an altitude/attitude belonging to standing still on the 
    // ground, power/speed 0
  }
  break;

  default:
    W_MOD(getId() << " cannot calculate inco mode " << mode);
  break;
  }

  // This works just like a normal calculation, only you provide the
  // steady state value (if your system is stable anyhow). So, if you
  // have other modules normally depending on your output, you should
  // also produce the equivalent output here. 
  // EventWriter<MyOutput> y(output_token, ts);

  // write the output into the output channel, using the EventWriter
  // N.B. you may also use:
  // output_token.sendEvent(ts, new MyOutput(arguments));
  // this is slightly more efficient. However, the EventWriter makes
  // it easier, because the code will look much more like your normal
  // calculation code.

  // now return. The real results from the trim calculation, as you
  // specified them in the TrimTable, will now be collected and sent
  // off for processing.
}

// Make a TypeCreator object for this module, the TypeCreator
// will check in with the scheme-interpreting code, and enable the
// creation of modules of this type
static TypeCreator<SignalGenerator> a(SignalGenerator::getMyParameterTable());

