/* ------------------------------------------------------------------   */
/*      item            : GUIStick.cxx
        made by         : tux
	from template   : DuecaModuleTemplate.cxx
        template made by: Rene van Paassen
        date            : Wed Apr  2 15:45:37 2014
	category        : body file 
        description     : 
	changes         : Wed Apr  2 15:45:37 2014 first version
	template changes: 030401 RvP Added template creation comment
	                  060512 RvP Modified token checking code
        language        : C++
*/

static const char c_id[] =
"$Id: GUIStick.cxx,v 1.1 2015/04/14 09:22:45 ostroosma Exp $";

#define GUIStick_cxx
// include the definition of the module class
#include "GUIStick.hxx"

// include the debug writing header, by default, write warning and 
// error messages
#define D_MOD
#define I_MOD
#define W_MOD
#define E_MOD
#include <debug.h>

// include additional files needed for your calculation here
// these macros are needed to parse the module's directory when
// looking for the .glade file (also see Makefile)
// eliminates the need for links.script entry
#define STRINGIZE(x) #x
#define STRINGIZE_VALUE_OF(x) STRINGIZE(x)

// the standard package for DUSIME, including template source
#define DO_INSTANTIATE
#include <dueca.h>

// class/module name
const char* const GUIStick::classname = "gui-stick";

// table with GTK callbacks
static GladeCallbackTable sc_table[] = 
  {
    { "thrlscale", "value-changed", gtk_callback(&GUIStick::throttleChanged)},
    { "thrrscale", "value-changed", gtk_callback(&GUIStick::throttleChanged)},
    { NULL, NULL, NULL, NULL }
  };
  
// Parameters to be inserted
const ParameterTable* GUIStick::getMyParameterTable()
{
  static const ParameterTable parameter_table[] = {
    { "set-timing", 
      new MemberCall<GUIStick,TimeSpec>
        (&GUIStick::setTimeSpec), set_timing_description },

    { "check-timing", 
      new MemberCall<GUIStick,vector<int> >
      (&GUIStick::checkTiming), check_timing_description },

    { "primary-controls",
      new VarProbe<GUIStick, bool>
      (&GUIStick::use_pc), "publish PrimaryControls channel"},
    { "secondary-controls",
      new VarProbe<GUIStick, bool>
      (&GUIStick::use_sc), "publish SecondaryControls channel"},
    { "primary-switches",
      new VarProbe<GUIStick, bool>
      (&GUIStick::use_ps), "publish PrimarySwitches channel"},
    { "secondary-switches",
      new VarProbe<GUIStick, bool>
      (&GUIStick::use_ss), "publish SecondarySwitches channel"},

    { "ux-gain",
      new VarProbe<GUIStick, double>
      (&GUIStick::ux_gain), "gain on PC.ux"},
    { "ux-offset",
      new VarProbe<GUIStick, double>
      (&GUIStick::ux_offset), "offset on PC.ux"},
    { "uy-gain",
      new VarProbe<GUIStick, double>
      (&GUIStick::uy_gain), "gain on PC.uy"},
    { "uy-offset",
      new VarProbe<GUIStick, double>
      (&GUIStick::uy_offset), "offset on PC.uy"},
    { "uz-gain",
      new VarProbe<GUIStick, double>
      (&GUIStick::uz_gain), "gain on PC.uz"},
    { "uz-offset",
      new VarProbe<GUIStick, double>
      (&GUIStick::uz_offset), "offset on PC.uz"},
    { "uc-gain",
      new VarProbe<GUIStick, double>
      (&GUIStick::uc_gain), "gain on PC.uc"},
    { "uc-offset",
      new VarProbe<GUIStick, double>
      (&GUIStick::uc_offset), "offset on PC.uc"},

    /* You can extend this table with labels and MemberCall or
       VarProbe pointers to perform calls or insert values into your
       class objects. Please also add a description (c-style string). 

       Note that for efficiency, set_timing_description and
       check_timing_description are pointers to pre-defined strings,
       you can simply enter the descriptive strings in the table. */

    /* The table is closed off with NULL pointers for the variable
       name and MemberCall/VarProbe object. The description is used to
       give an overall description of the module. */
    { NULL, NULL, "generates Primary & Secondary Controls/Switches from GUI"} };

  return parameter_table;
}

// constructor
GUIStick::GUIStick(Entity* e, const char* part, const
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
  Module(e, classname, part),

  // initialize the data you need in your simulation
  myPCWindow(), mySCWindow(), myPSWindow(), mySSWindow(),
  use_pc(false), use_sc(false), use_ps(false), use_ss(false),
  ux_gain(1.0), ux_offset(0.0), uy_gain(1.0), uy_offset(0.0),
  uz_gain(1.0), uz_offset(0.0), uc_gain(1.0), uc_offset(0.0),

  pc_ux(NULL), pc_uy(NULL), pc_uz(NULL), pc_uc(NULL),
  pc_ux_label(NULL), pc_uy_label(NULL), pc_uz_label(NULL), pc_uc_label(NULL),
  sc_sb(NULL), sc_thrl(NULL), sc_thrr(NULL), sc_lock(NULL),
  sc_f0(NULL), sc_f1(NULL), sc_f5(NULL), sc_f15(NULL), sc_f20(NULL),
  sc_f25(NULL), sc_f30(NULL),
  ps_xmit(NULL), ps_trup(NULL), ps_trdwn(NULL), ps_apd(NULL),
  ss_atd(NULL), ss_ignl(NULL), ss_ignr(NULL), 
  ss_up(NULL), ss_off(NULL), ss_dwn(NULL),

  // initialize the data you need for the trim calculation

  // initialize the channel access tokens
  // example
  // my_token(getId(), NameSet(getEntity(), "MyData", part)),
  pc_token(NULL), sc_token(NULL), ps_token(NULL), ss_token(NULL),

  // activity initialization
  myclock(),
  cb1(this, &GUIStick::doCalculation),
  do_calc(getId(), "generating stick inputs from GUI", &cb1, PrioritySpec(0,0))
{
  // do the actions you need for the simulation

  // connect the triggers for simulation
  do_calc.setTrigger(myclock);

}

bool GUIStick::complete()
{
  /* All your parameters have been set. You may do extended
     initialisation here. Return false if something is wrong. */
  bool res = true;
  string gladefile = string(STRINGIZE_VALUE_OF(GUIDIR)) + "guistick.glade";
  
  if(use_pc) {
    res = myPCWindow.readGladeFile(gladefile.c_str(), "pcwindow", 
				   reinterpret_cast<gpointer>(this), 
				   NULL);
    if (res) {
      pc_ux = myPCWindow["uxscale"];
      pc_uy = myPCWindow["uyscale"];
      pc_uz = myPCWindow["uzscale"];
      pc_uc = myPCWindow["ucscale"];
      pc_ux_label = myPCWindow["uxlabel"];
      pc_uy_label = myPCWindow["uylabel"];
      pc_uz_label = myPCWindow["uzlabel"];
      pc_uc_label = myPCWindow["uclabel"];

      myPCWindow.show();
      
      pc_token = new StreamChannelWriteToken<PrimaryControls>
	(getId(), NameSet(getEntity(), "PrimaryControls", getPart()));
      
    } else  {
      return res;
    }
  }

  if(use_sc) {
    res = mySCWindow.readGladeFile(gladefile.c_str(), "scwindow", 
				   reinterpret_cast<gpointer>(this), 
				   sc_table);
    if (res) {
      sc_sb = mySCWindow["sbscale"];
      sc_thrl = mySCWindow["thrlscale"];
      sc_thrr = mySCWindow["thrrscale"];
      sc_lock = mySCWindow["lockcheck"];
      sc_f0 = mySCWindow["flaps0radio"];
      sc_f1 = mySCWindow["flaps1radio"];
      sc_f5 = mySCWindow["flaps5radio"];
      sc_f15 = mySCWindow["flaps15radio"];
      sc_f20 = mySCWindow["flaps20radio"];
      sc_f25 = mySCWindow["flaps25radio"];
      sc_f30 = mySCWindow["flaps30radio"];

      mySCWindow.show();

      sc_token = new StreamChannelWriteToken<SecondaryControls>
	(getId(), NameSet(getEntity(), "SecondaryControls", getPart()));
    } else  {
      return res;
    }
  }

  if(use_ps) {
    res = myPSWindow.readGladeFile(gladefile.c_str(), "pswindow", 
				   reinterpret_cast<gpointer>(this), 
				   NULL);
    if (res) {
      ps_xmit = myPSWindow["xmitcheck"];
      ps_trup = myPSWindow["trimupcheck"];
      ps_trdwn = myPSWindow["trimdowncheck"];
      ps_apd = myPSWindow["apcheck"];

      myPSWindow.show();

      ps_token = new StreamChannelWriteToken<PrimarySwitches>
	(getId(), NameSet(getEntity(), "PrimarySwitches", getPart()));
    } else  {
      return res;
    }
  }

  if(use_ss) {
    res = mySSWindow.readGladeFile(gladefile.c_str(), "sswindow", 
				   reinterpret_cast<gpointer>(this), 
				   NULL);
    if (res) {
      ss_atd = mySSWindow["atcheck"];
      ss_ignl = mySSWindow["ignlcheck"];
      ss_ignr = mySSWindow["ignrcheck"];
      ss_up = mySSWindow["gearupradiobutton"];
      ss_off = mySSWindow["gearoffradiobutton"];
      ss_dwn = mySSWindow["geardownradiobutton"];

      mySSWindow.show();

      ss_token = new StreamChannelWriteToken<SecondarySwitches>
	(getId(), NameSet(getEntity(), "SecondarySwitches", getPart()));
    } else  {
      return res;
    }
  }
  
  return res;
}

// destructor
GUIStick::~GUIStick()
{
  //
}

// as an example, the setTimeSpec function
bool GUIStick::setTimeSpec(const TimeSpec& ts)
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
bool GUIStick::checkTiming(const vector<int>& i)
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
bool GUIStick::isPrepared()
{
  bool res = true;

  if(use_pc && pc_token) CHECK_TOKEN(*pc_token);
  if(use_sc && sc_token) CHECK_TOKEN(*sc_token);
  if(use_ps && ps_token) CHECK_TOKEN(*ps_token);
  if(use_ss && ss_token) CHECK_TOKEN(*ss_token);

  if(use_pc) {
    CHECK_CONDITION(pc_ux);
    CHECK_CONDITION(pc_uy);
    CHECK_CONDITION(pc_uz);
    CHECK_CONDITION(pc_uc);
    CHECK_CONDITION(pc_ux_label);
    CHECK_CONDITION(pc_uy_label);
    CHECK_CONDITION(pc_uz_label);
    CHECK_CONDITION(pc_uc_label);
   }

  if(use_sc) {
    CHECK_CONDITION(sc_sb);
    CHECK_CONDITION(sc_thrl);
    CHECK_CONDITION(sc_thrr);
    CHECK_CONDITION(sc_lock);
    CHECK_CONDITION(sc_f0);
    CHECK_CONDITION(sc_f1);
    CHECK_CONDITION(sc_f5);
    CHECK_CONDITION(sc_f15);
    CHECK_CONDITION(sc_f20);
    CHECK_CONDITION(sc_f25);
    CHECK_CONDITION(sc_f30);
  }

  if(use_ps) {
    CHECK_CONDITION(ps_xmit);
    CHECK_CONDITION(ps_trup);
    CHECK_CONDITION(ps_trdwn);
    CHECK_CONDITION(ps_apd);
  }

  if(use_ss) {
    CHECK_CONDITION(ss_atd);
    CHECK_CONDITION(ss_ignl);
    CHECK_CONDITION(ss_ignr);
    CHECK_CONDITION(ss_up);
    CHECK_CONDITION(ss_off);
    CHECK_CONDITION(ss_dwn);
  }

  // return result of checks
  return res;
}

// start the module
void GUIStick::startModule(const TimeSpec &time)
{
  do_calc.switchOn(time);
}

// stop the module
void GUIStick::stopModule(const TimeSpec &time)
{
  do_calc.switchOff(time);
}

// this routine contains the main simulation process of your module. You 
// should read the input channels here, and calculate and write the 
// appropriate output
void GUIStick::doCalculation(const TimeSpec& ts)
{
  // PrimaryControls
  if(use_pc && pc_token) {
    StreamWriter<PrimaryControls> pc(*pc_token, ts);
    
    pc.data().ux = gtk_range_get_value(GTK_RANGE(pc_ux))*ux_gain + ux_offset;
    pc.data().uy = gtk_range_get_value(GTK_RANGE(pc_uy))*uy_gain + uy_offset;
    pc.data().uz = gtk_range_get_value(GTK_RANGE(pc_uz))*uz_gain + uz_offset;
    pc.data().uc = gtk_range_get_value(GTK_RANGE(pc_uc))*uc_gain + uc_offset;

    pc.data().Sx = 0.0;
    pc.data().Sy = 0.0;
    pc.data().Sz = 0.0;
    pc.data().Sc = 0.0;
    pc.data().dSx = 0.0;
    pc.data().dSy = 0.0;
    pc.data().dSz = 0.0;
    pc.data().dSc = 0.0;
    pc.data().Mx = 0.0;
    pc.data().My = 0.0;
    pc.data().Mz = 0.0;
    pc.data().Mc = 0.0;

    pc.data().fbrake_left = 0.0;
    pc.data().fbrake_right = 0.0;
    pc.data().test = 0;

#ifdef GTK_LABELS_ACTUALLY_WORK
    {
      snprintf(ux_string, 31, "%.4f", pc.data().ux);
      gtk_label_set_text(GTK_LABEL(pc_ux_label), ux_string);
    }
    {
      snprintf(uy_string, 31, "%.4f", pc.data().uy);
      gtk_label_set_text(GTK_LABEL(pc_uy_label), uy_string);
    }
    {
      snprintf(uz_string, 31, "%.4f", pc.data().uz);
      gtk_label_set_text(GTK_LABEL(pc_uz_label), uz_string);
    }
    {
      snprintf(uc_string, 31, "%.4f", pc.data().uc);
      gtk_label_set_text(GTK_LABEL(pc_uc_label), uc_string);
    }
#endif
    //D_MOD(classname << " Tx " << pc.data());
  }
  
  // SecondaryControls
  if(use_sc && sc_token) {
    StreamWriter<SecondaryControls> sc(*sc_token, ts);

    sc.data().throttle_left = gtk_range_get_value(GTK_RANGE(sc_thrl));
    sc.data().throttle_right = gtk_range_get_value(GTK_RANGE(sc_thrr));
    sc.data().speedbrake = gtk_range_get_value(GTK_RANGE(sc_sb));
    if(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(sc_f0)))
       sc.data().flap_setting = 0.0;
    if(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(sc_f1)))
       sc.data().flap_setting = 1.0;
    if(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(sc_f5)))
       sc.data().flap_setting = 5.0;
    if(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(sc_f15)))
       sc.data().flap_setting = 15.0;
    if(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(sc_f20)))
       sc.data().flap_setting = 20.0;
    if(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(sc_f25)))
       sc.data().flap_setting = 25.0;
    if(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(sc_f30)))
       sc.data().flap_setting = 30.0;

    //D_MOD(classname << " Tx " << sc.data());
  }

  // PrimarySwitches
  if(use_ps && ps_token) {
    StreamWriter<PrimarySwitches> ps(*ps_token, ts);
    
    ps.data().xmit = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(ps_xmit));
    ps.data().trim_up = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(ps_trup));
    ps.data().trim_down = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(ps_trdwn));
    ps.data().AP_disconnect = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(ps_apd));

    //D_MOD(classname << " Tx " << ps.data());
  }

  if(use_ss && ss_token) {
    StreamWriter<SecondarySwitches> ss(*ss_token, ts);

    ss.data().AT_disconnect = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(ss_atd));
    ss.data().ignition_left = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(ss_ignl));
    ss.data().ignition_right = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(ss_ignr));
    ss.data().gear_up = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(ss_up));
    ss.data().gear_neutral = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(ss_off));
    ss.data().gear_down = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(ss_dwn));
  
    //D_MOD(classname << " Tx " << ss.data());
  }
} 

// GTK Callbacks
void GUIStick::throttleChanged(GtkWidget *w, gpointer gp)
{
  // only relevant if lock is engaged
  if(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(sc_lock))) {
    // and there's a difference between left and right
    if(fabs(gtk_range_get_value(GTK_RANGE(sc_thrl)) -
	    gtk_range_get_value(GTK_RANGE(sc_thrr))) > 9e-3) {
      // copy the changed one into the other one
      if(w == sc_thrl) {
	gtk_range_set_value(GTK_RANGE(sc_thrr), gtk_range_get_value(GTK_RANGE(sc_thrl)));
	//D_MOD(classname << " left to right ");
      }
      else if(w == sc_thrr) {
	gtk_range_set_value(GTK_RANGE(sc_thrl), gtk_range_get_value(GTK_RANGE(sc_thrr)));
 	//D_MOD(classname << " right to left ");
      }
    }
  }
}

// Make a TypeCreator object for this module, the TypeCreator
// will check in with the scheme-interpreting code, and enable the
// creation of modules of this type
static TypeCreator<GUIStick> a(GUIStick::getMyParameterTable());

