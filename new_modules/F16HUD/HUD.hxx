// ------------------------------------------------------------------
//
//      item            : HUD.hxx
//      made by         : Clark Borst / Herman Damveld
//      date            : Tue Dec  14 21:24:00 2004
//      category        : header file
//      description     : Head Up Display class that contains
//                      : all the indicator/tape objects
//      changes         : Wed Mar 23 10:25:00 2004 first version
//      language        : C++
//
// ------------------------------------------------------------------

#ifndef HUD_HXX
#define HUD_HXX

#include <GL/gl.h>

// HUD tapes, indicators etc...
#include "SpeedTape.hxx"
#include "PitchLadder.hxx"
#include "AltTape.hxx"
#include "BankIndicator.hxx"
#include "HeadingTape.hxx"
#include "AircraftReference.hxx"

#include "gl2dobjects.hxx"

// This class encapsulates all the indicators, tapes etc...
class HUD: public gl2DObject
{
   public:
      // constructor
      // screenwidth in drawing units, screenheight in drawing units, vert fov in degrees
      HUD(double width, double height, double vfov);

      // destructor
      ~HUD();

      // utility
      void SetScreenWidth(double width); // drawing units
      void SetScreenHeight(double height); // drawing units
      void SetVertFOV(double vfov); // degrees
  inline void SetScale(float s){scale = s;}
      inline double GetScreenWidth(void) {return screenwidth;} // drawing units
      inline double GetScreenHeight(void) {return screenheight;} // drawing units
      inline double GetVertFOV(void) {return vert_fov;} // degrees

      void SetColor(double r, double g, double b);
      void SetLineWidth(double lw);
      inline void ShowOutsideVisual(bool outsidevisual){show_outsidevisual=outsidevisual;}

      // Speed Tape
      inline void ShowSpeedTape(bool speed) {show_speedtape=speed;}
      inline void SetSpeedTapePosition(double x, double y) {theSpeedTape->SetScreenPosition(x, y);}
      inline void SetSpeedTapeSpeedIAS(double speedIAS) {theSpeedTape->SetSpeedIAS(speedIAS);} // m/s

      // Altitude Tape
      inline void ShowAltTape(bool alt) {show_alttape=alt;}
      inline void SetAltTapePosition(double x, double y) {theAltTape->SetScreenPosition(x, y);}
      inline void SetAltTapeAltitude(double alt) {theAltTape->SetAltitude(alt);}  // m

      // pitch ladder utility
      inline void ShowPitchLadder(bool pladder) {show_pitchladder=pladder;}
      inline void SetPitchLadderPosition(double x, double y) {thePitchLadder->SetScreenPosition(x, y);}
      // inline void SetPitchLadderVertFOV(double vert_fov) {thePitchLadder->SetVertFOV(vert_fov);}
      // inline void SetPitchLadderScreenHeight(double draw_height) { thePitchLadder->SetScreenHeight(draw_height);}
      inline void SetPitchLadderPitchAngle(double pitchAngle) {thePitchLadder->SetPitchAngle(pitchAngle);} //rad
      inline void SetPitchLadderRollAngle(double rollAngle) {thePitchLadder->SetRollAngle(rollAngle);} // rad

      // bankindicator utility
      inline void ShowBankIndicator(bool bank) {show_bankindicator=bank;}
      inline void SetBankIndicatorPosition(double x, double y) {theBankIndicator->SetScreenPosition(x, y);}
      inline void SetBankIndicatorRollAngle(double rollAngle) {theBankIndicator->SetRollAngle(rollAngle);} // rad

      // heading indicator utility
      inline void ShowHeadingTape(bool heading) {show_headingtape=heading;}
      inline void SetHeadingTapePosition(double x, double y) {theHeadingTape->SetScreenPosition(x, y);}
      inline void SetHeadingTapeHeading(double heading) {theHeadingTape->SetHeading(heading);} // rad
      inline void SetHeadingTapeHeadingBug(double headingbug) {theHeadingTape->SetHeadingBug(headingbug);} // rad

      // aircraft reference symbol
      inline void ShowAircraftReference(bool aircraftref) {show_aircraftreference=aircraftref;}
      inline void SetAircraftReferencePosition(double x, double y) {theAircraftReference->SetScreenPosition(x, y);}
      // inline void SetAircraftReferenceVertFOV(double vertFOV) {theAircraftReference->SetVertFOV(vertFOV);}  // deg
      // inline void SetAircraftReferenceScreenHeight(double height) {theAircraftReference->SetScreenHeight(height);}  // drawing units
      inline void SetAircraftReferenceGunCrossPitchAngle(double theta0) {theAircraftReference->SetGunCrossPitchAngle(theta0);} // rad
      inline void SetAircraftReferenceNz(double nz) {theAircraftReference->SetNz(nz);} // g's
      inline void SetAircraftReferenceNzMax(double nz_max) {theAircraftReference->SetNzMax(nz_max);} // g's //[HJD 24/04/2006]
      inline void ResetAircraftReferenceNz(void) {theAircraftReference->ResetNz();}
      inline void SetAircraftReferenceSteerpointNo(int no) {theAircraftReference->SetSteerpointNo(no);}
      inline void SetAircraftReferenceSteerpointDist(double dist) {theAircraftReference->SetSteerpointDist(dist);} // m
      inline void SetAircraftReferenceEstimateTimeEnroute(double ETE) {theAircraftReference->SetEstimateTimeEnroute(ETE);} // seconds

      // initialise some stuff
      void InitGL();

      // method to draw the HUD
      void DrawGL();

   protected:
      // width of screen in screen coordinates
      double screenwidth;

      // height of screen in screen coordinates
      double screenheight;

      // vertical field of view of the whole screen in degrees
      double vert_fov;


   protected:
      // lineWidth
      double lineWidth;

      // lineColor
      GLfloat lineColor[3];

  // scale, for resizing windows
  GLfloat scale;

   protected:
      bool show_speedtape;
      bool show_pitchladder;
      bool show_alttape;
      bool show_bankindicator;
      bool show_headingtape;
      bool show_aircraftreference;
      bool show_outsidevisual;

      SpeedTape         *theSpeedTape;
      PitchLadder       *thePitchLadder;
      AltTape           *theAltTape;
      BankIndicator     *theBankIndicator;
      HeadingTape       *theHeadingTape;
      AircraftReference *theAircraftReference;

  // inherited stuff from gl2dobjects
  inline virtual void draw() {DrawGL();}
};

#endif
