// ------------------------------------------------------------------
//
//      item            : Pitchladder.hxx
//      made by         : Clark Borst / Herman Damveld
//      date            : Tue Dec  14 21:24:00 2004
//      category        : header file
//      description     : Pitchladder class
//      changes         : Wed Mar 23 10:25:00 2004 first version
//      language        : C++
//
// ------------------------------------------------------------------

#ifndef PITCHLADDER_HXX
#define PITCHLADDER_HXX

class PitchLadder
{
   public:
      // default constructor
      // screenheight in drawing units, vert fov in degrees
      PitchLadder(double height, double vfov);

      // default destructor
      ~PitchLadder();

      // utility
      inline void SetVertFOV(double vertFOV){vert_fov = vertFOV;} // deg
      inline void SetScreenHeight(double height){screenheight = height;}
      inline void SetScreenWidth(double width){} 
      inline void SetLineWidth(double lw){lineWidth=lw;}
      inline void SetLineColor(double r, double g, double b){lineColor[0] = (GLfloat)r; lineColor[1] = (GLfloat)g; lineColor[2] = (GLfloat)b;}

      inline void SetPitchAngle(double pitch){pitch_angle = pitch;}   // rad
      inline void SetRollAngle(double roll){roll_angle = roll;}       // rad

      inline void SetScreenPosition(double x, double y){x_screenpos = x; y_screenpos = y;}

      // Draw in openGL display
      void DrawGL();

   private:
      void strokeString(double, double, char *, void *, double);
      void strokeString(double, double, double, char *, void *, double);

      double Rad2Deg(double);
      double Deg2Rad(double);

   private:
      double screenheight; // screenheight as used in gluOrtho2D
      double vert_fov;     // in deg

      double x_screenpos;  // in drawing units
      double y_screenpos;  // in drawing untis
      
      double lineWidth;
      GLfloat lineColor[3];

      double pitch_angle;  // in rad
      double roll_angle;   // in rad
};

#endif
