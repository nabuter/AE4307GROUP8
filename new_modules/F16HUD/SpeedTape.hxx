// ------------------------------------------------------------------
//
//      item            : SpeedTape.hxx
//      made by         : Clark Borst / Herman Damveld
//      date            : Tue Dec  14 21:24:00 2004
//      category        : header file
//      description     : Speed tape class
//      changes         : Wed Mar 23 10:25:00 2004 first version
//      language        : C++
//
// ------------------------------------------------------------------

#ifndef SPEEDTAPE_HXX
#define SPEEDTAPE_HXX

class SpeedTape
{
   public:
      // ctor
      SpeedTape();

      // dtor
      ~SpeedTape();

      // utility
      inline void SetScreenWidth(double width){};
      inline void SetScreenHeight(double height){};
      inline void SetVertFOV(double vfov){};

      inline void SetScreenPosition(double x, double y){x_screenpos = x; y_screenpos = y;}
      inline void SetLineWidth(double lw){lineWidth=lw;}
      inline void SetLineColor(double r, double g, double b){lineColor[0] = (GLfloat)r; lineColor[1] = (GLfloat)g; lineColor[2] = (GLfloat)b;}

      inline void SetSpeedIAS(double speed){speed_ms = speed;} // in m/s
      inline void SetSpeedBug(double speed_bug){speed_bug_ms = speed_bug;} // in m/s

      // opengl display
      void DrawGL();

   private:
      void strokeString(double , double , char *, void *, double);

      double x_screenpos;
      double y_screenpos;

      double lineWidth;
      GLfloat lineColor[3];

      // indicated airspeed in m/s !!
      double speed_ms; 
   
      // indicated airspeed in m/s !!
      double speed_bug_ms;
};

#endif
