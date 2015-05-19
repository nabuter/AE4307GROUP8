// ------------------------------------------------------------------
//
//      item            : AltTape.hxx
//      made by         : Clark Borst / Herman Damveld
//      date            : Tue Dec  14 21:24:00 2004
//      category        : header file
//      description     : Altitude tape class
//      changes         : Wed Mar 23 10:25:00 2004 first version
//      language        : C++
//
// ------------------------------------------------------------------

#ifndef ALTTAPE_HXX
#define ALTTAPE_HXX

class AltTape
{
   public:
      // ctor
      AltTape();

      // dtor
      ~AltTape();

      // utility
      inline void SetScreenHeight(double height){}
      inline void SetScreenWidth(double width){}
      inline void SetVertFOV(double vfov){}

      inline void SetScreenPosition(float x, float y){x_screenpos = x; y_screenpos = y;}
      inline void SetLineWidth(double lw){lineWidth=lw;}
      inline void SetLineColor(double r, double g, double b){lineColor[0] = (GLfloat)r; lineColor[1] = (GLfloat)g; lineColor[2] = (GLfloat)b;}
      inline void SetAltitude(float alt_m){altitude = alt_m;}
      inline void SetAltitudeBug(float alt_bug_m){altitude_bug = alt_bug_m;}

      // opengl display
      void DrawGL();

   private:
      void strokeString(double, double, char *, void *, double);
      void strokeString(double, double, char *, void *, double, int);
   
      double x_screenpos;
      double y_screenpos;

      double lineWidth;
      GLfloat lineColor[3];

      double altitude;
      double altitude_bug;
};

#endif
