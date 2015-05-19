// ------------------------------------------------------------------
//
//      item            : AircraftReference.hxx
//      made by         : Clark Borst / Herman Damveld
//      date            : Tue Dec  14 21:24:00 2004
//      category        : header file
//      description     : Aircraft reference symbol class
//      changes         : Wed Mar 23 10:25:00 2004 first version
//      language        : C++
//
// ------------------------------------------------------------------

#ifndef AIRCRAFTREFERENCE_HXX
#define AIRCRAFTREFERENCE_HXX

class AircraftReference
{
   public:
      // ctor
      AircraftReference(double height, double vfov);

      // dtor
      ~AircraftReference();

      // utility
      inline void SetScreenHeight(double height){screenheight = height;}  // drawing units
      inline void SetScreenWidth(double width){}
      inline void SetVertFOV(double vfov){vert_fov = vfov;} // deg

      // utility
      inline void SetScreenPosition(double x, double y){x_screenpos = x; y_screenpos = y;}
      inline void SetLineWidth(double lw){lineWidth = lw;}
      inline void SetLineColor(double r, double g, double b){lineColor[0] = (GLfloat)r; lineColor[1] = (GLfloat)g; lineColor[2] = (GLfloat)b;}

      inline void SetGunCrossPitchAngle(double theta_gun_cross_rad){theta0 = theta_gun_cross_rad;} // rad
      void SetNz(double nz_g); // g's
      inline void SetNzMax(double nm) {nz_max = nm;} // g's //[HJD] added 24/04/2006
      inline void ResetNz(void){nz_max = 1.0;}
      inline void SetSteerpointNo(int stpno){steerpointno = stpno;}
      inline void SetSteerpointDist(double stpdist){steerpointdist = stpdist;} // m
      inline void SetEstimateTimeEnroute(double EstTimeEnr){ETE = EstTimeEnr;} // seconds

      // opengl display
      void DrawGL();

   private:
      double Rad2Deg(double);
      double Deg2Rad(double);
      void strokeString(double, double, char*, void*, double);

   private:
      double vert_fov;
      double screenheight;
      double x_screenpos;
      double y_screenpos;

      double lineWidth;
      GLfloat lineColor[3];

      double theta0;          // rad
      double nz;              // g
      double nz_max;          // g
      int    steerpointno;    //
      double steerpointdist;  // m
      double ETE;             // seconds
};

#endif
