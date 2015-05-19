// ------------------------------------------------------------------
//
//      item            : BankIndicator.hxx
//      made by         : Clark Borst / Herman Damveld
//      date            : Tue Dec  14 21:24:00 2004
//      category        : header file
//      description     : Bank angle with sideslip indicator class
//      changes         : Wed Mar 23 10:25:00 2004 first version
//      language        : C++
//
// ------------------------------------------------------------------

#ifndef BANKINDICATOR_HXX
#define BANKINDICATOR_HXX

using namespace std;

class BankIndicator
{
   public:
      // ctor
      BankIndicator();

      // dtor
      ~BankIndicator();

      // utility
      inline void SetScreenHeight(double height){}
      inline void SetScreenWidth(double width){}
      inline void SetVertFOV(double vfov){}

      // utility
      inline void SetScreenPosition(double x, double y){x_screenpos = x; y_screenpos = y;}
      inline void SetLineWidth(double lw){lineWidth=lw;}
      inline void SetLineColor(double r, double g, double b){lineColor[0] = (GLfloat)r; lineColor[1] = (GLfloat)g; lineColor[2] = (GLfloat)b;}
      inline void SetRollAngle(double roll_rad){rollangle = roll_rad;}

      // opengl display
      void DrawGL();

   private:
      double Rad2Deg(double);
      double Deg2Rad(double);

   private:
      double x_screenpos;
      double y_screenpos;
      
      double lineWidth;
      GLfloat lineColor[3];

      double rollangle;
};

#endif
