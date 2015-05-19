// ------------------------------------------------------------------
//
//      item            : HeadingTape.hxx
//      made by         : Clark Borst / Herman Damveld
//      date            : Tue Dec  14 21:24:00 2004
//      category        : header file
//      description     : Heading tape class
//      changes         : Wed Mar 23 10:25:00 2004 first version
//      language        : C++
//
// ------------------------------------------------------------------

#ifndef HEADINGTAPE_HXX
#define HEADINGTAPE_HXX

class HeadingTape
{
   public:
      // ctor
      HeadingTape();

      // dtor
      ~HeadingTape();

      // utility
      inline void SetScreenHeight(double height){}
      inline void SetScreenWidth(double width){}
      inline void SetVertFOV(double vfov){}

      inline void SetScreenPosition(float x, float y){x_screenpos = x; y_screenpos = y;}
      inline void SetLineWidth(double lw){lineWidth = lw;}
      inline void SetLineColor(double r, double g, double b){lineColor[0] = (GLfloat)r; lineColor[1] = (GLfloat)g; lineColor[2] = (GLfloat)b;}
      inline void SetHeading(float heading_rad){heading = heading_rad;}
      inline void SetHeadingBug(float heading_bug_rad){heading_bug = heading_bug_rad;}

      // opengl display
      void DrawGL();

   private:
      void  strokeString(double , double , char *, void *, double);

      double Rad2Deg(double);
      double Deg2Rad(double);

   private:
      double x_screenpos;
      double y_screenpos;
      
      double lineWidth;
      GLfloat lineColor[3];

      double heading;     // rad
      double heading_bug; // rad
};

#endif
