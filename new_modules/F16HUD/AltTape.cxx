#include <GL/glut.h>
#include <iostream>
#include <math.h>
#include <cstdio>

#include "AltTape.hxx"

using namespace std;


AltTape::AltTape():
   x_screenpos (359.0),
   y_screenpos (  0.0),
   lineWidth   (  1.0),
   altitude    (  0.0),
   altitude_bug(  0.0)
{
   lineColor[0] = 0.0;
   lineColor[1] = 1.0;
   lineColor[2] = 0.0;
   cout << "AltTape: constructor called" << endl;
}


AltTape::~AltTape()
{
   cout << "AltTape: destructor called" << endl;
}


void AltTape::DrawGL()
{
   // convert meters to feet:
   GLdouble alt_ft = altitude*3.28084;

   char temp[10];
   int icount,jcount;

   // save current matrix
   glPushMatrix();
      glTranslatef(x_screenpos, y_screenpos, 0.0);

      // enable clipping planes
      GLdouble eqn[4];

      // set bottom clipping plane, bottom of alt tape
      eqn[0] =   0.0;   //   x
      eqn[1] =   1.0;   //   y
      eqn[2] =   0.0;   //   z
      eqn[3] = 143.0;   //   D
      glClipPlane(GL_CLIP_PLANE0,eqn);

      // set top clipping plane, top of alt tape
      eqn[0] =   0.0;   //   x
      eqn[1] =  -1.0;   //   y
      eqn[2] =   0.0;   //   z
      eqn[3] = 143.0;   //   D
      glClipPlane(GL_CLIP_PLANE1,eqn);

      // enable clipping planes
      // set bottom clipping plane, just above alt indicator
      eqn[0] =   0.0;   //   x
      eqn[1] =   1.0;   //   y
      eqn[2] =   0.0;   //   z
      eqn[3] = -20.0;   //   D
      glClipPlane(GL_CLIP_PLANE2,eqn);

      // enable clipping planes
      // set top clipping plane, just below alt indicator
      eqn[0] =   0.0;   //   x
      eqn[1] =  -1.0;   //   y
      eqn[2] =   0.0;   //   z
      eqn[3] = -20.0;   //   D
      glClipPlane(GL_CLIP_PLANE3,eqn);


      // enable blending
      glEnable(GL_BLEND);
      glEnable(GL_LINE_SMOOTH);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
      
      // draw tape
      // big ticks every 500 ft
      // small ticks every 100 ft;

      // get modulus of alt_ft:
      double mod = fmod(alt_ft, 500.0);
      // double fivehundredft = (alt_ft - mod)/500.0;

      // cout << "altitude:      " << altitude << endl;
      // cout << "alt_ft:        " << alt_ft << endl;
      // cout << "mod:           " << mod << endl;
      // cout << "fivehundredft: " << fivehundredft << endl;

      // store current matrix
      glPushMatrix();

         // draw tape ticks
         // units per altitude interval: 130 units cover 500 ft
         // hence 0.26 units per ft

         glEnable(GL_CLIP_PLANE0);
         glEnable(GL_CLIP_PLANE1);

         glTranslatef(0.0, -mod*0.26, 0.0);
         
         glLineWidth(lineWidth);
         glColor3fv(lineColor);
         glBegin(GL_LINES);
            for (icount=0; icount<5; icount++)
            {
               glVertex2f( 0.0, -260.0+icount*130.0);
               glVertex2f(14.0, -260.0+icount*130.0);
               for (jcount=0; jcount<5; jcount++)
               {
                  glVertex2f( 0.0, -260.0+icount*130.0+jcount*26.0);
                  glVertex2f( 9.0, -260.0+icount*130.0+jcount*26.0);
               }
            }
         glEnd();

         // disable clipping planes
         glDisable(GL_CLIP_PLANE0);
         glDisable(GL_CLIP_PLANE1);


         // draw tape digits
         // each fivehundred kts
         // 2000 ft -> 02,0

         glEnable(GL_CLIP_PLANE1);
         glEnable(GL_CLIP_PLANE2);
         
         for (icount=0; icount<5; icount++)
         {
            double firstdigits = (alt_ft + icount*500.0 - 1000.0 - fmod(alt_ft + icount*500.0 - 1000.0, 1000.0))/1000.0;
            double lastdigit = fmod((alt_ft + icount*500.0 - 1000.0 - fmod(alt_ft + icount*500.0 - 1000.0, 500.0)), 1000.0)/100.0;
            snprintf(temp, 10, "%02d,%01d", (int)firstdigits, (int)lastdigit );
            strokeString(24.0, -262.0+icount*130.0, temp, GLUT_STROKE_ROMAN, 0.15);
         }

         glDisable(GL_CLIP_PLANE1);
         glDisable(GL_CLIP_PLANE2);


         glEnable(GL_CLIP_PLANE0);
         glEnable(GL_CLIP_PLANE3);

         for (icount=0; icount<5; icount++)
         {
            double firstdigits = (alt_ft + icount*500.0 - 1000.0 - fmod(alt_ft + icount*500.0 - 1000.0, 1000.0))/1000.0;
            double lastdigit = fmod((alt_ft + icount*500.0 - 1000.0 - fmod(alt_ft + icount*500.0 - 1000.0, 500.0)), 1000.0)/100.0;
            snprintf(temp, 10, "%02d,%01d", (int)firstdigits, (int)lastdigit );
            strokeString(24.0, -262.0+icount*130.0, temp, GLUT_STROKE_ROMAN, 0.15);
         }

         // disable clipping planes
         glDisable(GL_CLIP_PLANE0);
         glDisable(GL_CLIP_PLANE3);

      glPopMatrix();   // restore
      
      glPushMatrix();

         // draw altitude indicator outline
         glLineWidth(lineWidth);
         glColor3fv(lineColor);
         glBegin(GL_LINE_LOOP);
            glVertex2f(103.0,  14.0);
            glVertex2f(29.0,  14.0);
            glVertex2f(20.0,   0.0);
            glVertex2f(29.0, -14.0);
            glVertex2f(103.0, -14.0);
            glVertex2f(103.0,  14.0);
         glEnd();

   
         // draw altitude indicator zero line
         glLineWidth(lineWidth);
         glColor3fv(lineColor);
         glBegin(GL_LINE_LOOP);
            glVertex2f(-17.0,   0.0);
            glVertex2f(-51.0,   0.0);
         glEnd();


         // show altitude
         mod = fmod(alt_ft, 1000.0);
         double thousands = (alt_ft-mod)/1000.0;
         mod = mod - fmod(mod, 10.0);
         glColor3fv(lineColor);
         snprintf(temp, 10, "%2d,%03d", (int)thousands, (int)mod);
         strokeString(100.0, -9.0, temp, GLUT_STROKE_ROMAN, 0.15, 1);


         // draw radio altitude outline
         glLineWidth(lineWidth);
         glColor3fv(lineColor);
         glBegin(GL_LINE_LOOP);
            glVertex2f( -22.0, -158.0);
            glVertex2f(  44.0, -158.0);
            glVertex2f(  44.0, -189.0);
            glVertex2f( -22.0, -189.0);
            glVertex2f( -22.0, -158.0);
         glEnd();
         strokeString(-50.0, -182.0, "AR", GLUT_STROKE_ROMAN, 0.15);
         snprintf(temp, 10, "%4.0f", (alt_ft-fmod(alt_ft, 10.0)));
         strokeString( 40.0, -182.0, temp, GLUT_STROKE_ROMAN, 0.15, 1);


         // draw altitude low indicator
         strokeString(-50.0, -222.0, "AL", GLUT_STROKE_ROMAN, 0.15);
         snprintf(temp, 10, "%4.0f", 300.0);
         strokeString( 40.0, -222.0, temp, GLUT_STROKE_ROMAN, 0.15, 1);
   
      glPopMatrix();   // restore

      //   disable blending
      glDisable(GL_BLEND);
      glDisable(GL_LINE_SMOOTH);

   // final restore of MODELVIEW matrix
   glPopMatrix();   // restore
}


void AltTape::strokeString(double x, double y, char *msg, void *font, double size)
{
   glPushMatrix();
      GLboolean glBlendEnabled = glIsEnabled(GL_BLEND);
      GLboolean glLineSmoothEnabled = glIsEnabled(GL_LINE_SMOOTH);

      glTranslatef(x, y, 0);
      glScalef(size, size, size);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
      glEnable(GL_BLEND);
      glEnable(GL_LINE_SMOOTH);
      glLineWidth(1.5*lineWidth);
      while (*msg)
      {
         glutStrokeCharacter(font, *msg);
         msg++;
      }

      if (glBlendEnabled == GL_FALSE)
      {
         glDisable(GL_BLEND);
      }
      if (glLineSmoothEnabled == GL_FALSE)
      {
         glDisable(GL_LINE_SMOOTH);
      }
      glLineWidth(lineWidth);
   glPopMatrix();
}


void AltTape::strokeString(double x, double y, char *msg, void *font, double size, int align)
{
   glPushMatrix();
      GLboolean glBlendEnabled = glIsEnabled(GL_BLEND);
      GLboolean glLineSmoothEnabled = glIsEnabled(GL_LINE_SMOOTH);

      glTranslatef(x, y, 0);
      glScalef(size, size, size);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
      glEnable(GL_BLEND);
      glEnable(GL_LINE_SMOOTH);
      glLineWidth(1.5*lineWidth);

      char *msgtwo = msg;
      float textwidth = 0.0;
      while (*msgtwo)
      {
         textwidth = textwidth + 80.0;
         msgtwo++;
      }
      glTranslatef(-textwidth, 0.0, 0.0);

      float charwidth = 0.0;
      while (*msg)
      {
         charwidth = glutStrokeWidth(font, (*msg));
         glutStrokeCharacter(font, *msg);
         glTranslatef(-charwidth+80, 0.0, 0.0);
         msg++;
      }

      if (glBlendEnabled == GL_FALSE)
      {
         glDisable(GL_BLEND);
      }
      if (glLineSmoothEnabled == GL_FALSE)
      {
         glDisable(GL_LINE_SMOOTH);
      }
      glLineWidth(lineWidth);
   glPopMatrix();
}
