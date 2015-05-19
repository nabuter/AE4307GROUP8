#include <GL/glut.h>
#include <iostream>
#include <math.h>
#include <cstdio>

#include "SpeedTape.hxx"

using namespace std;

SpeedTape::SpeedTape():
   x_screenpos (-359.0),
   y_screenpos    (0.0),
   lineWidth      (1.0),
   speed_ms       (0.0),
   speed_bug_ms   (0.0)
{
   lineColor[0] = 0.0;
   lineColor[1] = 1.0;
   lineColor[2] = 0.0;
   cout << "SpeedTape: constructor called" << endl;
}


SpeedTape::~SpeedTape()
{
   cout << "SpeedTape: destructor called" << endl;
}


void SpeedTape::DrawGL()
{
   // airspeed in kts
   GLdouble speed_kts = (3600.0/1852.0) * speed_ms;

   // plane equation for clipping planes
   GLdouble eqn[4];

   char temp[10];
   int icount, jcount;

   // store current position
   glPushMatrix();

      // define 2-D Modelview matrix
      glTranslatef(x_screenpos, y_screenpos, 0.0);

      // enable clipping planes
      // set bottom clipping plane, bottom of speed tape
      eqn[0] =   0.0;   //   x
      eqn[1] =   1.0;   //   y
      eqn[2] =   0.0;   //   z
      eqn[3] = 143.0;   //   D
      glClipPlane(GL_CLIP_PLANE0,eqn);

      // set top clipping plane, top of speed tape
      eqn[0] =   0.0;   //   x
      eqn[1] =  -1.0;   //   y
      eqn[2] =   0.0;   //   z
      eqn[3] = 143.0;   //   D
      glClipPlane(GL_CLIP_PLANE1,eqn);

      // enable clipping planes
      // set bottom clipping plane, just above speed indicator
      eqn[0] =   0.0;   //   x
      eqn[1] =   1.0;   //   y
      eqn[2] =   0.0;   //   z
      eqn[3] = -20.0;   //   D
      glClipPlane(GL_CLIP_PLANE2,eqn);

      // enable clipping planes
      // set top clipping plane, just below speed indicator
      eqn[0] =   0.0;   //   x
      eqn[1] =  -1.0;   //   y
      eqn[2] =   0.0;   //   z
      eqn[3] = -20.0;   //   D
      glClipPlane(GL_CLIP_PLANE3,eqn);



      // enable blending
      glEnable(GL_LINE_SMOOTH);
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

      // draw tape
      // big ticks at every 50 kts
      // small ticks at every 10 kts

      // get modulus of speed_kts:
      double mod = fmod(speed_kts, 50.0);
      double fiftykts = (speed_kts - mod)/50.0;
      
      // cout << "speed_kts:  "  << speed_kts << endl;
      // cout << "mod:        "  << mod << endl;
      // cout << "fiftykts:   "  << fiftykts << endl << endl;

      // save position
      glPushMatrix();

         // draw tape ticks
         // units per speed interval: 130 units cover 50 kts
         // hence 2.6 units per kt

         glEnable(GL_CLIP_PLANE0);
         glEnable(GL_CLIP_PLANE1);

         glTranslatef(0.0, -mod*2.6, 0.0);
         for (icount=0; icount<5; icount++)
         {
            glColor3fv(lineColor);
            glLineWidth(lineWidth);
            glBegin(GL_LINES);
               glVertex2f(-14.0, -260.0+icount*130.0);
               glVertex2f(  0.0, -260.0+icount*130.0);
            glEnd();

            for (jcount=0; jcount<5; jcount++)
            {
               glColor3fv(lineColor);
               glLineWidth(lineWidth);
               glBegin(GL_LINES);
                  glVertex2f(-9.0, -260.0+icount*130.0+jcount*26.0);
                  glVertex2f( 0.0, -260.0+icount*130.0+jcount*26.0);
               glEnd();
            }
         }
         
         glDisable(GL_CLIP_PLANE0);
         glDisable(GL_CLIP_PLANE1);


         glEnable(GL_CLIP_PLANE1);
         glEnable(GL_CLIP_PLANE2);

         // draw tape digits
         // first two digits at every 50 kts

         for (icount=0; icount<5; icount++)
         {
	   snprintf(temp, 10, "%2.0f", fiftykts*5.0 + icount*5.0 - 10.0);
            strokeString(-44.0, -262+icount*130.0, temp, GLUT_STROKE_ROMAN, 0.15);
         }

         // glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
         // glDisable(GL_BLEND);

         glDisable(GL_CLIP_PLANE1);
         glDisable(GL_CLIP_PLANE2);


         glEnable(GL_CLIP_PLANE0);
         glEnable(GL_CLIP_PLANE3);

         // draw tape digits
         // first two digits at every 50 kts

         for (icount=0; icount<5; icount++)
         {
	   snprintf(temp, 10, "%2.0f", fiftykts*5.0 + icount*5.0 - 10.0);
            strokeString(-44.0, -262+icount*130.0, temp, GLUT_STROKE_ROMAN, 0.15);
         }

         glDisable(GL_CLIP_PLANE0);
         glDisable(GL_CLIP_PLANE3);

      // restore
      glPopMatrix();


      glPushMatrix();
         // draw speed indicator outline
         glColor3fv(lineColor);
         glLineWidth(lineWidth);
         glBegin(GL_LINE_LOOP);
            glVertex2f(-83.0,  14.0);
            glVertex2f(-29.0,  14.0);
            glVertex2f(-20.0,   0.0);
            glVertex2f(-29.0, -14.0);
            glVertex2f(-83.0, -14.0);
            glVertex2f(-83.0,  14.0);
         glEnd();

         // draw speed indicator zero line
         glColor3fv(lineColor);
         glLineWidth(lineWidth);
         glBegin(GL_LINES);
            glVertex2f( 17.0,  0.0);
            glVertex2f( 51.0,  0.0);
         glEnd();

         // show speed
         glColor3fv(lineColor);
         snprintf(temp,10, "%02.0f",speed_kts);
         strokeString(-65.0, -9.0, temp, GLUT_STROKE_ROMAN, 0.15);
   
         // show 'C' calibrated airspeed
         glColor3fv(lineColor);
         snprintf(temp, 10, "C");
         strokeString(23.0, 21.0, temp, GLUT_STROKE_ROMAN, 0.15);

         // show mach
         // for now, assume at sea-level
         glColor3fv(lineColor);
         snprintf(temp,10,"%3.2f", speed_ms/334.0);
         strokeString(20.0, -195.0, temp, GLUT_STROKE_ROMAN, 0.15);


         // draw_speedbug();
         // not implemented yet!
      glPopMatrix();


      // disable blending
      glDisable(GL_BLEND);
      glDisable(GL_LINE_SMOOTH);
   
   // final restore of MODELVIEW matrix
   glPopMatrix();   // restore
}

void SpeedTape::strokeString(double x, double y, char *msg, void *font, double size)
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

      glLineWidth(1.0*lineWidth);
   glPopMatrix();
}

