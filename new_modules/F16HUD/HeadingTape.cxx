#include <GL/glut.h>
#include <iostream>
#include <math.h>
#include <cstdio>

#include "HeadingTape.hxx"

using namespace std;

HeadingTape::HeadingTape():
   x_screenpos(   0.0),
   y_screenpos(-200.0),
   lineWidth  (   1.0),
   heading    (   0.0), // rad
   heading_bug(   0.0)  // rad
{
   lineColor[0] = 0.0;
   lineColor[1] = 1.0;
   lineColor[2] = 0.0;
   cout << "HeadingTape: constructor called" << endl;
}

HeadingTape::~HeadingTape()
{
   cout << "HeadingTape: destructor called" << endl;
}

void HeadingTape::DrawGL()
{
   char temp[10];
   GLdouble eqn[4];

   // set right clipping plane, right of heading tape
   eqn[0] =  -1.0;   //   x
   eqn[1] =   0.0;   //   y
   eqn[2] =   0.0;   //   z
   eqn[3] = 130.0;   //   D
   glClipPlane(GL_CLIP_PLANE0,eqn);

   // set left clipping plane, left of heading tape
   eqn[0] =   1.0;   //   x
   eqn[1] =   0.0;   //   y
   eqn[2] =   0.0;   //   z
   eqn[3] = 130.0;   //   D
   glClipPlane(GL_CLIP_PLANE1,eqn);

   // set right clipping plane, left of heading indicator
   eqn[0] =  -1.0;   //   x
   eqn[1] =   0.0;   //   y
   eqn[2] =   0.0;   //   z
   eqn[3] = -28.0;   //   D
   glClipPlane(GL_CLIP_PLANE2,eqn);

   // set left clipping plane, right of heading indicator
   eqn[0] =   1.0;   //   x
   eqn[1] =   0.0;   //   y
   eqn[2] =   0.0;   //   z
   eqn[3] = -28.0;   //   D
   glClipPlane(GL_CLIP_PLANE3,eqn);


   // enable blending
   glEnable(GL_BLEND);
   glEnable(GL_LINE_SMOOTH);
   glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

   glPushMatrix();

      glTranslatef(x_screenpos, y_screenpos, 0.0);

      // draw zero line
      glColor3fv(lineColor);
      glLineWidth(lineWidth);
      glBegin(GL_LINES);
         glVertex2f(0.0,  0.0);
         glVertex2f(0.0, 23.0);
      glEnd();


      // draw heading ticks
      glPushMatrix();

         glTranslatef(-228.0/20.0 * (Rad2Deg(heading) + 360.0), -11.0, 0.0);

         glEnable(GL_CLIP_PLANE0);
         glEnable(GL_CLIP_PLANE1);

         for (int TickCntr = 0; TickCntr < 108; TickCntr++)
         {
            // 10 deg ticks
            glColor3fv(lineColor);
            glLineWidth(lineWidth);
            glBegin(GL_LINES);
               glVertex2f(228.0/20.0 * TickCntr*10.0,   0.0);
               glVertex2f(228.0/20.0 * TickCntr*10.0, -23.0);
            glEnd();

            // 5 deg ticks
            glColor3fv(lineColor);
            glLineWidth(lineWidth);
            glBegin(GL_LINES);
               glVertex2f(228.0/20.0 * (TickCntr*10.0 + 5.0),   0.0);
               glVertex2f(228.0/20.0 * (TickCntr*10.0 + 5.0), -12.0);
            glEnd();
         }

         glDisable(GL_CLIP_PLANE0);
         glDisable(GL_CLIP_PLANE1);


         glEnable(GL_CLIP_PLANE1);
         glEnable(GL_CLIP_PLANE2);

         for (int TickCntr = 0; TickCntr < 108; TickCntr++)
         {
            // 10 deg ticks
            glColor3fv(lineColor);
            int head_tenths = TickCntr%36;
            if (head_tenths == 0) head_tenths = 36;
            snprintf(temp, 10, "%02d", head_tenths);
            strokeString(228.0/20.0 * TickCntr*10.0 - 11.0, -53.0, temp, GLUT_STROKE_ROMAN, 0.15);
         }

         glDisable(GL_CLIP_PLANE1);
         glDisable(GL_CLIP_PLANE2);


         glEnable(GL_CLIP_PLANE0);
         glEnable(GL_CLIP_PLANE3);

         for (int TickCntr = 0; TickCntr < 108; TickCntr++)
         {
            // 10 deg ticks
            glColor3fv(lineColor);
            int head_tenths = TickCntr%36;
            if (head_tenths == 0) head_tenths = 36;
            snprintf(temp, 10, "%02d", head_tenths);
            strokeString(228.0/20.0 * TickCntr*10.0 - 11.0, -53.0, temp, GLUT_STROKE_ROMAN, 0.15);
         }

         glDisable(GL_CLIP_PLANE0);
         glDisable(GL_CLIP_PLANE3);

      glPopMatrix();


      // draw heading indicator + outline
      glPushMatrix();
         glColor3fv(lineColor);
         glLineWidth(lineWidth);
         glBegin(GL_LINE_LOOP);
            glVertex2f( 22.0, -42.0);
            glVertex2f( 22.0, -70.0);
            glVertex2f(-22.0, -70.0);
            glVertex2f(-22.0, -42.0);
            glVertex2f( 22.0, -42.0);
         glEnd();

         glColor3fv(lineColor);

         double head_corr = fmod(Rad2Deg(heading), 360.0);
         if (head_corr < 0.0) head_corr = head_corr + 360.0;
         if (fabs(head_corr) < 0.5) head_corr = 360.0;

         snprintf(temp, 10, "%03.0f", head_corr);
         strokeString(-16.0, -64.0, temp, GLUT_STROKE_ROMAN, 0.15);
      glPopMatrix();

   glDisable(GL_BLEND);
   glDisable(GL_LINE_SMOOTH);

   glPopMatrix();
}


void HeadingTape::strokeString(double x, double y, char *msg, void *font, double size)
{
   glPushMatrix();
      GLboolean glBlendEnabled = glIsEnabled(GL_BLEND);
      GLboolean glLineSmoothEnabled = glIsEnabled(GL_LINE_SMOOTH);

      glTranslatef(x, y, 0.0);
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


double HeadingTape::Rad2Deg(double rad)
{
   return(rad*(180.0/M_PI));
}


double HeadingTape::Deg2Rad(double deg)
{
   return(deg*(M_PI/180.0));
}

