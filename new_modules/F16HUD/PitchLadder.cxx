// ------------------------------------------------------------------
//
//      item            : Pitchladder.cxx
//      made by         : Clark Borst / Herman Damveld
//      date            : Tue Dec  14 21:24:00 2004
//      category        : source file
//      description     : Pitchladder class
//      changes         : Wed Mar 23 10:25:00 2004 first version
//                      : Thu Sep 15 18.58:00 2005 match true sphere
//      language        : C++
//
// ------------------------------------------------------------------

#include <GL/glut.h>
#include <iostream>
#include <math.h>
#include <cstdio>

#include "PitchLadder.hxx"

using namespace std;

PitchLadder::PitchLadder(double height, double vfov):
   screenheight(height),  // drawing units
   vert_fov    (  vfov),  // deg
   x_screenpos (   0.0),  // drawing units
   y_screenpos (   0.0),  // drawing units
   lineWidth   (   1.0),  // drawing units
   pitch_angle (   0.0),  // rad
   roll_angle  (   0.0)   // rad
{
   lineColor[0] = 0.0;
   lineColor[1] = 1.0;
   lineColor[2] = 0.0;
   cout << "PitchLadder: constructor called" << endl;
}


PitchLadder::~PitchLadder()
{
   cout << "PitchLadder: destructor called" << endl;
}


void PitchLadder::DrawGL()
{
   double distToScreen = (screenheight/2.0) / tan(Deg2Rad(vert_fov)/2.0);

   // rotate z: phi
   GLdouble h_rot = Rad2Deg(roll_angle);


   // plane equation for clipping planes
   GLdouble eqn[4];

   eqn[0] =   0.0;  // x
   eqn[1] =  -1.0;  // y
   eqn[2] =   0.0;  // z
   eqn[3] = 300.0;  // d
   glClipPlane(GL_CLIP_PLANE0, eqn);
      
   eqn[0] =   0.0;  // x
   eqn[1] =   1.0;  // y
   eqn[2] =   0.0;  // z
   eqn[3] = 300.0;  // d
   glClipPlane(GL_CLIP_PLANE1, eqn);

   eqn[0] =   1.0;  // x
   eqn[1] =   0.0;  // y
   eqn[2] =   0.0;  // z
   eqn[3] = 450.0;  // d
   glClipPlane(GL_CLIP_PLANE2, eqn);
      
   eqn[0] =  -1.0;  // x
   eqn[1] =   0.0;  // y
   eqn[2] =   0.0;  // z
   eqn[3] = 450.0;  // d
   glClipPlane(GL_CLIP_PLANE3, eqn);

   // draw pitch ladder
   glEnable(GL_CLIP_PLANE0);
   glEnable(GL_CLIP_PLANE1);
   glEnable(GL_CLIP_PLANE2);
   glEnable(GL_CLIP_PLANE3);


   glEnable(GL_BLEND);
   glEnable(GL_LINE_SMOOTH);
   glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

   // re-define 2-D Modelview matrix
   glPushMatrix();   // save

      // rotate h_rot degrees:
      glRotatef(h_rot, 0.0, 0.0, 1.0);   // rotate

      // cout << "distToScreen: " << distToScreen << endl;
      // cout << "pitch_angle (deg): " << Rad2Deg(pitch_angle) << endl;
      // cout << "h (units): " << distToScreen*tan(-pitch_angle) << endl << endl;

      // horizon line
      glPushMatrix();
         glTranslatef(0.0, distToScreen*tan(-pitch_angle), 0.0);

         glColor3fv(lineColor);
         glLineWidth(lineWidth);
         glBegin(GL_LINES);
            glVertex2f(-335.0, 0.0);
            glVertex2f( -44.0, 0.0);
            glVertex2f(  44.0, 0.0);
            glVertex2f( 335.0, 0.0);
         glEnd();
      glPopMatrix();


      // +5 deg pitch lines
      for (int pitchIncr = 1; pitchIncr<19; pitchIncr++)
      {
         glPushMatrix();
            glTranslatef(0.0, distToScreen*tan(-pitch_angle+Deg2Rad(5.0*pitchIncr)), 0.0);

            glColor3fv(lineColor);   // object color
            glLineWidth(lineWidth);
            glBegin(GL_LINES);
               glVertex2f( -44.0,   0.0);
               glVertex2f(-125.0,   0.0);
               glVertex2f(-125.0,   0.0);
               glVertex2f(-125.0, -22.0);
            glEnd();
            glBegin(GL_LINES);
               glVertex2f(  44.0,   0.0);
               glVertex2f( 125.0,   0.0);
               glVertex2f( 125.0,   0.0);
               glVertex2f( 125.0, -22.0);
            glEnd();

            char temp[10];
            snprintf(temp, 10, "%2.0f", 5.0*pitchIncr);
            strokeString(-182, -10.0, -h_rot, temp, GLUT_STROKE_ROMAN, 0.15);
            strokeString(+158, -10.0, -h_rot, temp, GLUT_STROKE_ROMAN, 0.15);
         glPopMatrix();
      }

      // -5 deg pitch lines
      for (int pitchIncr = 1; pitchIncr<18; pitchIncr++)
      {
         glPushMatrix();
            glTranslatef(0.0, distToScreen*tan(-pitch_angle+Deg2Rad(-5.0*pitchIncr)), 0.0);

            glColor3fv(lineColor);   // object color
            glLineWidth(lineWidth);
            glBegin(GL_LINES);
               glVertex2f( -44.0,  0.0);
               glVertex2f( -58.0,  0.0);
               glVertex2f( -75.0,  0.0);
               glVertex2f( -92.0,  0.0);
               glVertex2f(-109.0,  0.0);
               glVertex2f(-125.0,  0.0);
               glVertex2f(-125.0,  0.0);
               glVertex2f(-125.0, 22.0);
            glEnd();
            glBegin(GL_LINES);
               glVertex2f(  44.0,  0.0);
               glVertex2f(  58.0,  0.0);
               glVertex2f(  75.0,  0.0);
               glVertex2f(  92.0,  0.0);
               glVertex2f( 109.0,  0.0);
               glVertex2f( 125.0,  0.0);
               glVertex2f( 125.0,  0.0);
               glVertex2f( 125.0, 22.0);
            glEnd();

            char temp[10];
            snprintf(temp,10, "%2.0f", -5.0*pitchIncr);
            strokeString(-182, -10.0, -h_rot, temp, GLUT_STROKE_ROMAN, 0.15);
            strokeString(+158, -10.0, -h_rot, temp, GLUT_STROKE_ROMAN, 0.15);
         glPopMatrix();
      }

   // restore
   glPopMatrix();   

   glDisable(GL_BLEND);
   glDisable(GL_LINE_SMOOTH);

   glDisable(GL_CLIP_PLANE0);
   glDisable(GL_CLIP_PLANE1);
   glDisable(GL_CLIP_PLANE2);
   glDisable(GL_CLIP_PLANE3);
}


void PitchLadder::strokeString(double x, double y, char *msg, void *font, double size)
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
         glColor3fv(lineColor);
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


void PitchLadder::strokeString(double x, double y, double rot, char *msg, void *font, double size)
{
   glPushMatrix();
      GLboolean glBlendEnabled = glIsEnabled(GL_BLEND);
      GLboolean glLineSmoothEnabled = glIsEnabled(GL_LINE_SMOOTH);

      glTranslatef(x, y, 0.0);
      glRotatef(rot, 0.0, 0.0, 1.0);   // rotate
      glScalef(size, size, size);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
      glEnable(GL_BLEND);
      glEnable(GL_LINE_SMOOTH);
      glLineWidth(1.5*lineWidth);
      while (*msg)
      {
         glColor3fv(lineColor);
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


double PitchLadder::Rad2Deg(double rad)
{
   return(rad*(180.0/M_PI));
}


double PitchLadder::Deg2Rad(double deg)
{
   return(deg*(M_PI/180.0));
}
