#include <GL/glut.h>
#include <iostream>
#include <math.h>
#include <cstdio>

#ifndef M_PI
#define M_PI 3.1415926535
#endif

#include "AircraftReference.hxx"

using namespace std;

AircraftReference::AircraftReference(double height, double vfov):
   vert_fov      (  vfov),     // deg
   screenheight  (height),
   x_screenpos   (   0.0),
   y_screenpos   (   0.0),
   lineWidth     (   1.0),
   theta0        (   0.0),     // rad
   nz            (   1.0),     // g's
   nz_max        (   1.0),     // g's
   steerpointno  (   1  ),
   steerpointdist(7408.0),     // m
   ETE           ( 100.0)      // seconds
{
   lineColor[0] = 0.0;
   lineColor[1] = 1.0;
   lineColor[2] = 0.0;
   cout << "AircraftReference: constructor called" << endl;
}


AircraftReference::~AircraftReference()
{
   cout << "AircraftReference: destructor called" << endl;
}


void AircraftReference::SetNz(double nz_g)
{
   nz = nz_g;
   if (nz_g > nz_max)
   {
      nz_max = nz_g;
   }
};


void AircraftReference::DrawGL()
{
   char temp[10];
   double distToScreen = (screenheight/2.0) / tan(Deg2Rad(vert_fov)/2.0);

   // enable blending
   glEnable(GL_BLEND);
   glEnable(GL_LINE_SMOOTH);
   glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

   glPushMatrix();
      glTranslatef(x_screenpos, y_screenpos, 0.0);

      // gun cross
      glPushMatrix();
         glTranslatef(0.0, distToScreen*tan(theta0), 0.0);
         glLineWidth(lineWidth);
         glColor3fv(lineColor);
         glBegin(GL_LINES);
            glVertex2f( -11.0,   0.0);
            glVertex2f( -22.0,   0.0);
            glVertex2f(  11.0,   0.0);
            glVertex2f(  22.0,   0.0);
            glVertex2f(   0.0, -11.0);
            glVertex2f(   0.0, -22.0);
            glVertex2f(   0.0,  11.0);
            glVertex2f(   0.0,  22.0);
         glEnd();
      glPopMatrix();

      // nz indicator
      glColor3fv(lineColor);
      glLineWidth(lineWidth);
      snprintf(temp, 10, "%1.1f", nz);
      strokeString(-339, 175.0, temp, GLUT_STROKE_ROMAN, 0.15);

      // nz_max indicator
      glColor3fv(lineColor);
      glLineWidth(lineWidth);
      snprintf(temp, 10, "%1.1f", nz_max);
      strokeString(-379, -235.0, temp, GLUT_STROKE_ROMAN, 0.15);

      // NAV indicator
      glColor3fv(lineColor);
      glLineWidth(lineWidth);
      snprintf(temp, 10, "NAV");
      strokeString(-379, -275.0, temp, GLUT_STROKE_ROMAN, 0.15);

      // Slant range
      glColor3fv(lineColor);
      glLineWidth(lineWidth);
      snprintf(temp, 10, "B%05.1f", (steerpointdist/1852.0));
      strokeString(259, -275.0, temp, GLUT_STROKE_ROMAN, 0.15);

      // ETE
      glColor3fv(lineColor);
      glLineWidth(lineWidth);
      char minussign = ' ';
      double secs = 0.0;
      double mins = 0.0;
      if (ETE < 0.0)
      {
         minussign = '-';
         secs = floor(fmod(ETE, 60.0));
         mins = (ETE-secs)/60.0;
         secs = -secs;
      }
      else
      {
         secs = floor(fmod(ETE, 60.0));
         mins = (ETE-secs)/60.0;
      }
      snprintf(temp, 10, "%c%02.0f:%02.0f", minussign, mins, secs);
      strokeString(259, -315.0, temp, GLUT_STROKE_ROMAN, 0.15);

      // dist to steerpoint > steerpoint no
      glColor3fv(lineColor);
      glLineWidth(lineWidth);
      snprintf(temp, 10, "%03.0f>%02d", (steerpointdist/1852.0), steerpointno);
      strokeString(259, -355.0, temp, GLUT_STROKE_ROMAN, 0.15);

   glPopMatrix();

   glDisable(GL_BLEND);
   glDisable(GL_LINE_SMOOTH);
}


void AircraftReference::strokeString(double x, double y, char *msg, void *font, double size)
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


double AircraftReference::Rad2Deg(double rad)
{
   return(rad*(180.0/M_PI));
}


double AircraftReference::Deg2Rad(double deg)
{
   return(deg*(M_PI/180.0));
}

