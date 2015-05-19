#include <GL/glut.h>
#include <iostream>
#include <math.h>

#include "BankIndicator.hxx"

using namespace std;

BankIndicator::BankIndicator():
   x_screenpos(   0.0),
   y_screenpos(-195.0),
   lineWidth  (   1.0),
   rollangle  (   0.0)
{
   lineColor[0] = 0.0;
   lineColor[1] = 1.0;
   lineColor[2] = 0.0;
   cout << "BankIndicator: constructor called" << endl;
}

BankIndicator::~BankIndicator()
{
   cout << "BankIndicator: destructor called" << endl;
}


void BankIndicator::DrawGL()
{
   glEnable(GL_BLEND);
   glEnable(GL_LINE_SMOOTH);
   glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
      
   glPushMatrix();   // save
      glTranslatef(x_screenpos, y_screenpos, 0.0);


      glLineWidth(lineWidth);
      glColor3fv(lineColor);


      // draw bank angle indicator ring
      // 0 deg line
      glPushMatrix();
         glRotatef(0.0, 0.0, 0.0, 0.0);
         glTranslatef(0.0, -170.0, 0.0);
         glBegin(GL_LINES);
            glVertex2f(0.0,   0.0);
            glVertex2f(0.0, -20.0);
         glEnd();
      glPopMatrix();

      // 10 deg line
      glPushMatrix();
         glRotatef(10.0, 0.0, 0.0, 1.0);
         glTranslatef(0.0, -170.0, 0.0);
         glBegin(GL_LINES);
            glVertex2f(0.0,   0.0);
            glVertex2f(0.0, -12.0);
         glEnd();
      glPopMatrix();
      
      // 20 deg line
      glPushMatrix();
         glRotatef(20.0, 0.0, 0.0, 1.0);
         glTranslatef(0.0, -170.0, 0.0);
         glBegin(GL_LINES);
            glVertex2f(0.0,   0.0);
            glVertex2f(0.0, -12.0);
         glEnd();
      glPopMatrix();

      // 30 deg line
      glPushMatrix();
         glRotatef(30.0, 0.0, 0.0, 1.0);
         glTranslatef(0.0, -170.0, 0.0);
         glBegin(GL_LINES);
            glVertex2f(0.0,   0.0);
            glVertex2f(0.0, -20.0);
         glEnd();
      glPopMatrix();

      // 45 deg line
      glPushMatrix();
         glRotatef(45.0, 0.0, 0.0, 1.0);
         glTranslatef(0.0, -170.0, 0.0);
         glBegin(GL_LINES);
            glVertex2f(0.0,   0.0);
            glVertex2f(0.0, -20.0);
         glEnd();
      glPopMatrix();

      // -10 deg line
      glPushMatrix();
         glRotatef(-10.0, 0.0, 0.0, 1.0);
         glTranslatef(0.0, -170.0, 0.0);
         glBegin(GL_LINES);
            glVertex2f(0.0,   0.0);
            glVertex2f(0.0, -12.0);
         glEnd();
      glPopMatrix();
      
      // -20 deg line
      glPushMatrix();
         glRotatef(-20.0, 0.0, 0.0, 1.0);
         glTranslatef(0.0, -170.0, 0.0);
         glBegin(GL_LINES);
            glVertex2f(0.0,   0.0);
            glVertex2f(0.0, -12.0);
         glEnd();
      glPopMatrix();

      // -30 deg line
      glPushMatrix();
         glRotatef(-30.0, 0.0, 0.0, 1.0);
         glTranslatef(0.0, -170.0, 0.0);
         glBegin(GL_LINES);
            glVertex2f(0.0,   0.0);
            glVertex2f(0.0, -20.0);
         glEnd();
      glPopMatrix();

      // -45 deg line
      glPushMatrix();
         glRotatef(-45.0, 0.0, 0.0, 1.0);
         glTranslatef(0.0, -170.0, 0.0);
         glBegin(GL_LINES);
            glVertex2f(0.0,   0.0);
            glVertex2f(0.0, -20.0);
         glEnd();
      glPopMatrix();



      // draw bank angle indication
      glLineWidth(lineWidth);
      glColor3fv(lineColor);

      if (fabs((Rad2Deg(rollangle))) <= 45.0)
      {
         glPushMatrix();
            glRotatef(Rad2Deg(rollangle), 0.0, 0.0, 1.0);
            glTranslatef(0.0, -190.0, 0.0);

            // triangle indication
            glBegin(GL_LINE_LOOP);
               glVertex2f(  0.0,   0.0);
               glVertex2f( 11.0, -20.0);
               glVertex2f( 11.0, -24.0);
               glVertex2f(-11.0, -24.0);
               glVertex2f(-11.0, -20.0);
               glVertex2f(  0.0,   0.0);
            glEnd();
         glPopMatrix(); //restore
      }

   glPopMatrix();   // restore
   
   glDisable(GL_BLEND);
   glDisable(GL_LINE_SMOOTH);
}


double BankIndicator::Rad2Deg(double rad)
{
   return(rad*(180.0/M_PI));
}

double BankIndicator::Deg2Rad(double deg)
{
   return(deg*(M_PI/180.0));
}
