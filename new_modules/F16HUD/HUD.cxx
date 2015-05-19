#include <GL/glut.h>
#include <iostream>
#include "HUD.hxx"

using namespace std;


HUD::HUD(double width, double height, double vfov):
   screenwidth           (width),
   screenheight          (height),
   vert_fov              (vfov),
   lineWidth             (1.0),
   scale                 (1.0),
   show_speedtape        (true),
   show_pitchladder      (true),
   show_alttape          (true),
   show_bankindicator    (true),
   show_headingtape      (true),
   show_aircraftreference(true),
   show_outsidevisual    (true),
   theSpeedTape          (new SpeedTape()),
   thePitchLadder        (new PitchLadder(screenheight, vert_fov)),
   theAltTape            (new AltTape()),
   theBankIndicator      (new BankIndicator()),
   theHeadingTape        (new HeadingTape()),
   theAircraftReference  (new AircraftReference(screenheight, vert_fov))
{
   lineColor[0] = 0.0;
   lineColor[1] = 1.0;
   lineColor[2] = 0.0;

   // propagate this color to all subsystems   
   InitGL();
}


HUD::~HUD()
{
   delete theSpeedTape;
   delete thePitchLadder;
   delete theAltTape;
   delete theBankIndicator;
   delete theHeadingTape;
   delete theAircraftReference;
}


void HUD::SetScreenWidth(double width)
{
   screenwidth = width;

   theSpeedTape->SetScreenWidth(screenwidth);
   thePitchLadder->SetScreenWidth(screenwidth);
   theAltTape->SetScreenWidth(screenwidth);
   theBankIndicator->SetScreenWidth(screenwidth);
   theHeadingTape->SetScreenWidth(screenwidth);
   theAircraftReference->SetScreenWidth(screenwidth);
};


void HUD::SetScreenHeight(double height)
{
   screenheight = height;

   theSpeedTape->SetScreenHeight(screenheight);
   thePitchLadder->SetScreenHeight(screenheight);
   theAltTape->SetScreenHeight(screenheight);
   theBankIndicator->SetScreenHeight(screenheight);
   theHeadingTape->SetScreenHeight(screenheight);
   theAircraftReference->SetScreenHeight(screenheight);
};


void HUD::SetVertFOV(double vfov)
{
   vert_fov = vfov;

   theSpeedTape->SetVertFOV(vert_fov);
   thePitchLadder->SetVertFOV(vert_fov);
   theAltTape->SetVertFOV(vert_fov);
   theBankIndicator->SetVertFOV(vert_fov);
   theHeadingTape->SetVertFOV(vert_fov);
   theAircraftReference->SetVertFOV(vert_fov);
};


void HUD::InitGL()
{
   theSpeedTape->SetLineWidth(lineWidth);
   thePitchLadder->SetLineWidth(lineWidth);
   theAltTape->SetLineWidth(lineWidth);
   theBankIndicator->SetLineWidth(lineWidth);
   theHeadingTape->SetLineWidth(lineWidth);
   theAircraftReference->SetLineWidth(lineWidth);

   theSpeedTape->SetLineColor(lineColor[0], lineColor[1], lineColor[2]);
   thePitchLadder->SetLineColor(lineColor[0], lineColor[1], lineColor[2]);
   theAltTape->SetLineColor(lineColor[0], lineColor[1], lineColor[2]);
   theBankIndicator->SetLineColor(lineColor[0], lineColor[1], lineColor[2]);
   theHeadingTape->SetLineColor(lineColor[0], lineColor[1], lineColor[2]);
   theAircraftReference->SetLineColor(lineColor[0], lineColor[1], lineColor[2]);
}


void HUD::SetColor(double r, double g, double b)
{
   lineColor[0] = r;
   lineColor[1] = g;
   lineColor[2] = b;

   theSpeedTape->SetLineColor(r, g, b);
   thePitchLadder->SetLineColor(r, g, b);
   theAltTape->SetLineColor(r, g, b);
   theBankIndicator->SetLineColor(r, g, b);
   theHeadingTape->SetLineColor(r, g, b);
   theAircraftReference->SetLineColor(r, g, b);
};


void HUD::SetLineWidth(double lw)
{
   lineWidth = lw;

   theSpeedTape->SetLineWidth(lineWidth);
   thePitchLadder->SetLineWidth(lineWidth);
   theAltTape->SetLineWidth(lineWidth);
   theBankIndicator->SetLineWidth(lineWidth);
   theHeadingTape->SetLineWidth(lineWidth);
   theAircraftReference->SetLineWidth(lineWidth);
};


void HUD::DrawGL()
{
   glMatrixMode(GL_PROJECTION);

   //save 3D projection matrix
   glPushMatrix();

      // define 2-D Projection matrix
      glLoadIdentity();
      //gluOrtho2D(-screenwidth/2.0, screenwidth/2.0, -screenheight/2.0, screenheight/2.0);
      gluOrtho2D(-512, 512, -384, 384);

      glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);

      // draw 2-D objects
      glMatrixMode(GL_MODELVIEW);
      glLoadIdentity();

      glPushMatrix();
         glScalef(scale*1.0, scale*1.0, scale*1.0);

         if(!show_outsidevisual)
         {
            // black out the outside visual
            glPushMatrix();
               glLineWidth(lineWidth);
               glColor3f(0.0, 0.0, 0.0);
               glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
               glBegin(GL_QUADS);
                  glVertex2f(-3000.0,  3000.0);
                  glVertex2f( 3000.0,  3000.0);
                  glVertex2f( 3000.0, -3000.0);
                  glVertex2f(-3000.0, -3000.0);
               glEnd();
            glPopMatrix();
         }

         if(show_aircraftreference) theAircraftReference->DrawGL();
         if(show_speedtape)         theSpeedTape->DrawGL();
         if(show_pitchladder)       thePitchLadder->DrawGL();
         if(show_alttape)           theAltTape->DrawGL();
         if(show_bankindicator)     theBankIndicator->DrawGL();
         if(show_headingtape)       theHeadingTape->DrawGL();

      glPopMatrix();

      glMatrixMode(GL_PROJECTION);

   glPopMatrix();
}
