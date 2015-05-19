#include "gl2dobjects.hxx"
#include <cmath>

gl2DObject::gl2DObject(): _x(0.0f), _y(0.0f), _s(3.0f), _r(0.0f), _g(1.0f), _b(0.0f), _a(1.0f), _draw(true)
{
}

gl2DObject::~gl2DObject()
{
}

void gl2DObject::visible(bool vis)
{
  _draw = vis;
}

void gl2DObject::setPos(float x, float y)
{
  _x = x;
  _y = y;
}

void gl2DObject::setColor(float r, float g, float b, float a)
{
  _r = r;
  _g = g;
  _b = b;
  _a = a;
}

void gl2DObject::setScale(float s)
{
  _s = s;
}


float gl2DObject::x()
{
  return _x;
}

float gl2DObject::y()
{
  return _y;
}

float gl2DObject::scale()
{
  return _s;
}

float gl2DObject::r()
{
  return _r;
}

float gl2DObject::g()
{
  return _g;
}

float gl2DObject::b()
{
  return _b;
}

float gl2DObject::a()
{
  return _a;
}

void gl2DObject::alpha(float a)
{
  _a = a;
}

void gl2DObject::draw()
{
}

void gl2DObject::display()
{
  if (_draw)
    draw();
}

crossHair::crossHair(): gl2DObject()
{
}

crossHair::~crossHair()
{
}

void crossHair::draw()
{
  glLineWidth(1.5);
  glColor4f(0.0,0.0,0.0,0.75);
  glBegin(GL_LINES);
    glVertex2f(0.0, -6.0);
    glVertex2f(0.0,  6.0);
  glEnd();
  glBegin(GL_LINES);
    glVertex2f(-6.0, 0.0);
    glVertex2f( 6.0, 0.0);
  glEnd();
}

captureInfo::captureInfo(): gl2DObject(), a_max(0.8f), _d(0), _count(0)
{
}

captureInfo::~captureInfo()
{
}

void captureInfo::direction(int d)
{
  _d = d;
  _draw = true;
  _a = 0.1f;
  _count = 0;
}

void captureInfo::draw()
{
  glLineWidth(1.5f);
  float flip = 1.0f;
  switch (_d)
  {
    case -1:
      flip = -1.0f;
    case 1:
      glColor4f(0.0f,0.8f,0.5f,0.5f*a_max*_a);
      glBegin(GL_POLYGON);
        glVertex2f(_x+     0.0f   ,_y-1.0f*_s);
        glVertex2f(_x+     0.0f   ,_y-2.0f*_s);
        glVertex2f(_x+flip*3.0f*_s,_y+0.0f   );
        glVertex2f(_x+     0.0f   ,_y+2.0f*_s);
        glVertex2f(_x+     0.0f   ,_y+1.0f*_s);
        glVertex2f(_x-flip*2.0f*_s,_y+1.0f*_s);
        glVertex2f(_x-flip*2.0f*_s,_y-1.0f*_s);
      glEnd();
      glColor4f(0.0f,1.0f,0.5f,a_max*_a);
      glBegin(GL_LINE_LOOP);
        glVertex2f(_x+     0.0f   ,_y-1.0f*_s);
        glVertex2f(_x+     0.0f   ,_y-2.0f*_s);
        glVertex2f(_x+flip*3.0f*_s,_y+0.0f   );
        glVertex2f(_x+     0.0f   ,_y+2.0f*_s);
        glVertex2f(_x+     0.0f   ,_y+1.0f*_s);
        glVertex2f(_x-flip*2.0f*_s,_y+1.0f*_s);
        glVertex2f(_x-flip*2.0f*_s,_y-1.0f*_s);
      glEnd();
      break;
    case 0:
      glBegin(GL_QUAD_STRIP);
      glColor4f(0.0f,0.8f,0.5f,0.5f*a_max*_a);
      float r1 = 1.5f;
      float r2 = 2.0f;
      for (int i = 0; i <= 20; ++i)
      {
        glVertex2f(_x + r2*_s*cos(i*2*M_PI/20.0), _y + r2*_s* sin(i*2*M_PI/20.0));
        glVertex2f(_x + r1*_s*cos(i*2*M_PI/20.0), _y + r1*_s* sin(i*2*M_PI/20.0));
      }
      glEnd();
      glColor4f(0.0f,1.0f,0.5f,a_max*_a);
      glBegin(GL_LINE_LOOP);
      for (int i = 0; i < 20; ++i)
        glVertex2f(_x + r1*_s*cos(i*2*M_PI/20.0), _y + r1*_s* sin(i*2*M_PI/20.0));
      glEnd();
      glBegin(GL_LINE_LOOP);
      for (int i = 0; i < 20; ++i)
        glVertex2f(_x + r2*_s*cos(i*2*M_PI/20.0), _y + r2*_s* sin(i*2*M_PI/20.0));
      glEnd();
      break;
  }
}

void captureInfo::operator++(int)
{
  if (_count < 10)
    _a += 0.1f;
  else if (_count > 60)
    _a -= 0.05f;
  if (_count > 80)
    _draw = false;
  _count++;
}

#if 0
void ViewWindow::DrawInfoMessage(char* string, double x, double y)
{
      // draw a semi-transparant (alpha blended) box
      // for the message     
      glColor4f(0.5,0.5,0.5,0.6); // 0.6 alpha
      glBegin(GL_QUADS);
         glVertex2f(x,y);
         glVertex2f(x+210.0,y);
         glVertex2f(x+210.0,y+40.0);
         glVertex2f(x,y+40.0);
      glEnd();
      
      // draw the text message inside the box
      glColor3f(1.0,1.0,1.0); // white
      strokeString(lx+10.0, ly+15.0, string, GLUT_STROKE_ROMAN, 0.10);
      
}

void ViewWindow::strokeString(double x, double y, char *msg, void *font, double size)
{
   glPushMatrix();
      GLboolean glBlendEnabled = glIsEnabled(GL_BLEND);
      GLboolean glLineSmoothEnabled = glIsEnabled(GL_LINE_SMOOTH);

      glTranslatef(x, y, 0.0);
      glScalef(size, size, size);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
      glEnable(GL_BLEND);
      glEnable(GL_LINE_SMOOTH);
      glLineWidth(1.5);
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
 
      glLineWidth(1.0);
   glPopMatrix();
}

bool ViewWindow::setStringAndLoc(const char* string, double x, double y){
  strcpy(lstring, string);
  lx = x;
  ly = y;

  return true;
}
#endif
