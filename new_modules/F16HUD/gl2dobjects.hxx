#ifndef GL2DOBJECTS_HXX
#define GL2DOBJECTS_HXX

#include <GL/glut.h>

/**
	@author  <jellerbroek@lrcs-041f.lr.tudelft.nl>
*/
class gl2DObject{
public: // Constructor, destructor
  gl2DObject();

  virtual ~gl2DObject();

protected: // Overloadable functions
  virtual void draw();

public: // Inherited functions
  void  visible(bool vis);
  void  setPos(float x, float y);
  void  setColor(float r, float g, float b, float a);
  void  setScale(float s);
  void  alpha(float a);
  float x();
  float y();
  float scale();
  float r();
  float g();
  float b();
  float a();

public:
  void display();
  
protected: // member variables
  float _x, _y, _s, _r, _g, _b, _a;
  bool  _draw;
};

class crossHair: public gl2DObject
{
  public:
    crossHair();
    ~crossHair();

  public:
    void draw();
};

class captureInfo: public gl2DObject
{
  public:
    captureInfo();
    ~captureInfo();

  public:
    void draw();
    void operator++(int);
    void direction(int d);
    
  protected:
    float a_max;
    int _d, _count;
};

#endif

