/////////////////////////////////////////////////////////////////////////////
//
// CAnimationListener.h
//
// R�mi Coulom
//
// December, 2003
//
/////////////////////////////////////////////////////////////////////////////
#ifndef CAnimationListener_Declared
#define CAnimationListener_Declared

class CAnimationListener // anlis
{
 public: ////////////////////////////////////////////////////////////////////
  virtual void ButtonPressed(double x, double y) = 0;
  virtual ~CAnimationListener() {}
};

#endif
