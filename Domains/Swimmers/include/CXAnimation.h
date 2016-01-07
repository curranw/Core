/////////////////////////////////////////////////////////////////////////////
//
// CXAnimation.h
//
// Rémi Coulom
//
// December, 2003
//
/////////////////////////////////////////////////////////////////////////////
#ifndef CXAnimation_Declared
#define CXAnimation_Declared

#include "CXWindow.h"
#include "CAnimation.h"
#include "clktimer.h"

class CXAnimation: public CXWindow, public CAnimation // xanim
{
 private: ///////////////////////////////////////////////////////////////////
  const int Width;
  const int Height;

  Pixmap pixmap;
  Pixmap pixmapBuffer;

  CClockTimer clktimer;

  void Erase();

 public: ////////////////////////////////////////////////////////////////////
  CXAnimation(double xMinInit,
              double yMinInit,
              double xMaxInit,
              double yMaxInit,
              double ScaleInit,
              double TimeStepInit);

  void SetColor(double r, double g, double b);
  void Dot(double x, double y);
  void Line(double x0, double y0, double x1, double y1);
  void StartTime() {clktimer.GetInterval();}
  void SwitchBuffers();
  void NewFrame();
  void Events();

  ~CXAnimation();
};

#endif
