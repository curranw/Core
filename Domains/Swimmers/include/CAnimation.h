/////////////////////////////////////////////////////////////////////////////
//
// CAnimation.h
//
// Rémi Coulom
//
// December, 2003
//
/////////////////////////////////////////////////////////////////////////////
#ifndef CAnimation_Declared
#define CAnimation_Declared

class CAnimationListener;

class CAnimation // anim
{
 private: ///////////////////////////////////////////////////////////////////
  double xMin, yMin, xMax, yMax;
  double Scale;
  double TimeStep;

  CAnimationListener *panlis;

  int fStop;

 protected: /////////////////////////////////////////////////////////////////
  void ConvertToPixels(double x, double y, int &xi, int &yi);
  void ConvertFromPixels(int xi, int yi, double &x, double &y);
  void ButtonPressed(double x, double y);

 public: ////////////////////////////////////////////////////////////////////
  CAnimation(double xMinInit,
             double yMinInit,
             double xMaxInit,
             double yMaxInit,
             double ScaleInit,
             double TimeStepInit);

  double GetTimeStep() const {return TimeStep;}
  void SetListener(CAnimationListener *panlisNew) {panlis = panlisNew;}

  virtual void SetColor(double r, double g, double b) {};
  virtual void Dot(double x, double y) = 0;
  virtual void Line(double x0, double y0, double x1, double y1) = 0;
  virtual void StartTime() {};
  virtual void NewFrame() = 0;
  virtual void Events() {};

  void ResetStopFlag() {fStop = 0;}
  void SetStopFlag() {fStop = 1;}
  int GetStopFlag() const {return fStop;}

  virtual ~CAnimation() {}
};

#endif
