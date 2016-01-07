////////////////////////////////////////////////////////////////////////////
//
// COptimalControlProblem.h
//
// Remi Coulom
//
// September, 2000
//
////////////////////////////////////////////////////////////////////////////
#ifndef COptimalControlProblem_Declared
#define COptimalControlProblem_Declared

#include "random.h"

class CValueFunction;

class COptimalControlProblem // ocp
{
 private: /////////////////////////////////////////////////////////////////
  const int StateDimension;
  const int ControlDimension;

  double Shortness;

  double *pxMin;
  double *pxMax;
  int *pBoundaryType;
  double *puMin;
  double *puMax;

 protected: ///////////////////////////////////////////////////////////////
  void SetShortness(double s) {Shortness = s;}

  void SetXMin(int i, double x) {pxMin[i] = x;}
  void SetXMax(int i, double x) {pxMax[i] = x;}

  void SetUMin(int i, double u) {puMin[i] = u;}
  void SetUMax(int i, double u) {puMax[i] = u;}

  void SetBoundaryType(int i, int bt) {pBoundaryType[i] = bt;}

 public: //////////////////////////////////////////////////////////////////
  COptimalControlProblem(int StateInit,
                         int ControlInit,
                         double ShortnessInit);

  int GetStateDimension() const {return StateDimension;}
  int GetControlDimension() const {return ControlDimension;}
  double GetShortness() const {return Shortness;}

  double GetXMin(int i) const {return pxMin[i];}
  double GetXMax(int i) const {return pxMax[i];}
  double GetUMin(int i) const {return puMin[i];}
  double GetUMax(int i) const {return puMax[i];}

  enum {Normal, Cylinder, Unbounded};
  int GetBoundaryType(int i) const {return pBoundaryType[i];}

  virtual void Step(double *px, const double *pf, double dt) const;
  virtual void AdamsStep(double *pxPrev,
                         double *px,
                         const double *pf,
                         double dt) const;
  virtual void BigStep(double *px,
                       double *pu,
                       double *pf,
                       double DeltaT,
                       CValueFunction &valf,
                       double &V0,
                       double &V1,
                       double &Reward) const;

  //
  // State space
  //
  virtual int InBounds(const double *px) const;
  virtual void RandomX(double *px, CRandom<unsigned> &rnd) const;
  virtual void RandomBoundaryX(double *px, CRandom<unsigned> &rnd) const;
  virtual void RandomXChange(double *px, CRandom<unsigned> &rnd) const {RandomX(px, rnd);}

  //
  // Control space
  //
  virtual int InU(const double *pu) const;
  virtual void RandomU(double *pu, CRandom<unsigned> &rnd) const;
  virtual int DiscreteControls() const;
  virtual void GetControls(const double *px, double *ppu) const;
  virtual void GetGreedyControl(const double *px,
                                CValueFunction &valf, // after SetInput(px)
                                double *pu) const;
  virtual void GetGreedyControl(const double *px,
                                const double *pGradV,
                                double *pu) const;

  //
  // System dynamics
  //
  virtual void f(const double *px, const double *pu, double *pf) const = 0;

  //
  // dH/du for actor-critic algorithms
  //
  virtual void GetActorGradient(const double *px,
                                CValueFunction &valf, // after SetInput(px)
                                double *pGradH) const;

  //
  // Gradients of r and f for purely critic algorithms
  //
  virtual void GetRGradient(const double *px,
                            const double *pu,
                            const double *pGradX, // dx1/dw1 dx1/dw2 ...
                            const double *pGradU, // du1/dw1 du1/dw2 ...
                            int n,
                            double *pGradR) const;
  virtual void GetFGradient(const double *px,
                            const double *pu,
                            const double *pGradX,
                            const double *pGradU,
                            int n,
                            double *pGradF) const;

  //
  // Reward function and boundary conditions
  //
  virtual double r(const double *px, const double *pu) const = 0;
  virtual double BoundaryValue(const double *px) const {return 0;}

  //
  // Goal
  //
  virtual int Goal(const double *px) const {return 0;}

  //
  // Destructor
  //
  virtual ~COptimalControlProblem();
};

#endif
