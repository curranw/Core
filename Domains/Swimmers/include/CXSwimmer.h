/////////////////////////////////////////////////////////////////////////////
//
// CXSwimmer.cpp
//
// Remi Coulom
//
// November, 2001
//
/////////////////////////////////////////////////////////////////////////////
#ifndef CXSwimmer_Declared
#define CXSwimmer_Declared

#include "CVector.h"
#include "CXAnimation.h"
#include "CAnimationListener.h"

class CPolicy;
class CSwimmer;

class CXSwimmer: private CAnimationListener // swimmerwin
{
private: ///////////////////////////////////////////////////////////////////


    const int SimulationSteps;
    const double TimeStep;
    const double WinSizeX;
    const double WinSizeY;

    const CSwimmer &swimmer;
    const CPolicy &policy;

    CVector vx;
    double Theta;
    CVector vx2;
    CVector vu;
    CVector vf;
    double x0;
    double y0;
    double t;

    double Offset;

    void ButtonPressed(double x, double y);

public: ////////////////////////////////////////////////////////////////////
    CXSwimmer(const CSwimmer &swimmerInit,
              const CPolicy &policyInit,
              double OffsetInit = 0.0,
              double ScaleInit = 70.0,
              int SimulationStepsInit = 8,
              double TimeStepInit = 0.02,
              double WinSizeX = 12,
              double WinSizeY = 9);
    CXAnimation xanim;
    CVector GetState() {return vx;}
    void Step();
    void Draw();
    void Animate();
    void Reset();
};

#endif
