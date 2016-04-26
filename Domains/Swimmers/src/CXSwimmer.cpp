/////////////////////////////////////////////////////////////////////////////
//
// CXSwimmer.cpp
//
// Remi Coulom
//
// November, 2001
//
/////////////////////////////////////////////////////////////////////////////
#include "CXSwimmer.h"
#include "CPolicy.h"
#include "CSwimmer.h"

#include <cmath>
#include <time.h>

/////////////////////////////////////////////////////////////////////////////
// Constructor
/////////////////////////////////////////////////////////////////////////////
CXSwimmer::CXSwimmer(const CSwimmer &swimmerInit,
                     const CPolicy &policyInit,
                     double OffsetInit,
                     double ScaleInit,
                     int SimulationStepsInit,
                     double TimeStepInit,
                     double WinSizeXInit,
                     double WinSizeYInit) :
    xanim(0, 0, WinSizeXInit, WinSizeYInit, ScaleInit, TimeStepInit),
    SimulationSteps(SimulationStepsInit),
    TimeStep(TimeStepInit),
    WinSizeX(WinSizeXInit),
    WinSizeY(WinSizeYInit),
    swimmer(swimmerInit),
    policy(policyInit),
    vx(2 * swimmer.GetSegments() + 2),
    vx2(2 * swimmer.GetSegments() + 2),
    vu(swimmer.GetSegments() - 1),
    vf(2 * swimmer.GetSegments() + 2),
    Offset(OffsetInit)
{
    Theta = 0.0;
    t = 0.0;
    x0 = WinSizeX / 2;
    y0 = WinSizeY / 2 - double(swimmer.GetSegments()) / 2;
    vx[0] = 0.0;
    vx[1] = 0.0;
    for (int i = swimmer.GetSegments(); --i >= 0;)
    {
        vx[2 * i + 2] = 3.141592653 / 2;
        vx[2 * i + 3] = 0.0;
    }

    for (int i = swimmer.GetSegments() - 1; --i >= 0;)
        vu[i] = 0;

    xanim.SetListener(this);
}

/////////////////////////////////////////////////////////////////////////////
// ButtonPressed
/////////////////////////////////////////////////////////////////////////////
void CXSwimmer::ButtonPressed(double x, double y)
{
    Theta = atan2(y - y0, x - x0);
}

/////////////////////////////////////////////////////////////////////////////
// Step
/////////////////////////////////////////////////////////////////////////////
void CXSwimmer::Step()
{
    const double DeltaT = TimeStep / double(SimulationSteps);
    for (int i = SimulationSteps; --i >= 0;)
    {
        if (x0 > WinSizeX)
            x0 = 0;
        else if (x0 < 0)
            x0 = WinSizeX;
        if (y0 > WinSizeY)
            y0 = 0;
        else if (y0 < 0)
            y0 = WinSizeY;
        swimmer.RotateState(vx, -Theta + Offset, vx2);
        policy.GetControl(vx2, vu);
        swimmer.f(vx, vu, vf);
        swimmer.Step(vx, vf, DeltaT);
        x0 += DeltaT * swimmer.GetVx();
        y0 += DeltaT * swimmer.GetVy();
        t += DeltaT;
    }
}

/////////////////////////////////////////////////////////////////////////////
// Draw
/////////////////////////////////////////////////////////////////////////////
void CXSwimmer::Draw()
{
    //
    // Swimmer
    //
    xanim.SetColor(1, 0, 0);
    double x = x0;
    double y = y0;
    xanim.Dot(x, y);
    xanim.SetColor(0, 0, 0);
    for (int i = 0; i < swimmer.GetSegments(); i++)
    {
        double NewX = x + 1.0 * std::cos(vx[2 * i + 2]);
        double NewY = y + 1.0 * std::sin(vx[2 * i + 2]);
        xanim.Line(x, y, NewX, NewY);
        xanim.Dot(NewX, NewY);

        //
        // Wrap around the window
        //
        {
            int fChange = 0;

            if (NewX > WinSizeX)
            {
                fChange = 1;
                NewX -= WinSizeX;
                x -= WinSizeX;
            }
            else if (NewX < 0)
            {
                fChange = 1;
                NewX += WinSizeX;
                x += WinSizeX;
            }
            if (NewY > WinSizeY)
            {
                fChange = 1;
                NewY -= WinSizeY;
                y -= WinSizeY;
            }
            else if (NewY < 0)
            {
                fChange = 1;
                NewY += WinSizeY;
                y += WinSizeY;
            }

            if (fChange)
            {
                xanim.Dot(x, y);
                xanim.Line(x, y, NewX, NewY);
                xanim.Dot(NewX, NewY);
            }
        }

        x = NewX;
        y = NewY;
    }

    //
    // Arrow indicating the swimming direction
    //
    const double xCenter = 1.0;
    const double yCenter = 8.0;
    const double Radius = 0.7;
    const double ArrowSize = 0.15;
    const double ArrowAngle = 0.5;

    double xArrow = xCenter + Radius * std::cos(Theta);
    double yArrow = yCenter + Radius * std::sin(Theta);
    double x1 = xArrow - ArrowSize * std::cos(Theta + ArrowAngle);
    double y1 = yArrow - ArrowSize * std::sin(Theta + ArrowAngle);
    double x2 = xArrow - ArrowSize * std::cos(Theta - ArrowAngle);
    double y2 = yArrow - ArrowSize * std::sin(Theta - ArrowAngle);

    xanim.SetColor(0, 0, 1);
    xanim.Dot(xCenter, yCenter);
    xanim.Line(xCenter, yCenter, xArrow, yArrow);
    xanim.Line(xArrow, yArrow, x1, y1);
    xanim.Line(xArrow, yArrow, x2, y2);
}

/////////////////////////////////////////////////////////////////////////////
// Animate
/////////////////////////////////////////////////////////////////////////////
void CXSwimmer::Animate()
{
    xanim.Events();
    xanim.StartTime();
    xanim.ResetStopFlag();
    while(!xanim.GetStopFlag())
    {
        Draw();
        Step();
#if 0
        if (t > 3.3)
            Theta = atan2(0, -1);
        if (t > 6.6)
            break;
#endif
        xanim.Events();
        xanim.NewFrame();
    }
}

void CXSwimmer::Reset()
{
    Theta = 0.0;
    t = 0.0;
    x0 = WinSizeX / 2;
    y0 = WinSizeY / 2 - double(swimmer.GetSegments()) / 2;
    vx[0] = 0.0;
    vx[1] = 0.0;
    for (int i = swimmer.GetSegments(); --i >= 0;)
    {
        double r = double(rand())/RAND_MAX;
        vx[2 * i + 2] = -3.14159 + r * (3.14159 + 3.14159);
        //vx[2 * i + 2] = 3.141592653 / 2;
        //vx[2 * i + 2] = -3.141592653;
        vx[2 * i + 3] = 0.0;
    }

    for (int i = swimmer.GetSegments() - 1; --i >= 0;)
        vu[i] = 0;

    xanim.SetListener(this);
}
