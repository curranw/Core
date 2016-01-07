////////////////////////////////////////////////////////////////////////////
//
// COptimalControlProblem.cpp
//
// Remi Coulom
//
// September, 2000
//
////////////////////////////////////////////////////////////////////////////
#include "COptimalControlProblem.h"
#include "random.h"
#include "CValueFunction.h"

////////////////////////////////////////////////////////////////////////////
// Constructor
////////////////////////////////////////////////////////////////////////////
COptimalControlProblem::COptimalControlProblem(int StateInit,
                                               int ControlInit,
                                               double ShortnessInit) :
    StateDimension(StateInit),
    ControlDimension(ControlInit),
    Shortness(ShortnessInit)
{
    pxMin = new double[StateDimension];
    pxMax = new double[StateDimension];
    pBoundaryType = new int[StateDimension];

    for (int i = StateDimension; --i >= 0;)
    {
        pxMin[i] = pxMax[i] = 0.0;
        pBoundaryType[i] = Normal;
    }

    puMin = new double[ControlDimension];
    puMax = new double[ControlDimension];

    for (int i = ControlDimension; --i >= 0;)
        puMin[i] = puMax[i] = 0.0;
}

////////////////////////////////////////////////////////////////////////////
// Destructor
////////////////////////////////////////////////////////////////////////////
COptimalControlProblem::~COptimalControlProblem()
{
    delete[] pxMin;
    delete[] pxMax;
    delete[] pBoundaryType;

    delete[] puMin;
    delete[] puMax;
}

////////////////////////////////////////////////////////////////////////////
// Evaluate step
////////////////////////////////////////////////////////////////////////////
void COptimalControlProblem::Step(double *px,
                                  const double *pf,
                                  double dt) const
{
    for (int i = StateDimension; --i >= 0;)
    {
        px[i] += dt * pf[i];
        if (pBoundaryType[i] == Cylinder)
        {
            if (px[i] > pxMax[i])
                px[i] += pxMin[i] - pxMax[i];
            else if (px[i] < pxMin[i])
                px[i] += pxMax[i] - pxMin[i];
        }
    }
}

////////////////////////////////////////////////////////////////////////////
// Evaluate step with second order Adams Predictor Corrector
////////////////////////////////////////////////////////////////////////////
void COptimalControlProblem::AdamsStep(double *pxPrev,
                                       double *px,
                                       const double *pf,
                                       double dt) const
{
    for (int i = StateDimension; --i >= 0;)
    {
        double x = 0.5 * (pxPrev[i] + px[i]) + 1.5 * dt * pf[i];
        pxPrev[i] = px[i];
        px[i] = x;

        if (pBoundaryType[i] == Cylinder)
        {
            if (px[i] > pxMax[i])
            {
                px[i] += pxMin[i] - pxMax[i];
                pxPrev[i] += pxMin[i] - pxMax[i];
            }
            else if (px[i] < pxMin[i])
            {
                px[i] += pxMax[i] - pxMin[i];
                pxPrev[i] += pxMax[i] - pxMin[i];
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////
// Big step
// Assumes that valf.SetInput(px) has been called
////////////////////////////////////////////////////////////////////////////
void COptimalControlProblem::BigStep(double *px,
                                     double *pu,
                                     double *pf,
                                     double DeltaT,
                                     CValueFunction &valf,
                                     double &V0,
                                     double &V1,
                                     double &Reward) const
{
    V0 = valf.GetOutput();

    GetGreedyControl(px, valf, pu);
    Reward = r(px, pu);
    f(px, pu, pf);
    Step(px, pf, DeltaT);

    if (!InBounds(px))
        V1 = BoundaryValue(px);
    else
    {
        valf.SetInput(px);
        V1 = valf.GetOutput();
    }
}

////////////////////////////////////////////////////////////////////////////
// Default state space
////////////////////////////////////////////////////////////////////////////
int COptimalControlProblem::InBounds(const double *px) const
{
    for (int i = StateDimension; --i >= 0;)
        if (pBoundaryType[i] == Normal &&
                (px[i] >= pxMax[i] || px[i] <= pxMin[i]))
            return 0;
    return 1;
}

////////////////////////////////////////////////////////////////////////////
// Default random starting position
////////////////////////////////////////////////////////////////////////////
void COptimalControlProblem::RandomX(double *px, CRandom<unsigned> &rnd) const
{
    for (int i = StateDimension; --i >= 0;)
        px[i] = pxMin[i] + rnd.NextDouble() * (pxMax[i] - pxMin[i]);
}

////////////////////////////////////////////////////////////////////////////
// Random boundary point
////////////////////////////////////////////////////////////////////////////
void COptimalControlProblem::RandomBoundaryX(double *px, CRandom<unsigned> &rnd) const
{
    int Boundaries = 0;

    for (int i = StateDimension; --i >= 0;)
    {
        px[i] = pxMin[i] + rnd.NextDouble() * (pxMax[i] - pxMin[i]);
        if (pBoundaryType[i] == Normal)
            Boundaries += 2;
    }

    if (Boundaries)
    {
        int r = rnd.NewValue() % Boundaries;
        int s = r / 2;

        for (int i = StateDimension; --i >= 0;)
        {
            if (i == s)
                if (r & 1)
                    px[i] = pxMin[i];
                else
                    px[i] = pxMax[i];
            else
                px[i] = pxMin[i] + rnd.NextDouble() * (pxMax[i] - pxMin[i]);
        }
    }
}

////////////////////////////////////////////////////////////////////////////
// Default control space
////////////////////////////////////////////////////////////////////////////
int COptimalControlProblem::InU(const double *pu) const
{
    for (int i = ControlDimension; --i >= 0;)
        if (pu[i] > puMax[i] || pu[i] < puMin[i])
            return 0;
    return 1;
}

////////////////////////////////////////////////////////////////////////////
// Default random control
////////////////////////////////////////////////////////////////////////////
void COptimalControlProblem::RandomU(double *pu, CRandom<unsigned> &rnd) const
{
    for (int i = ControlDimension; --i >= 0;)
        pu[i] = puMin[i] + rnd.NextDouble() * (puMax[i] - puMin[i]);
}

////////////////////////////////////////////////////////////////////////////
// Default discrete controls
////////////////////////////////////////////////////////////////////////////
int COptimalControlProblem::DiscreteControls() const
{
    return 2 * ControlDimension;
}

////////////////////////////////////////////////////////////////////////////
// Default discrete controls
////////////////////////////////////////////////////////////////////////////
void COptimalControlProblem::GetControls(const double *px,
                                         double *ppu) const
{
    for (int i = 2 * ControlDimension * ControlDimension; --i >= 0;)
        ppu[i] = 0;

    for (int i = ControlDimension; --i >= 0;)
    {
        int ind = i * (1 + 2 * ControlDimension);
        ppu[ind] = GetUMax(i);
        ppu[ind + ControlDimension] = GetUMin(i);
    }
}

////////////////////////////////////////////////////////////////////////////
// Greedy control from a value function
////////////////////////////////////////////////////////////////////////////
void COptimalControlProblem::GetGreedyControl(const double *px,
                                              CValueFunction &valf,
                                              double *pu) const
{
    for (int i = ControlDimension; --i >= 0;)
        pu[i] = 0;
}

////////////////////////////////////////////////////////////////////////////
// Greedy control with respect to a value gradient
////////////////////////////////////////////////////////////////////////////
void COptimalControlProblem::GetGreedyControl(const double *px,
                                              const double *pGradV,
                                              double *pu) const
{
    for (int i = ControlDimension; --i >= 0;)
        pu[i] = 0;
}

////////////////////////////////////////////////////////////////////////////
// Gradient of H with respect to control for actor-critic algorithms
////////////////////////////////////////////////////////////////////////////
void COptimalControlProblem::GetActorGradient(const double *px,
                                              CValueFunction &valf,
                                              double *pGradH) const
{
    for (int i = ControlDimension; --i >= 0;)
        pGradH[i] = 0;
}

////////////////////////////////////////////////////////////////////////////
// Gradient of r with respect to n actor parameters
////////////////////////////////////////////////////////////////////////////
void COptimalControlProblem::GetRGradient(const double *px,
                                          const double *pu,
                                          const double *pGradX,
                                          const double *pGradU,
                                          int n,
                                          double *pGradR) const
{
    for (int i = n; --i >= 0;)
        pGradR[i] = 0;
}

////////////////////////////////////////////////////////////////////////////
// Gradient of f with respect to n actor parameters
////////////////////////////////////////////////////////////////////////////
void COptimalControlProblem::GetFGradient(const double *px,
                                          const double *pu,
                                          const double *pGradX,
                                          const double *pGradU,
                                          int n,
                                          double *pGradF) const
{
    for (int i = n * StateDimension; --i >= 0;)
        pGradF[i] = 0;
}
