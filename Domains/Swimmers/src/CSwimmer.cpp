////////////////////////////////////////////////////////////////////////////
//
// CSwimmer.cpp
//
// Remi Coulom
//
// October, 2001
//
////////////////////////////////////////////////////////////////////////////
#include "CSwimmer.h"
#include "CValueFunction.h"
#include "pi.h"
#include "CVector.h"

#include <cmath>

#if 1
const double vMax = 1.0;
const double ThetaDotMax = 6.0;
const double uMax = 5.0;
const double SegLength = 1.0;
const double SegMass = 1.0;
const double k = 10.0;
#else
const double vMax = 2.5;
const double ThetaDotMax = 6.0;
const double uMax = 2.0;
const double SegLength = 1.0;
const double SegMass = 1.0;
const double k = 1.0;
#endif

////////////////////////////////////////////////////////////////////////////
// Constructor
////////////////////////////////////////////////////////////////////////////
CSwimmer::CSwimmer(int SegmentsInit, int fRotateInit) :
    COptimalControlProblem(2 * SegmentsInit + 2, SegmentsInit - 1, 0.2),
    Segments(SegmentsInit),
    fRotate(fRotateInit),
    lud(Segments + 2)
{
    // x dot
    SetXMin(0, -vMax);
    SetXMax(0, vMax);
    SetBoundaryType(0, Unbounded);

    // y dot
    SetXMin(1, -vMax);
    SetXMax(1, vMax);
    SetBoundaryType(1, Unbounded);

    // Theta_i and Theta_i dot
    for (int i = 1; i <= SegmentsInit; i++)
    {
        SetXMin(2 * i, -MY_PI);
        SetXMax(2 * i, +MY_PI);
        SetBoundaryType(2 * i, Cylinder);
        SetXMin(2 * i + 1, -ThetaDotMax);
        SetXMax(2 * i + 1, +ThetaDotMax);
        SetBoundaryType(2 * i + 1, Unbounded);
    }

    // control
    for (int i = SegmentsInit - 1; --i >= 0;)
    {
        SetUMin(i, -uMax);
        SetUMax(i, +uMax);
    }

    //
    // Allocate tables
    //
    pdxCache = new double[2 * Segments + 2];
    pdMatrix = new double[(Segments + 2) * (Segments + 3)];
    pdLU = new double[(Segments + 2) * (Segments + 2)];
    pIndex = new int[Segments + 2];
    pdb = new double[Segments + 2];
    pdRH = new double[Segments + 2];
    pdSolution = new double[Segments + 2];
    pdADotx = new double[Segments + 1];
    pdADoty = new double[Segments + 1];
    pdGDotDotx = new double[Segments + 3];
    pdGDotDoty = new double[Segments + 3];
    pdADotDotx = new double[(Segments + 1) * (Segments + 3)];
    pdADotDoty = new double[(Segments + 1) * (Segments + 3)];
    pdfx = new double[(Segments + 1) * (Segments + 3)];
    pdfy = new double[(Segments + 1) * (Segments + 3)];
    pGradV = new double[Segments * 2 + 2];

    //
    // Fill tables with constant values
    //
    for (int i = Segments + 3; --i >= 0;)
    {
        pdfx[i] = 0.0;
        pdfy[i] = 0.0;
    }

    //
    // Reset cache
    //
    for (int i = 2 * Segments + 2; --i >= 0;)
        pdxCache[i] = i;

    {
        CVector vx(2 * Segments + 2);
        vx.Zero();
        ComputeMatrix(vx);
    }
}

////////////////////////////////////////////////////////////////////////////
// Compute matrix
////////////////////////////////////////////////////////////////////////////
void CSwimmer::ComputeMatrix(const double *px) const
{
    //
    // Cache
    //
    {
        int fDone = 1;
        for (int i = 2 * Segments + 2; --i >= 0;)
            if (px[i] != pdxCache[i])
            {
                pdxCache[i] = px[i];
                fDone = 0;
            }
        if (fDone)
            return;
    }

    //
    // ADot and ADotDot
    //
    const double Coeff = 0.5 * SegMass / (Segments * SegMass);

    pdADotx[0] = 0.0;
    pdADoty[0] = 0.0;
    double GDotx = 0.0;
    double GDoty = 0.0;

    for (int j = Segments + 3; --j >= 0;)
    {
        pdADotDotx[j] = 0.0;
        pdADotDoty[j] = 0.0;
        pdGDotDotx[j] = 0.0;
        pdGDotDoty[j] = 0.0;
    }

    for (int i = 1; i <= Segments; i++)
    {
        double c = std::cos(px[2 * i]);
        double s = std::sin(px[2 * i]);
        double ThetaDot = px[2 * i + 1];
        const int Line = i * (Segments + 3);
        const int PrevLine = (i - 1) * (Segments + 3);

        pdADotx[i] = pdADotx[i - 1] - SegLength * ThetaDot * s;
        pdADoty[i] = pdADoty[i - 1] + SegLength * ThetaDot * c;

        for (int j = Segments + 3; --j >= 0;)
        {
            pdADotDotx[Line + j] = pdADotDotx[PrevLine + j];
            pdADotDoty[Line + j] = pdADotDoty[PrevLine + j];
        }
        pdADotDotx[Line + i + 1] += -SegLength * s;
        pdADotDoty[Line + i + 1] += SegLength * c;
        pdADotDotx[Line + Segments + 2] += SegLength * ThetaDot * ThetaDot * c;
        pdADotDoty[Line + Segments + 2] += SegLength * ThetaDot * ThetaDot * s;

        GDotx += Coeff * (pdADotx[i - 1] + pdADotx[i]);
        GDoty += Coeff * (pdADoty[i - 1] + pdADoty[i]);
        for (int j = Segments + 3; --j >= 0;)
        {
            pdGDotDotx[j] += Coeff * (pdADotDotx[PrevLine + j] + pdADotDotx[Line + j]);
            pdGDotDoty[j] += Coeff * (pdADotDoty[PrevLine + j] + pdADotDoty[Line + j]);
        }
    }

    for (int i = 0; i <= Segments; i++)
    {
        pdADotx[i] += px[0] - GDotx;
        pdADoty[i] += px[1] - GDoty;
        pdADotDotx[i * (Segments + 3) + 0] += 1.0;
        pdADotDoty[i * (Segments + 3) + 1] += 1.0;
        for (int j = Segments + 3; --j >= 0;)
        {
            pdADotDotx[i * (Segments + 3) + j] -= pdGDotDotx[j];
            pdADotDoty[i * (Segments + 3) + j] -= pdGDotDoty[j];
        }
    }

    //
    // Fill f arrays
    //
    for (int i = 1; i <= Segments; i++)
    {
        double c = std::cos(px[2 * i]);
        double s = std::sin(px[2 * i]);
        const int Line = i * (Segments + 3);
        const int PrevLine = (i - 1) * (Segments + 3);

        for (int j = Segments + 3; --j >= 0;)
        {
            pdfx[Line + j] = pdfx[PrevLine + j] +
                    SegMass * 0.5 * (pdADotDotx[Line + j] + pdADotDotx[PrevLine + j]);
            pdfy[Line + j] = pdfy[PrevLine + j] +
                    SegMass * 0.5 * (pdADotDoty[Line + j] + pdADotDoty[PrevLine + j]);
        }
        double F = -k * SegLength * 0.5 * (-(pdADotx[i] + pdADotx[i - 1]) * s +
                (pdADoty[i] + pdADoty[i - 1]) * c);
        pdfx[Line + Segments + 2] += -F * s;
        pdfy[Line + Segments + 2] += F * c;
    }

    //
    // Compute the linear system to be solved
    //
    for (int j = Segments + 3; --j >= 0;)
    {
        pdMatrix[j] = pdfx[Segments * (Segments + 3) + j];
        pdMatrix[j + Segments + 3] = pdfy[Segments * (Segments + 3) + j];
    }
    for (int i = 1; i <= Segments; i++)
    {
        double c = std::cos(px[2 * i]);
        double s = std::sin(px[2 * i]);
        double ThetaDot = px[2 * i + 1];
        int MatrixLine = (i + 1) * (Segments + 3);
        int Line = i * (Segments + 3);
        int PrevLine = (i - 1) * (Segments + 3);

        for (int j = Segments + 3; --j >= 0;)
            pdMatrix[MatrixLine + j] = SegLength * 0.5 *
                    (c * (pdfy[Line + j] + pdfy[PrevLine + j]) -
                    s * (pdfx[Line + j] + pdfx[PrevLine + j]));
        pdMatrix[MatrixLine + Segments + 2] +=
                k * ThetaDot * SegLength * SegLength * SegLength / 12;
        pdMatrix[MatrixLine + i + 1] -= SegMass * SegLength / 12;
    }

    //
    // LU-Decompose its matrix
    //
    for (int i = Segments + 2; --i >= 0;)
    {
        for (int j = Segments + 2; --j >= 0;)
            pdLU[i * (Segments + 2) + j] = pdMatrix[i * (Segments + 3) + j];
        pdb[i] = pdMatrix[i * (Segments + 3) + Segments + 2];
    }
    lud.Decompose(pdLU, pIndex);
}

////////////////////////////////////////////////////////////////////////////
// f (system dynamics)
////////////////////////////////////////////////////////////////////////////
void CSwimmer::f(const double *px, const double *pu, double *pf) const
{
    ComputeMatrix(px);
    for (int i = Segments + 2; --i >= 0;)
        pdRH[i] = pdb[i];
    for (int i = 0; i < Segments - 1; i++)
    {
        pdRH[i + 3] -= pu[i];
        pdRH[i + 2] += pu[i];
    }
    lud.Solve(pdLU, pIndex, pdRH, pdSolution);
    pf[0] = pdSolution[0];
    pf[1] = pdSolution[1];
    for (int i = Segments; --i >= 0;)
    {
        pf[2 * i + 2] = px[2 * i + 3];
        pf[2 * i + 3] = pdSolution[i + 2];
    }
}

////////////////////////////////////////////////////////////////////////////
// r
////////////////////////////////////////////////////////////////////////////
double CSwimmer::r(const double *px, const double *pu) const
{
    return px[0];
}

////////////////////////////////////////////////////////////////////////////
// Destructor
////////////////////////////////////////////////////////////////////////////
CSwimmer::~CSwimmer()
{
    delete[] pdxCache;
    delete[] pdMatrix;
    delete[] pdLU;
    delete[] pIndex;
    delete[] pdb;
    delete[] pdRH;
    delete[] pdSolution;
    delete[] pdADotx;
    delete[] pdADoty;
    delete[] pdGDotDotx;
    delete[] pdGDotDoty;
    delete[] pdADotDotx;
    delete[] pdADotDoty;
    delete[] pdfx;
    delete[] pdfy;
    delete[] pGradV;
}

////////////////////////////////////////////////////////////////////////////
// valf-based controller (suppose that input set and forward prop done)
////////////////////////////////////////////////////////////////////////////
void CSwimmer::GetGreedyControl(const double *px,
                                CValueFunction &valf,
                                double *pu) const
{
    pGradV[0] = valf.GetDerivative(0);
    pGradV[1] = valf.GetDerivative(1);
    for (int i = Segments; --i >= 0;)
        pGradV[2 * i + 3] = valf.GetDerivative(2 * i + 3);
    GetGreedyControl(px, pGradV, pu);
}

////////////////////////////////////////////////////////////////////////////
// Value-function-based controller
////////////////////////////////////////////////////////////////////////////
void CSwimmer::GetGreedyControl(const double *px,
                                const double *pGradV,
                                double *pu) const
{
    ComputeMatrix(px);
    pdRH[0] = pGradV[0];
    pdRH[1] = pGradV[1];
    for (int i = Segments; --i >= 0;)
        pdRH[i + 2] = pGradV[2 * i + 3];
    lud.SolveTranspose(pdLU, pIndex, pdRH, pdSolution);
    for (int i = 0; i < Segments - 1; i++)
    {
        double a = pdSolution[i + 2] - pdSolution[i + 3];
        if (a > 0)
            pu[i] = GetUMax(i);
        else
            pu[i] = GetUMin(i);
    }
}

////////////////////////////////////////////////////////////////////////////
// Actor gradient
////////////////////////////////////////////////////////////////////////////
void CSwimmer::GetActorGradient(const double *px,
                                CValueFunction &valf,
                                double *pGradH) const
{
    ComputeMatrix(px);
    pdRH[0] = valf.GetDerivative(0);
    pdRH[1] = valf.GetDerivative(1);
    for (int i = Segments; --i >= 0;)
        pdRH[i + 2] = valf.GetDerivative(2 * i + 3);
    lud.SolveTranspose(pdLU, pIndex, pdRH, pdSolution);
    for (int i = 0; i < Segments - 1; i++)
        pGradH[i] = pdSolution[i + 2] - pdSolution[i + 3];
}

////////////////////////////////////////////////////////////////////////////
// Rotate state
////////////////////////////////////////////////////////////////////////////
void CSwimmer::RotateState(const double *px1, double Theta, double *px2) const
{
    double c = std::cos(Theta);
    double s = std::sin(Theta);

    double temp0 = c * px1[0] - s * px1[1];
    double temp1 = s * px1[0] + c * px1[1];

    px2[0] = temp0;
    px2[1] = temp1;

    for (int i = 0; i < Segments; i++)
    {
        px2[2 * i + 2] = px1[2 * i + 2] + Theta;
        px2[2 * i + 3] = px1[2 * i + 3];
    }
}

////////////////////////////////////////////////////////////////////////////
// Random state change
////////////////////////////////////////////////////////////////////////////
#include "random.h"
void CSwimmer::RandomXChange(double *px, CRandom<unsigned> &rnd) const
{
    if (fRotate)
    {
        double Theta = rnd.NextDouble() * 2 * MY_PI;
        RotateState(px, Theta, px);
    }
    else
        RandomX(px, rnd);
}

////////////////////////////////////////////////////////////////////////////
// Compute the relative position of the center of mass, with respect
// to the first point
////////////////////////////////////////////////////////////////////////////
void CSwimmer::GetG(const double *px, double &Gx, double &Gy) const
{
    Gx = 0.0;
    Gy = 0.0;

    double x = 0.0;
    double y = 0.0;

    for (int i = 0; i < Segments; i++)
    {
        Gx += x;
        Gy += y;
        x += std::cos(px[2 * i + 2]);
        y += std::sin(px[2 * i + 2]);
        Gx += x;
        Gy += y;
    }

    Gx /= 2 * Segments;
    Gy /= 2 * Segments;
}

void CSwimmer::reset()
{
    delete[] pdxCache;
    delete[] pdMatrix;
    delete[] pdLU;
    delete[] pIndex;
    delete[] pdb;
    delete[] pdRH;
    delete[] pdSolution;
    delete[] pdADotx;
    delete[] pdADoty;
    delete[] pdGDotDotx;
    delete[] pdGDotDoty;
    delete[] pdADotDotx;
    delete[] pdADotDoty;
    delete[] pdfx;
    delete[] pdfy;
    delete[] pGradV;



    // x dot
    SetXMin(0, -vMax);
    SetXMax(0, vMax);
    SetBoundaryType(0, Unbounded);

    // y dot
    SetXMin(1, -vMax);
    SetXMax(1, vMax);
    SetBoundaryType(1, Unbounded);

    // Theta_i and Theta_i dot
    for (int i = 1; i <= Segments; i++)
    {
        SetXMin(2 * i, -MY_PI);
        SetXMax(2 * i, +MY_PI);
        SetBoundaryType(2 * i, Cylinder);
        SetXMin(2 * i + 1, -ThetaDotMax);
        SetXMax(2 * i + 1, +ThetaDotMax);
        SetBoundaryType(2 * i + 1, Unbounded);
    }

    // control
    for (int i = Segments - 1; --i >= 0;)
    {
        SetUMin(i, -uMax);
        SetUMax(i, +uMax);
    }

    //
    // Allocate tables
    //
    pdxCache = new double[2 * Segments + 2];
    pdMatrix = new double[(Segments + 2) * (Segments + 3)];
    pdLU = new double[(Segments + 2) * (Segments + 2)];
    pIndex = new int[Segments + 2];
    pdb = new double[Segments + 2];
    pdRH = new double[Segments + 2];
    pdSolution = new double[Segments + 2];
    pdADotx = new double[Segments + 1];
    pdADoty = new double[Segments + 1];
    pdGDotDotx = new double[Segments + 3];
    pdGDotDoty = new double[Segments + 3];
    pdADotDotx = new double[(Segments + 1) * (Segments + 3)];
    pdADotDoty = new double[(Segments + 1) * (Segments + 3)];
    pdfx = new double[(Segments + 1) * (Segments + 3)];
    pdfy = new double[(Segments + 1) * (Segments + 3)];
    pGradV = new double[Segments * 2 + 2];

    //
    // Fill tables with constant values
    //
    for (int i = Segments + 3; --i >= 0;)
    {
        pdfx[i] = 0.0;
        pdfy[i] = 0.0;
    }

    //
    // Reset cache
    //
    for (int i = 2 * Segments + 2; --i >= 0;)
        pdxCache[i] = i;

    {
        CVector vx(2 * Segments + 2);
        vx.Zero();
        ComputeMatrix(vx);
    }
}
