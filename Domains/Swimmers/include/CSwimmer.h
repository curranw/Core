////////////////////////////////////////////////////////////////////////////
//
// CSwimmer.h
//
// Remi Coulom
//
// October, 2001
//
////////////////////////////////////////////////////////////////////////////
#ifndef CSwimmer_Declared
#define CSwimmer_Declared

#include "COptimalControlProblem.h"
#include "CLUDecomposition.h"

class CSwimmer : public COptimalControlProblem // swimmer
{
private: /////////////////////////////////////////////////////////////////
    const int Segments; // n
    const int fRotate;
    CLUDecomposition lud;

    double *pdxCache;    // (2 * n + 2) last px given to ComputeMatrix
    double *pdMatrix;    // (n+2)*(n+3) system: x y Theta_1 ... Theta_n
    double *pdLU;        // (n+2)*(n+2) LU decomposition matrix
    int *pIndex;         // (n+2) LU decomposition reordering
    double *pdb;         // (n+2) constant right-hand vector
    double *pdRH;        // (n+2) right-hand when solving linear system
    double *pdSolution;  // (n+2) linear system solution
    double *pdADotx;     // (n+1) Array of x coordinate of joint velocity
    double *pdADoty;     // (n+1) Array of y corrdinate of joint velocity
    double *pdGDotDotx;  // (n+3) G acceleration (x)
    double *pdGDotDoty;  // (n+3) G acceleration (y)
    double *pdADotDotx;  // (n+1)*(n+3) equations of joint acceleration (x)
    double *pdADotDoty;  // (n+1)*(n+3) equations of joint acceleration (y)
    double *pdfx;        // (n+1)*(n+3) equations for internal forces (x)
    double *pdfy;        // (n+1)*(n+3) equations for internal forces (y)
    double *pGradV;

    void ComputeMatrix(const double *px) const;

public: //////////////////////////////////////////////////////////////////
    CSwimmer(int SegmentsInit = 2, int fRotateInit = 1);

    int GetSegments() const {return Segments;}

    void f(const double *px, const double *pu, double *pf) const;
    double r(const double *px, const double *pu) const;
    double GetVx() const {return pdADotx[0];}
    double GetVx(int i) {return pdADotx[i];}
    double GetVy() const {return pdADoty[0];}
    double GetVy(int i) {return pdADoty[i];}

    void GetGreedyControl(const double *px,
                          CValueFunction &valf,
                          double *pu) const;
    void GetGreedyControl(const double *px,
                          const double *pGradV,
                          double *pu) const;
    void GetActorGradient(const double *px,
                          CValueFunction &valf,
                          double *pGradH) const;

    void RotateState(const double *px1, double Theta, double *px2) const;
    void RandomXChange(double *px, CRandom<unsigned> &rnd) const;
    void GetG(const double *px, double &Gx, double &Gy) const;

    void reset();

    ~CSwimmer();
};

#endif
