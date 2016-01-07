/////////////////////////////////////////////////////////////////////////////
//
// CValueFunction.h
//
// Rémi Coulom
//
// December, 2000
//
/////////////////////////////////////////////////////////////////////////////
#ifndef CValueFunction_Declared
#define CValueFunction_Declared

#include "CParametricApproximator.h"

class CValueFunction: public virtual CParametricApproximator // valf
{
 public: //////////////////////////////////////////////////////////////////
  virtual double GetOutput() const = 0;
  virtual double GetDerivative(int k) const = 0;
  virtual void AddGradient(double *pEligibility, double LambdaGamma) const = 0;
  virtual void AddGradientToWeights(double Coeff) {};
};

#endif
