/////////////////////////////////////////////////////////////////////////////
//
// CParametricApproximator.h
//
// Rémi Coulom
//
// September, 2003
//
/////////////////////////////////////////////////////////////////////////////
#ifndef CParametricApproximator_Declared
#define CParametricApproximator_Declared

#include "random.h"

#include <iosfwd>

class CParametricApproximator // pa
{
 public: //////////////////////////////////////////////////////////////////
  virtual int GetWeights() const = 0;
  virtual double GetWeight(int i) const = 0;

  virtual void SetWeight(int i, double x) = 0;
  virtual void SetRandom(CRandom<unsigned> &rnd);
  virtual void SetZero();
  virtual void SetAll(double x);
  virtual void SetInput(const double *px) = 0;
  virtual void AddToWeights(double Eta, const double *pAdd) = 0;

  // ??? These two functions should be outside the class ?
  void BinaryWrite(std::ostream &os) const;
  void BinaryRead(std::istream &is);

  virtual ~CParametricApproximator() {}
};

#endif
