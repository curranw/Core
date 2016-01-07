////////////////////////////////////////////////////////////////////////////
//
// CGreedyPolicy.h
//
// Remi Coulom
//
// March, 2002
//
////////////////////////////////////////////////////////////////////////////
#ifndef CGreedyPolicy_Declared
#define CGreedyPolicy_Declared

#include "CPolicy.h"
class COptimalControlProblem;
class CValueFunction;

class CGreedyPolicy: public CPolicy // greedypolicy
{
 private: /////////////////////////////////////////////////////////////////
  CValueFunction &valf;
  COptimalControlProblem &ocp;

 public: //////////////////////////////////////////////////////////////////
  CGreedyPolicy(CValueFunction &valfInit, COptimalControlProblem &ocpInit);

  void GetControl(const double *px, double *pu) const;
};

#endif
