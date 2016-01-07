/////////////////////////////////////////////////////////////////////////////
//
// CDummySwimmer.h
//
// Rémi Coulom
//
// December, 2002
//
/////////////////////////////////////////////////////////////////////////////
#ifndef CDummySwimmer_Declared
#define CDummySwimmer_Declared

#include "CPolicy.h"

class CSwimmer;

class CDummySwimmer: public CPolicy
{
 private:
  const CSwimmer &swimmer;

 public: 
  CDummySwimmer(const CSwimmer &swimmerInit): swimmer(swimmerInit) {}

  void GetControl(const double *px, double *pu) const;
};

#endif
