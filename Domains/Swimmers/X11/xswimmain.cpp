/////////////////////////////////////////////////////////////////////////////
//
// xswimmain.cpp
//
// R�mi Coulom
//
// December, 2002
//
/////////////////////////////////////////////////////////////////////////////
#include "CXSwimmer.h"
#include "CSwimmer.h"
#include "CDummySwimmer.h"

/////////////////////////////////////////////////////////////////////////////
// Main function
/////////////////////////////////////////////////////////////////////////////
int main()
{
 CSwimmer swimmer(7);
 CDummySwimmer dummy(swimmer);
 CXSwimmer swimmerwin(swimmer, dummy);
 swimmerwin.Animate();
 return 0;
}
