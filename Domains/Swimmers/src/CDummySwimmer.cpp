/////////////////////////////////////////////////////////////////////////////
//
// CDummySwimmer.cpp
//
// Remi Coulom
//
// December, 2002
//
/////////////////////////////////////////////////////////////////////////////
#include "CDummySwimmer.h"
#include "CSwimmer.h"

/////////////////////////////////////////////////////////////////////////////
// Dummy swimmer controller
//
// This function takes two parameters:
//  px is a pointer to an array that represents the state of the swimmer
//  pu is a pointer to an array of control torques
//
// More details about the model of swimming dynamics can be found in section
// B.4 of my thesis, which can be downloaded at http://remi.coulom.free.fr/
//
// This function should fill the pu array. swimmer.GetControlDimension() returns
// the number of elements of this array, which is equal to the number of joints in
// the body of the swimmer. This array should be filled so that each element pu[i]
// has a value between swimmer.GetUMin(i) and swimmer.GetUMax(i).
//
// The meaning of the elements in the px array is as follows:
//  px[0] is the derivative with respect to time of the abscissa (x) of the
//        center of mass of the swimmer.
//  px[1] is the derivative with respect to time of the ordinate (y) of the
//        center of mass of the swimmer.
//  px[2] is the angle of the first segment with respect to the X-axis (that is
//        the target direction)
//  px[3] is the derivative of px[2] with respect to time
//  px[4] is the angle of the second segment with respect to the X-axis
//  px[5] is the derivative of px[4] with respect to time
//
// and so on for all the following angles. swimmer.GetSegments() returns
// the number of segments of the swimmer. px[2 * swimmer.GetSegments()] is
// the angle of the last segment with respect to the X-axis and
// px[2 * swimmer.GetSegments() + 1] its derivative with respect to time.
//
// The challenge is to rewrite this function so that the swimmer moves
// in the direction of the X-axis as fast as possible.
//
/////////////////////////////////////////////////////////////////////////////
void CDummySwimmer::GetControl(const double *px, double *pu) const
{
 for (int i = swimmer.GetControlDimension(); --i >= 0;)
  pu[i] = (i & 1) ? 0 : swimmer.GetUMax(i);
}
