#ifndef HLLE_H
#define HLLE_H

#include "MatVec.h"
#include "LLE.h"

/***************************************************
  HLLE class to compute the d_out - dimensional Hessian 
   Eigenmapping Embedding of a matrix using the 
   k nearest neighbors

    Implementation based on algorithm outlined in
     'Hessian Eigenmaps: new locally linear embedding techniques
      for high-dimensional data'
        by C. Grimes and D. Donoho, March 2003

    Created by Jake Vanderplas
     vanderplas@astro.washington.edu
     November 2008
****************************************************/
class HLLE : public LLE{
 private:
  void cmpweight_(bool train);
  void cmp_training_proj_();
 public:
  HLLE() : LLE(){}
  HLLE(const Matrix<double>& training_data) : LLE(training_data){}
  HLLE(const std::string& filename) : LLE(filename){}

  void check_inputs();
  std::string ID();
};

#endif /*HLLE_H*/
