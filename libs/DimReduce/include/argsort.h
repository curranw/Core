#ifndef ARGSORT_H
#define ARGSORT_H

#include "MatVec.h"
#include <algorithm>
//structure to help argsort
template <class T>
struct LT_Indices{
  Vector<T>* x_ptr;
  LT_Indices(Vector<T> &x) {x_ptr = &x;}
  bool operator()(int i1, int i2) const {return ((*x_ptr)(i1) < (*x_ptr)(i2));}
};


//function argsort
// given a vector x,
//  returns a vector of indices to sort x in increasing order
template <class T>
void argsort(Vector<T>& x , Vector<int>& indices)
{
  int N = x.size();
  
  if(indices.size() != N)
    indices.reallocate(N,0);
  
  for(int i=0;i<N;i++)
    indices(i)=i;

  std::sort(indices.arr(), indices.arr()+N, LT_Indices<T>(x));
}

#endif //ARGSORT_H
