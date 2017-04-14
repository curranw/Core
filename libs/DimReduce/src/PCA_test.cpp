//#include "PCA.h"
//#include <math.h>

//int main(){
//  int D=10;
//  int N=100;
//  Matrix<double> M(D,N);
//  for(int i=0; i<D; i++)
//    for(int j=0; j<N; j++)
//      M(i,j) = (std::min(i,j)+2) * (std::max(i,j)-1);
    
//  //std::cout << "data input: " << M << "\n\n";
  
//  PCA M_PCA(M);

//  //M_PCA.set_var(0.9);
//  M_PCA.set_d_out(9);

//  for(int i=0;i<3;i++){
//    std::cout << "---------------------------------------------------------\n";
//    M_PCA.compute(PCA_TYPE(i));
    
//    //std::cout << "training proj: " << M_PCA.training_projection() << "\n";

//    std::cout << "singular values: " << M_PCA.singular_values() << "\n";
    
//    //std::cout << "eigenvectors: " << M_PCA.eigenvectors() << "\n";
    
//    //std::cout << "reconstruction: " << M_PCA.reconstruct() << "\n";

//    Matrix<double> err = M - M_PCA.reconstruct();

//    std::cout << "RMS: " << sqrt( blas_NRM2(err)/N/D ) << "\n";

//    std::cout << "variance conserved: " << M_PCA.var() << "\n";
//    std::cout << "dimensions used: " << M_PCA.d_out() << "\n";
//  }



//  return 0;
//}




