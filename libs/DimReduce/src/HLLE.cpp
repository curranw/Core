#include "HLLE.h"
#include "MatVec.h"
#include "MatSym.h"
#include "MatVecDecomp.h"
#include "DimReduceExcept.h"
#include "argsort.h"
#include <iostream>
#include <sstream>
#include <math.h>


/************************************************************
  cmp_training_proj_()
************************************************************/

void HLLE::cmp_training_proj_()
{
  int dp = d_out_*(d_out_+1)/2;
  int N = Ntrain_;
  
  if (!quiet_)
    std::cout << "HLLE::compute_projection()\n"
	      << " - Performing HLLE on " << N <<" points in " 
	      << d_in_ << "->"<< d_out_ << " dimensions.\n";
  
  //-------------------------------------------------
  // Find k nearest neighbors to each point
  compute_training_nbrs();
  
  //------------------------------------------------
  // Find weight matrix
  compute_training_weights();
  Matrix<double>& W = weight_matrix_;
  
  //-----------------------------------------------
  /* now we find the null space of W by finding the first
     d+1 eigenvectors of W^T * W */
  if(!quiet_) 
    std::cout << " - Constructing ["<<N<<" x "<<N
	      <<"] Covariance Matrix.\n";
  
  Matrix<double,SYM> C = W.Transpose() * W;
  
  //clear the memory
  weight_matrix_.reallocate(0,0);
  
  training_proj_.reallocate(d_out_,N);
  
  if (!quiet_){
    std::cout << " - Finding null space of weight matrix with ARPACK\n";
    std::cout << "    + ";
  }

  if (!quiet_)
    std::cout << "   + ";
  EIGS_AR C_EIGS(C,d_out_,"SM",1);

  //copy values to training_proj_
  training_proj_ = C_EIGS.evecs.Transpose();
  
  if (!quiet_)
    std::cout << " - Normalizing null space with SVD\n";
  
  training_proj_ *= sqrt(N);
  
  /* now we need to normalize Y = projection
   * we need R = (Y.T*Y)^(-1/2)
   *   do this with an SVD of Y
   *      Y = U*sig*V.T
   *      Y.T*Y = (V*sig.T*U.T) * (U*sig*V.T)
   *            = V*(sig^2)*V.T
   *   so
   *      R = V * sig^-1 * V.T
   * 
   *   and our return value is
   *      Y*R
   */
  
  SVD proj_SVD(training_proj_);
  
  Matrix<double>Smat(d_out_,d_out_,0.0);
  for(int i=0;i<d_out_;i++)
    Smat(i,i) = 1.0 / proj_SVD.S(i);
  
  Matrix<double> R = training_proj_ * proj_SVD.VT.Transpose();
  Matrix<double> R2 = R*Smat;
  training_proj_ = R2 * proj_SVD.VT;
}


/************************************************************
  cmpweight_(bool train)
************************************************************/
void HLLE::cmpweight_(bool train)
{
  if(train==false){
    //regular LLE weight matrix should be used if we are
    // adding new points to an existing projection
    LLE::cmpweight_(train);
  }
  else{
    int dp = ( d_out_*(d_out_+1) )/2;
    int N = Ntrain_;
  
    if(!quiet_) 
      std::cout << " - Constructing [" 
		<< dp*N << " x " << N
		<< "] weight matrix.\n";

    weight_matrix_.reallocate(dp*N,N);

    Matrix<double>& W = weight_matrix_;

    Matrix<double> neighborhood(d_in_,k_);
    Vector<double> n_center(d_in_);
    Matrix<double> Yi(k_,1+d_out_+dp);
  
    //D_mm: a diagonal matrix, with diagonal elements accessed
    // by D_mm_diag
    Matrix<double> D_mm(k_,k_,0.0);
    Vector<double> D_mm_diag = D_mm.diag();
      
    Vector<double> temp1;
    Vector<double> temp2;
    Vector<double> temp3;
  
    for (size_t i=0;i<N;i++)
    {
      //obtain all points in the neighborhood
      // and subtract their centroid
      n_center.SetAllTo(0.0);

      for(int j=0;j<k_;j++)
	{
	  neighborhood.col(j) = training_data_.col(training_nbrs_(j,i));
	  n_center += neighborhood.col(j);
	}
      n_center /= k_;
      
      for(int j=0;j<k_;j++){
	neighborhood.col(j) -= n_center;
      }
      
      //Compute local coordinates using SVD
      SVD n_SVD(neighborhood);

      /*note: given NC = neighborhood.ncols()
                    NR = neighborhood.nrows()
	if NR >= NC, U is overwritten on neighborhood
	otherwise, U is written to VT
      */
      
      /* Now construct Yi such that:
	  column 0 is all 1
	  column 1...d_out is V, where SVD of neighborhood = U * s * V^T
	  column d_out+1...d_out+dp is the hessian estimator 
	                            of the neighborhood */
      Yi.col(0).SetAllTo(1.0);

      for(size_t j=0; j<d_out_; j++){
	Yi.col(j+1) = n_SVD.VT.row(j);
      }
      
      int count = 0;
      
      for(int mm=0;mm<d_out_;mm++)
	for(int nn=mm;nn<d_out_;nn++){
	  D_mm.diag() = Yi.col(mm+1);
	  Yi.col(1+d_out_+count) = D_mm * Yi.col(nn+1);
	  count++;
	}
      
      /* Orthogonalize the linear and quadratic forms with
	 a QR factorization of Yi */
      QRD Yi_QRD(Yi);
      
      /* present w is given by the cols of Yi
	 between d_out+1 and dp+d_out+1 */

      /* Make cols of w sum to one */
      double S;
      for(int j=d_out_+1; j<dp+d_out_+1;j++){
	S =  temp1.SumElements();
	if(S<1E-4) break;
	
	Yi_QRD.Q.col(j) /= S;
      }
      
      /* Put weights in Weight matrix */
      for(int j=0;j<k_;j++){
	W.col( training_nbrs_(j,i) ).SubVector(i*dp,(i+1)*dp) =
	  Yi_QRD.Q.row(j).SubVector(d_out_+1,dp+d_out_+1);
      }
    }
  }
}

/************************************************************
  check_inputs()
************************************************************/
void HLLE::check_inputs(){
  LLE::check_inputs();
  if (k_<=d_in_)
    throw TrainingError("HLLE Error: k must be greater than d_in\n");
}


/************************************************************
  ID()
************************************************************/
std::string HLLE::ID(){
  std::ostringstream oss;
  
  oss << "HLLE" << "K" << k_ << "D" << d_out_;
  return oss.str();
}
