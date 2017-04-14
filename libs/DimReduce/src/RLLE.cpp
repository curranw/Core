#include "RLLE.h"
#include "HLLE.h"
#include "LLE.h"
#include "IRWPCA.h"
#include "argsort.h"
#include "DimReduceExcept.h"
#include "fitstools.h"

#include <iostream>
#include <math.h>

void RLLE_::update_training_file(){
  LLE::update_training_file();
  if(has_r_scores_)
    training_file_.append_image(r_scores_,r_scores_ID());
}
  

void RLLE_::load_fits(const std::string& fits_file){
  LLE::load_fits(fits_file);
  std::string filename = fits_file;
  split_fits_title(filename);

  if(!recalc_){
    if( training_file_.ext_in_file( r_scores_ID() ) ){
      training_file_.move_to_hdu( r_scores_ID() );
      training_file_.read_image(r_scores_);
      has_r_scores_ = true;
    }
  }
}
    

void RLLE1::cmp_training_proj_()
{
  int N = Ntrain_;
  
  if (!quiet_)
    std::cout << "RLLE1::compute_projection():\n"
	      << " - Performing RLLE on " << N <<" points in " 
	      << d_in_ << "->"<< d_out_ << " dimensions.\n";
  
  // Determine nearest neighbors and r_scores
  compute_r_scores();
  
  //------------------------------------------------
  // Determine how many points to cut
  int N_cut = int(r_*N);
  
  Vector<int> indices;
  argsort(r_scores_,indices);
  double R_cutoff = r_scores_(indices(N_cut));
  
  //------------------------------------------------
  // Construct a matrix of good data
  if (!quiet_)
    std::cout << " - Cutting " << N_cut << " out of " << N
	      << " points with reliability scores < " << R_cutoff << "\n";
  
  Matrix<double>data_trunc(d_in_,N-N_cut);
  for(int i=N_cut;i<N;i++){
    data_trunc.col(i-N_cut) = training_data_.col(indices(i));
  }
  //------------------------------------------------
  // use LLE to obtain projection
  if (!quiet_){
    std::cout << " - Using ";
    if(hessian_) std::cout << "H";
    std::cout << "LLE to project truncated data\n";
  }
  
  Matrix<double> proj_trunc;
  LLE* LLEobj;
  if(hessian_)
    LLEobj = new HLLE(data_trunc);
  else
    LLEobj = new LLE(data_trunc);
  
  LLEobj->set_quiet(quiet_);
  LLEobj->set_k(k_);
  LLEobj->set_d_out(d_out_);
  
  LLEobj->compute_projection();
  LLEobj->compute_projection(training_data_);
  training_proj_.deep_copy( LLEobj->test_projection() );
  
  delete LLEobj;
}

void RLLE2::cmp_training_proj_(){
  int N = Ntrain_;
  
  if (!quiet_)
    std::cout << "RLLE2::compute_projection():\n"
	      << " - Performing RLLE on " << N <<" points in " 
	      << d_in_ << "->"<< d_out_ << " dimensions.\n";
  
  // Determine nearest neighbors and r_scores
  compute_r_scores();
  
  //-------------------------------------------------
  // Find k nearest neighbors to each point
  compute_weighted_neighbors();
  
  //-------------------------------------------------
  // Find weight matrix
  compute_training_weights();
  
  //------------------------------------------------
  // Find null space to obtain projection
  if (!quiet_){
    std::cout << " - Finding null space of weight matrix\n"
	      << "    + constructing Covariance matrix\n";
  }
  /* for LLE we use M = (I-W)^T * (I-W)
   *  and find the null space of M using the eigenvalue problem
   *   M * v = lam * v
   *
   * for RLLE2 we use the weights in R_scores
   *  let S = R_scores^2
   *  find the null space of (S*M) using the general eigenvalue problem
   *    M * v = lam * S^-1 * v
   */
  
  //let W = (W-I)
  Matrix<double>& W = weight_matrix_;
  W.diag() -= 1.0;
  
  Matrix<double,SYM> Cov = W.Transpose() * W;
  
  //clear weight matrix: it's no longer needed
  W.reallocate(0,0);
  
  Matrix<double> S(N,N,0.0);
  
  //let S.diag() = R^-2.  If R[i]==0, set it to 1E-15
  for(int i=0;i<N;i++)
    {
      if(r_scores_(i)>0)
	S(i,i) = 1.0 / pow(r_scores_(i),2);
      else
	S(i,i) = 1E30;
    }
  
  if (!quiet_)
    std::cout << "    + finding null space of Covariance Matrix with ARPACK\n";
  
  //now find eigenvectors of W...
  training_proj_.reallocate(d_out_,N);

  if (!quiet_)
    std::cout << "    + ";

  EIGS_AR C_EIGS(Cov,d_out_,"SM",1);
  
  if (!quiet_)
    std::cout << " - Success!!\n";

  //copy values to training_proj_
  training_proj_ = C_EIGS.evecs.Transpose();
}


void RLLE_::set_r_scores(const Vector<double>& r_scores){
  if(has_r_scores_)
    throw TrainingError("r_scores are already set\n");
  if(r_scores.size() != Ntrain_)
    throw TrainingError("size of r_scores must match number "
			"of points in training_data\n");
  r_scores_.viewof(r_scores);
  has_r_scores_ = true;
}


void RLLE_::load_r_scores(const std::string& fits_file){
  if(has_r_scores_)
    throw TrainingError("r_scores are already set\n");
  fitstools ffile(fits_file);
  ffile.read_image(r_scores_);
  if(r_scores_.size() != Ntrain_)
    throw TrainingError("size of r_scores must match number "
			"of points in training_data\n");
  has_r_scores_ = true;
  ffile.close_file();
}

void RLLE_::clear_r_scores(){
  if(!has_r_scores_)
    throw TrainingError("cannot clear_r_scores : r_scores have "
			"not been computed\n");
  r_scores_.reallocate(0);
  has_r_scores_ = false;
}


void RLLE_::compute_r_scores()
{
  if(has_r_scores_){
    if (!quiet_)
      std::cout << " - Using previously computed R_scores\n";
  }else{
    if (!quiet_)
      std::cout << " - Determining reliability scores based on\n"
		<< "    Iteratively Reweighted PCA of each neighborhood\n";
    
    compute_training_nbrs();

    int N = Ntrain_;
    
    r_scores_.reallocate(N,0.0);
    
    //data structures needed for finding scores
    Matrix<double> neighborhood(d_in_,k_);
    
    //data structures needed for IRWPCA procedure
    Matrix<double> trans(d_out_,d_in_);
    Vector<double> mu(d_in_);
    Vector<double> A(k_);

    for (size_t i=0;i<N;i++)
      {
	//construct neighborhood
	for(int j=0;j<k_;j++){
	  neighborhood.col(j) = 
	    training_data_.col(training_nbrs_(j,i)) - training_data_.col(i);
	}
	//find weights with Iteratively Reweighted PCA of neighborhood
	try
	  {
	    IRWPCA(neighborhood,A,mu,trans);
	  }
	catch(IterException& ex)
	  {
	    std::cerr << "   + IRWPCA did not converge for point "
		      << i << ": using equal weights in this neighborhood.\n";
	    A.SetAllTo(1.0/k_);
	  }
	
	//add weights to reliability scores
	for(int j=0;j<k_;j++)
	  r_scores_(training_nbrs_(j,i)) += A(j);
      }
    has_r_scores_ = true;
  }
}


void RLLE2::compute_weighted_neighbors()
{
  if(!quiet_) 
    std::cout << " - finding " << k_ << "+" << r_ << " nearest neighbors "
	      << "of each point\n"
	      << "    and reducing to " << k_ << " most reliable\n";
  int N = Ntrain_;
  double d;
  
  Matrix<double> data_i(d_in_,N);
  Vector<double> D(N);
  Vector<int> indices(N);
  
  for(int i=0;i<N;i++){
    //  find k+r nearest neighbors of point i
    // compute distances from point i to all other points
    for (size_t j=0;j<N;j++){
      data_i.col(j) = training_data_.col(i) - training_data_.col(j);
      D(j) = blas_NRM2( data_i.col(j) );
    }
    // find k+r nearest points
    argsort(D,indices);
    
    //now find the k of these with largest R_scores
    //swap indices until smallest R_scores are in positions k+1...k+r
    for(int j1=k_+r_; j1>k_; j1--)
      for(int j2=1; j2<j1; j2++)
	if( r_scores_(j1) > r_scores_(j2) ) 
	  indices.swap(j1,j2);
    
    //neighbors.col(i) = indices.SubVector(1,k+1)
    training_nbrs_.col(i) = indices.SubVector(1,k_+1);
  }
}



/************************************************************
  check_inputs()          
************************************************************/

void RLLE1::check_inputs()
{
  LLE::check_inputs();

  if(r_>=1 || r_<0)
    throw TrainingError("RLLE1 Error: r must satisfy 0 <= r < 1\n");
}


void RLLE2::check_inputs()
{
  LLE::check_inputs();
  
  if (r_<0)
    throw TrainingError("RLLE2 Error: r must be positive\n");
  
  if (k_+r_>=Ntrain_)
    throw TrainingError("RLLE2 Error: k+r must be less than "
			"number of training points\n");
}

/************************************************************
  ID()
************************************************************/
std::string RLLE1::ID(){
  std::ostringstream oss;
  oss << "RLLE1";
  if(hessian_)
    oss << "H";
  oss << "K" << k_ << "D" << d_out_ << "R" << r_;
  return oss.str();
}

std::string RLLE2::ID(){
  std::ostringstream oss;
  oss << "RLLE2K" << k_ << "D" << d_out_ << "R" << r_;
  return oss.str();
}
