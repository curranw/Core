#include "PCA.h"
#include "MatVecDecomp.h"
#include "DimReduceExcept.h"
#include <string>
#include <sstream>
#include <iostream>
#include <math.h>



/************************************************************
  constructors
************************************************************/
PCA::PCA(){
  init();
}

PCA::PCA(const Matrix<double>& training_data)
{
  init();
  set_training_data(training_data);
}

PCA::PCA(const std::string& filename)
{
  init(); 
  load_fits(filename);
}

/************************************************************
  init()
   initializes all class data
************************************************************/
void PCA::init(){
  Ntrain_ = -1;
  d_in_ = -1;
  d_out_ = -1;
  
  var_ = -1.0;
  total_variance_ = 0.0;
  
  learn_d_out_ = false;
  quiet_ = false;
  
  has_training_data_ = false;
  has_training_proj_ = false;
  has_eigs_ = false;
}

/************************************************************
 *  load_fits(const std::string& fits_file)
 ************************************************************/
void PCA::load_fits(const std::string& fits_file){
  std::string filename = fits_file;
  
  if( split_fits_title(filename) )
    throw TrainingError("load_fits: cannot use fits extension\n");

  load_training_data(filename);

  check_inputs();
}

/************************************************************
  update_training_file()
************************************************************/
void PCA::update_training_file()
{
  if(has_training_proj_){
    training_file_.append_image(training_proj_,ID());
    training_file_.append_image(training_data_mean_,mean_ID());
    training_file_.append_image(eigenvectors_,evecs_ID());
    training_file_.move_to_hdu(evecs_ID());
    
    training_file_.write_key("VAR_TOT",total_variance_,
			     "sum of eigenvalues");

    for(int i=0;i<d_out_;i++){
      std::ostringstream oss1;
      oss1 << "SVAL" << i+1;
      std::ostringstream oss2;
      oss2 << "singular value " << i+1;
      training_file_.write_key(oss1.str(),
			       singular_values_(i),
			       oss2.str());
    }
  }
}


/************************************************************
  load_training_[data/proj](const std::string& fits_file)
************************************************************/
void PCA::load_training_data(const std::string& fits_file){
  if(has_training_data_)
    throw TrainingError("training_data is already set\n");
  training_file_.open_file(fits_file);
  training_file_.read_image(training_data_);
  
  d_in_ = training_data_.nrows();
  Ntrain_ = training_data_.ncols();
  has_training_data_ = true;
}

void PCA::load_training_proj(const std::string& fits_file){
  if(has_training_proj_)
    throw TrainingError("training_proj is already set\n");
  fitstools tpf(fits_file);
  tpf.read_image(training_proj_);
  if(training_proj_.ncols() != Ntrain_)
    throw TrainingError("number of points in training_proj must "
			"match number of points in training_data\n");
  d_out_ = training_proj_.nrows();
  tpf.close_file();
  has_training_proj_ = true;
}

/************************************************************
  set_training_[data/proj](const Matrix<double>& training_data)
************************************************************/
void PCA::set_training_data(const Matrix<double>& training_data){
  if(has_training_data_)
    throw TrainingError("training_data is already set\n");
  training_data_.viewof(training_data);
  d_in_ = training_data_.nrows();
  Ntrain_ = training_data_.ncols();
  has_training_data_ = true;
}

void PCA::set_training_proj(const Matrix<double>& training_proj){
  if(has_training_proj_)
    throw TrainingError("training_proj is already set\n");
  if(training_proj.ncols() != Ntrain_)
    throw TrainingError("number of points in training_proj must "
			"match number of points in training_data\n");
  training_proj_.viewof(training_proj);
  d_out_ = training_proj_.nrows();
  has_training_proj_ = true;
}


/************************************************************
  clear_training_[data/proj]()
************************************************************/
void PCA::clear_training_data(){
  if(!has_training_data_)
    throw TrainingError("cannot clear_training_data: data is not set\n");
  training_data_.reallocate(0,0);
  Ntrain_ = 0;
  if(has_training_proj_)
    clear_training_proj();
  training_file_.close_file();
  has_training_data_ = false;
}

void PCA::clear_training_proj(){
  if(!has_training_proj_)
    throw TrainingError("cannot clear_training_proj: data is not set\n");
  training_proj_.reallocate(0,0);
  training_data_mean_.reallocate(0);
  eigenvectors_.reallocate(0,0);
  singular_values_.reallocate(0);
  training_reconstruction_.reallocate(0,0);
}


/************************************************************
  check_inputs()
************************************************************/
void PCA::check_inputs()
{
  //check inputs to make sure they make sense
  if (Ntrain_ <= 0)
    throw TrainingError("PCA Error: no training data supplied\n");
  
  if(learn_d_out_)
    {
      if ((var_<0) || (var_>1.0))
	throw TrainingError("PCA Error: variance must be in range "
			    "0.0 < var < 1.0\n");  
    }
  else
    {
      if (d_out_ <= 0)
	throw TrainingError("PCA Error: d_out must be positive\n");
      
      if (d_in_ < d_out_)
	throw TrainingError("PCA Error: output dimension must be "
			    "less than input dimension");
    }
}


/************************************************************
  ID()
************************************************************/
std::string PCA::ID(){
  std::ostringstream oss;
  oss << "PCA" << "D" << d_out_;
  return oss.str();
}


/************************************************************
 * compute()
 ************************************************************/
void PCA::compute(PCA_TYPE type){
  check_inputs();

  if(!quiet_){
    std::cout << "PCA::compute : \n"
	      << " - performing PCA on " << Ntrain_ << " points in "
	      << d_in_ << " dimensions.\n";
  }
    
  //calculate mean and center data
  training_data_mean_.reallocate(d_in_);
  Matrix<double> X_centered;
  X_centered.deep_copy( training_data_ );

  cmp_mean_Xc_(X_centered);

  //compute output dimension
  cmp_output_dim_(X_centered);
  
  //do eigen-analysis
  if(type == SVD_PCA){
    cmp_projection_SVD_(X_centered);
  }
  else if(type == EIG_PCA){
    cmp_projection_EIG_(X_centered);
    cmp_proj_from_eigs_(X_centered);
  }
  else if(type == AREIG_PCA){
    cmp_projection_AREIG_(X_centered);
    cmp_proj_from_eigs_(X_centered);
  }
  else{
    throw TrainingError("unrecognized PCA type");
  }
}



void PCA::cmp_mean_Xc_(Matrix<double>& X_centered){
  int N = Ntrain_;
  
  for(int i=0;i<d_in_;i++){
    training_data_mean_(i) = training_data_.row(i).SumElements();
  }
  training_data_mean_ /= N;
  
  for(int j=0;j<N;j++){
    X_centered.col(j) -= training_data_mean_;
  }
  
  X_centered /= sqrt(N-1);
}


void PCA::cmp_output_dim_(Matrix<double>& X_centered){
  SVD XC_SVD(X_centered,false); //compute singular values only
  
  total_variance_ = blas_NRM2(XC_SVD.S);
  total_variance_ *= total_variance_;
  
  if(learn_d_out_){
    if(!quiet_)
      std::cout << " - calculating d_out for variance = " << var_ << "\n";
    //compute d_out from variance
    double variance = 0.0;
    d_out_ = 0;

    do{
      variance += pow(XC_SVD.S(d_out_) , 2) / total_variance_;
      ++d_out_;
    }while(variance < var_ && d_out_ < d_in_);

    learn_d_out_ = false;
    if(!quiet_)
      std::cout << "     d_out = " << d_out_ << "\n";

  }else{
    if(!quiet_)
      std::cout << " - calculating variance for d_out = " << d_out_ << "\n";
    //compute variance from d_out
    var_ = 0.0;
    for(int i=0;i<d_out_;i++){
      var_ += pow(XC_SVD.S(i) , 2) / total_variance_;
    }
    if(!quiet_)
      std::cout << "     variance = " << var_ << "\n";
  }
}
  

void PCA::cmp_projection_SVD_(Matrix<double>& X_centered){
    if(!quiet_)
      std::cout << " - calculating full projection with SVD\n";
    
    SVD X_SVD(X_centered,true,true); //compute_uv, and overwrite X_centered
    
    //copy eigenvalues
    singular_values_.reallocate(d_out_);
    singular_values_ = X_SVD.S.SubVector(0,d_out_);
    
    //copy eigenvectors
    eigenvectors_.reallocate(d_in_,d_out_);
    for(int i=0;i<d_out_;i++)
      eigenvectors_.col(i) = X_SVD.U.col(i);
    
    has_eigs_ = true;
    
    //copy training_proj
    training_proj_.reallocate(d_out_,Ntrain_);
    for(int i=0;i<d_out_;i++)
      training_proj_.row(i) = X_SVD.VT.row(i);
    
    has_training_proj_ = true;
}

void PCA::cmp_projection_EIG_(Matrix<double>& X_centered){
    if(!quiet_)
      std::cout << " - calculating full eigen-decomposition with LAPACK\n";
    
    Matrix<double,SYM> C = X_centered * X_centered.Transpose();
    EIGS C_EIGS(C);
    
    //copy eigenvalues
    singular_values_.reallocate(d_out_);
    for(int i=0;i<d_out_;i++)
      singular_values_(i) = sqrt( C_EIGS.evals(d_in_-1-i) );
    
    //copy eigenvectors
    eigenvectors_.reallocate(d_in_,d_out_);
    for(int i=0;i<d_out_;i++)
      eigenvectors_.col(i) = C_EIGS.evecs.col(d_in_-1-i);
    
    has_eigs_ = true;
    has_training_proj_ = false;
}

void PCA::cmp_projection_AREIG_(Matrix<double>& X_centered){
    if(!quiet_)
      std::cout << " - calculating partial eigen-decomposition with ARPACK\n";
    
    Matrix<double,SYM> C = X_centered * X_centered.Transpose();
    EIGS_AR C_EIGS(C,d_out_,"LM");
    
    //copy eigenvalues
    singular_values_.reallocate(d_out_);
    for(int i=0;i<d_out_;i++)
      singular_values_(i) = sqrt( C_EIGS.evals(d_out_-1-i) );
    
    //copy eigenvectors
    eigenvectors_.reallocate(d_in_,d_out_);
    for(int i=0;i<d_out_;i++)
      eigenvectors_.col(i) = C_EIGS.evecs.col(d_out_-1-i);
    
    has_eigs_ = true;
    has_training_proj_ = false;
}
  

void PCA::cmp_proj_from_eigs_(Matrix<double>& X_centered){
  if(!quiet_)
    std::cout << " - calculating training projection "
	      << "from eigen-decomposition\n";
  
  //construct training_proj_
  training_proj_.reallocate(d_out_,Ntrain_);
  Matrix<double> sing_val_matrix(d_out_,d_out_,0.0);
  for(int i=0;i<d_out_;i++)
    sing_val_matrix(i,i) = 1.0 / singular_values_(i);
  Matrix<double> tmp = sing_val_matrix * eigenvectors_.Transpose();
  training_proj_ = tmp * X_centered;
  
  has_training_proj_ = true;
}



const Matrix<double>& PCA::reconstruct(int neigs){
  if(!has_training_proj_){
    d_out_ = neigs;
    compute();
  }
  
  if(neigs>0 && neigs != d_out_){
    std::cerr << "using " << d_out_ << " eigs for reconstruction\n"
	      << "  (" << neigs << " requested)\n";
  }

  training_reconstruction_.reallocate(d_in_,Ntrain_,0.0);

  Matrix<double> sing_value_matrix(d_out_,d_out_);
  sing_value_matrix.diag() = singular_values_;

  Matrix<double> tmp = eigenvectors_ * sing_value_matrix;

  training_reconstruction_ = tmp * training_proj_;

  training_reconstruction_ *= sqrt(Ntrain_-1);

  for(int i=0;i<Ntrain_;i++)
    training_reconstruction_.col(i) += training_data_mean_;

  return training_reconstruction_;
}


/************************************************************
 * WPCA
 ************************************************************/

WPCA::WPCA(){
  init();
}

WPCA::WPCA(const Matrix<double>& training_data,
	   const Vector<double>& weights){
  init();
  set_training_data(training_data);
  set_weights(weights);
}

void WPCA::init(){
  has_weights_ = false;
  PCA::init();
}


void WPCA::cmp_mean_Xc_(Matrix<double>& X_centered){
  int N = Ntrain_;

  for(int i=0;i<d_in_;i++){
    training_data_mean_(i) = blas_DOT( training_data_.row(i), weights_);
  }
  training_data_mean_ /= weights_.SumElements();
  
  for(int i=0;i<N;i++){
    X_centered.col(i) -= training_data_mean_;
    X_centered.col(i) *= sqrt(weights_(i));
  }
  
  X_centered /= sqrt(N-1);
}
  

void WPCA::load_fits(const std::string& fits_file){
  std::string filename = fits_file;
  
  if( split_fits_title(filename) )
    throw TrainingError("load_fits: cannot use fits extension\n");

  load_training_data(filename);
  if( training_file_.ext_in_file(weights_ID()) ){
    training_file_.move_to_hdu(weights_ID());
    training_file_.read_image(weights_);
    has_weights_ = true;
  }else{
    throw TrainingError("weights must be provided for WPCA");
  }
  if(weights_.size() != Ntrain_)
    throw TrainingError("weights size must match number of data points\n");
  check_inputs();
}


void WPCA::load_weights(const std::string& fits_file){
  if(has_weights_)
    throw TrainingError("weights are already set\n");
  training_file_.open_file(fits_file);
  training_file_.read_image(weights_);
  
  if(weights_.size() != Ntrain_)
    throw TrainingError("weights size must match number of data points\n");
  has_weights_ = true;
}


void WPCA::set_weights(const Vector<double>& weights){
  if(has_weights_)
    throw TrainingError("weights are already set\n");
  if(weights.size() != Ntrain_)
    throw TrainingError("size of weights must "
			"match number of points in training_data\n");
  weights_.viewof(weights);
  has_weights_ = true;
}

void WPCA::clear_weights(){
  if(!has_weights_)
    throw TrainingError("cannot clear_weights: weights not set\n");
  weights_.reallocate(0);
  has_weights_ = false;
}


//update_training_file()
void WPCA::update_training_file()
{
  PCA::update_training_file();
  if(has_weights_){
    training_file_.append_image(weights_,weights_ID());
  }
}


/************************************************************
 * IRWPCA:
 *   Iteratively Reweighted PCA.
 *
 *   Essentially, normal PCA tries to find the matrix B which minimizes
 *    (in a least-squares sense)
 *      E = sum(e_j^2)
 *     where
 *      e_j = | (x_j-mu) - B*B^T*(x_j-mu) |^2
 *
 *   mu is the sample mean, and B is the matrix whose columns are the 
 *    eigenvectors of the covariance C = Sum_j[ (x_j-mu) * (x_j-mu)^T ]
 *
 *   In IRWPCA, we create weights based on the errors e_j, and iterate
 *    such that the points with largest e_j are deweighted, until 
 *    mu and B converge
 *
 ************************************************************/

IRWPCA::IRWPCA(int max_iter, double tol) 
  : max_iter_(max_iter), tol_(tol){
  init();
}



void IRWPCA::load_fits(const std::string& fits_file){
  std::string filename = fits_file;
  
  if( split_fits_title(filename) )
    throw TrainingError("load_fits: cannot use fits extension\n");

  load_training_data(filename);

  check_inputs();
}



void IRWPCA::compute(PCA_TYPE type/* = SVD_PCA*/){
  int N = training_data_.ncols();
  weights_.reallocate(N,1.0);
  has_weights_ = true;


  if(!quiet_){
    std::cout << "IRWPCA::compute : \n"
	      << " - performing PCA on " << Ntrain_ << " points in "
	      << d_in_ << " dimensions.\n"
	      << "    > max_iter = " << max_iter_ << "\n"
	      << "    > tol = " << tol_ << "\n";
  }
  
  if (max_iter_<1)
    throw TrainingError("max_iter must be positive");
  
  bool Q = quiet_;
  quiet_ = true;
  //find initial guess with equal weights
  WPCA::compute(type);
  
  //storage needed for procedure
  Matrix<double,SYM> B_BT(d_in_);
  Vector<double> V(d_in_);
  Vector<double> V_minus_mu(d_in_);
  
  Vector<double> weights_old;
  Matrix<double> evecs_old;

  int count;

  for(count=1;count<max_iter_;count++)
    {
      weights_old.deep_copy(weights_);
      evecs_old.deep_copy(eigenvectors_);

      B_BT = eigenvectors_ * eigenvectors_.Transpose();
      
      //find errors, store them in weights_ vector
      for(int i=0;i<N;i++)
	{
	  V = training_data_.col(i) - training_data_mean_;
	  V_minus_mu.deep_copy(V);
	  V -= B_BT * V_minus_mu;
	  weights_(i) = pow( blas_NRM2(V) , 2 );
	}     
      
      //convert errors to new weights using Huber function
      //  (Huber 1973)
      double c = 0.5 * weights_.SumElements() / Ntrain_;
      for(int i=0;i<Ntrain_;i++)
	weights_(i) = (weights_(i)>c) ? c/weights_(i) : 1.0;
      weights_ /= weights_.SumElements();

      //compute new weighted mean and eigenvectors
      WPCA::compute(type);
      
      /**  check for convergence: **/
      
      //  RMS of new weights minus old weights
      weights_old -= weights_;
      double Wres = sqrt( blas_NRM2(weights_old) / weights_old.size() );
      
      //  RMS of new eigenvectors minus old eigenvectors
      evecs_old -= eigenvectors_;
      double Tres = sqrt( blas_NRM2(evecs_old) / evecs_old.size() );
	
      if ( (Wres<tol_) ){ //&& (Tres<tol_) ){
	if (!Q)
	  std::cout << "IRWPCA: converged to tol = " << tol_ 
		    << " in " << count << " iterations\n";
	break;
      }
      std::cout << " Wres = " << Wres << "\n Tres = " << Tres << "\n"
		<< "   (Note: checking only for Wres convergence)\n";
    }//end for

  //if tolerance was not reached, return an error message
  if(count == max_iter_)
    throw IterException(max_iter_,tol_);
  
  Matrix<double> X_centered(d_in_,Ntrain_);
  X_centered = training_data_;
  for(int i=0;i<Ntrain_;i++)
    X_centered.col(i) -= training_data_mean_;
  X_centered /= sqrt(Ntrain_-1);
  cmp_proj_from_eigs_(X_centered);

  quiet_ = Q;
}
