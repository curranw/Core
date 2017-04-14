#include "LLE.h"
#include "DimReduceExcept.h"
#include "MatVec.h"
#include "MatSym.h"
#include "MatVecDecomp.h"
#include "argsort.h"
#include "fitstools.h"
#include "Data.h"
#include <iostream>
#include <sstream>
#include <math.h>


/************************************************************
  constructors
************************************************************/
LLE::LLE(){
  init();
}

LLE::LLE(const Matrix<double>& training_data){
  init();
  set_training_data(training_data);
}

/************************************************************
  init()
   initializes all class data
************************************************************/
void LLE::init(){
  Ntrain_ = -1;
  Ntest_ = -1;
  k_ = -1;
  d_in_ = -1;
  d_out_ = -1;
  
  var_ = -1.0;
  
  recalc_ = false;
  learn_d_out_ = false;
  quiet_ = false;
  
  has_training_data_ = false;
  has_training_proj_ = false;
  has_test_data_ = false;
  has_test_proj_ = false;
  has_sigma_ = false;
}

/************************************************************
 *  load_fits(const std::string& fits_file)
 ************************************************************/
void LLE::load_fits(const std::string& fits_file){
  std::string filename = fits_file;
  
  if( split_fits_title(filename) )
    throw TrainingError("load_fits: cannot use fits extension\n");

  load_training_data(filename);

  check_inputs();
  compute_d_out();
  
  has_training_proj_ = false;
  has_sigma_ = false;

  if(!recalc_){
    if( training_file_.ext_in_file(ID()) ){
      training_file_.move_to_hdu(ID());
      training_file_.read_image(training_proj_);
      has_training_proj_ = true;
    }
    
    if( training_file_.ext_in_file(sigma_ID()) ){
      training_file_.move_to_hdu(sigma_ID());
      training_file_.read_image(sigma_);
      has_sigma_ = true;
    }
    
    //read in neighbors.  If not enough are computed,
    // they will be recalculated
    if( training_file_.ext_in_file( neighbors_ID() ) ){
      training_file_.move_to_hdu(neighbors_ID());
      training_file_.read_image(training_nbrs_);
    }
  }
}


/************************************************************
  load_[test/training]_[data/proj](const std::string& fits_file)
  load_sigma(const std::string& fits_file)
************************************************************/
void LLE::load_training_data(const std::string& fits_file){
  if(has_training_data_)
    throw TrainingError("training_data is already set\n");
  training_file_.open_file(fits_file);
  training_file_.read_image(training_data_);
  
  d_in_ = training_data_.nrows();
  Ntrain_ = training_data_.ncols();
  has_training_data_ = true;
}

void LLE::load_training_proj(const std::string& fits_file){
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

void LLE::load_test_data(const std::string& fits_file){
  if(has_test_data_)
    throw TrainingError("test_data is already set\n");
  test_file_.open_file(fits_file);
  test_file_.read_image(test_data_);
  if(test_data_.nrows() != d_in_)
    throw TrainingError("dimension of test data must match "
			"dimension of training data\n");
  Ntest_ = test_data_.ncols();
  has_test_data_ = true;
}

void LLE::load_test_proj(const std::string& fits_file){
  if(has_test_proj_)
    throw TrainingError("test_proj is already set\n");
  fitstools tpf(fits_file);
  tpf.read_image(test_proj_);
  if(test_proj_.ncols() != Ntest_)
    throw TrainingError("number of points in test_proj must "
			"match number of points in test_data\n");
  tpf.close_file();
  has_test_proj_ = true;
}

void LLE::load_sigma(const std::string& fits_file){
  if(has_sigma_)
    throw TrainingError("sigma is already set\n");
  fitstools ffile(fits_file);
  ffile.read_image(sigma_);
  if(sigma_.size() != Ntrain_)
    throw TrainingError("number of points in sigma_ must "
			"match number of points in training_data\n");
  ffile.close_file();
  has_sigma_ = true;
}

/************************************************************
  set_[test/training]_[data/proj](const Matrix<double>& training_data)
************************************************************/
void LLE::set_training_data(const Matrix<double>& training_data){
  if(has_training_data_)
    throw TrainingError("training_data is already set\n");
  training_data_.viewof(training_data);
  d_in_ = training_data_.nrows();
  Ntrain_ = training_data_.ncols();
  has_training_data_ = true;
}

void LLE::set_training_proj(const Matrix<double>& training_proj){
  if(has_training_proj_)
    throw TrainingError("training_proj is already set\n");
  if(training_proj.ncols() != Ntrain_)
    throw TrainingError("number of points in training_proj must "
			"match number of points in training_data\n");
  training_proj_.viewof(training_proj);
  d_out_ = training_proj_.nrows();
  has_training_proj_ = true;
}

void LLE::set_test_data(const Matrix<double>& test_data){
  if(has_test_data_)
    throw TrainingError("test_data is already set\n");
  if(test_data.nrows() != d_in_)
    throw TrainingError("dimension of test data must match "
			"dimension of training data\n");
  test_data_.viewof(test_data);
  Ntest_ = test_data.ncols();
  has_test_data_ = true;
}

void LLE::set_test_proj(const Matrix<double>& test_proj){
  if(has_test_proj_)
    throw TrainingError("test_proj is already set\n");
  if(test_proj.ncols() != Ntest_)
    throw TrainingError("number of points in test_proj must "
			"match number of points in test_data\n");
  test_proj_.viewof(test_proj);
  has_test_proj_ = true;
}

/************************************************************
  clear_[test/training]_[data/proj]()
************************************************************/
void LLE::clear_training_data(){
  if(!has_training_data_)
    throw TrainingError("cannot clear_training_data: data is not set\n");
  training_data_.reallocate(0,0);
  Ntest_ = 0;
  training_nbrs_.reallocate(0,0);
  if(has_training_proj_)
    clear_training_proj();
  if(has_test_proj_)
    clear_test_proj();
  if(has_sigma_)
    clear_sigma();
  training_file_.close_file();
  has_training_data_ = false;
}

void LLE::clear_training_proj(){
  if(!has_training_proj_)
    throw TrainingError("cannot clear_training_proj: data is not set\n");
  training_proj_.reallocate(0,0);
  has_training_proj_ = false;
}

void LLE::clear_test_data(){
  if(!has_test_data_)
    throw TrainingError("cannot clear_test_data: data is not set\n");
  test_data_.reallocate(0,0);
  Ntest_ = 0;
  test_nbrs_.reallocate(0,0);
  if(has_test_proj_)
    clear_test_proj();
  //test_file_.close_file();
  has_test_data_ = false;
}

void LLE::clear_test_proj(){
  if(!has_test_proj_)
    throw TrainingError("cannot clear_test_proj: data is not set\n");
  test_proj_.reallocate(0,0);
  has_test_proj_ = false;
}

/************************************************************
  check_inputs()
************************************************************/
void LLE::check_inputs()
{
  //check inputs to make sure they make sense
  if (k_ <= 0)
    throw TrainingError("LLE Error: k must be positive\n");
  
  if (Ntrain_ <= 0)
    throw TrainingError("LLE Error: no training data supplied\n");

  if (k_ >= Ntrain_)
    throw TrainingError("LLE Error: k must be less than "
			"number of training points\n");
  
  if(learn_d_out_)
    {
      if ((var_<0) || (var_>1.0))
	throw TrainingError("LLE Error: variance must be in range "
			    "0.0 < var < 1.0\n");  
    }
  else
    {
      if (d_out_ <= 0)
	throw TrainingError("LLE Error: d_out must be positive\n");
      
      if (k_ < d_out_)
	throw TrainingError("LLE Error: k must be greater than "
			    "or equal to output dimension");
      
      if (d_in_ <= d_out_)
	throw TrainingError("LLE Error: output dimension must be "
			    "less than input dimension");
    }
}


/************************************************************
  cmpnbrs_()
************************************************************/
void LLE::cmpnbrs_(bool train){
  Matrix<double>* training_data;
  Matrix<double>* test_data;
  Matrix<int>* neighbors;
  //let training_data, test_data, and neighbors 
  //  point to the appropriate data
  training_data = &training_data_;
  if(has_test_data_ && !train){
    test_data = &test_data_;
    neighbors = &test_nbrs_;
  }
  else{
    test_data = &training_data_;
    neighbors = &training_nbrs_;
  }
  
  //if test data and training data are the same, then the nearest
  // neighbor of a point is itself.  Use a variable starting_k to
  // correct for this
  int starting_k = (test_data==training_data) ? 1 : 0;
  int Ntest = test_data->ncols();
  int Ntrain = training_data->ncols();

  //now loop through and create neighbors
  neighbors->reallocate(k_,Ntest);
  
  Matrix<double> data_i(d_in_,Ntrain);
  Vector<double> D(Ntrain);
  Vector<int> indices(Ntrain);
  
  //temporary vectors for arithemetic
  Vector<double> colj;
  Vector<double> test_coli;
  Vector<double> train_colj;
  Vector<int> ncoli;
  Vector<int> iSV;
  
  for(int i=0;i<Ntest;i++)
    {
      //find k nearest neighbors of test_data[i] within training_data
      for (size_t j=0;j<Ntrain;j++){
	data_i.col(j) = test_data->col(i) - training_data->col(j);
	D(j) = blas_NRM2( data_i.col(j) );
      }
      // find k nearest points
      
      argsort(D,indices);
      
      neighbors->col(i) = indices.SubVector(starting_k,starting_k+k_);
    }
}


/************************************************************
  cmpweight_(bool train)
************************************************************/
void LLE::cmpweight_(bool train)
{
  Matrix<double>* training_data;
  Matrix<double>* test_data;
  Matrix<int>* neighbors;
  
  //let training_data and test_data point to the appropriate data
  training_data = &training_data_;
  
  if(has_test_data_ && !train){
    test_data = &test_data_;
    neighbors = &test_nbrs_;
  }
  else{
    test_data = &training_data_;
    neighbors = &training_nbrs_;
  }

  int Ntest = test_data->ncols();
  int Ntrain = training_data->ncols();
  
  if(!quiet_) 
    std::cout << " - Constructing [" 
	      << Ntest << " x " << Ntrain 
	      << "] weight matrix.\n";

  weight_matrix_.reallocate(Ntest,Ntrain,0.0);

  Matrix<double> neighborhood(d_in_,k_);
  Matrix<double,SYM> Q(k_,k_);
  Vector<double> w(k_);
  
  for (size_t i=0;i<Ntest;i++)
    {
      for(int j=0;j<k_;j++){
	neighborhood.col(j) =
	  training_data->col( (*neighbors)(j,i) ) - test_data->col(i);
      }
      
      //Construct the [k x k] covariance matrix 
      //   of the neighborhood
      Q = neighborhood.Transpose() * neighborhood;
      
      //add a fraction of the trace to the diagonal
      // this prevents matrix from being singular
      //Q += 0.001 * float(Q.Trace());
      Q.diag() += 0.001*Q.Trace();
      
      //solve for w in Q*w = [1,1,1...1]^T
      //  and put into weight matrix
      w.SetAllTo(1.0);
      
      SOLVE(Q,w);
      w /= w.SumElements();
      
      for(int j=0;j<k_;j++)
	weight_matrix_(i,(*neighbors)(j,i)) = w(j);
    }
}
  
/************************************************************
  compute_training_nbrs()
************************************************************/
void LLE::compute_training_nbrs(){
  if(training_nbrs_.nrows() >= k_)
    {
      //already computed enough neighbors
      if(!quiet_) 
	std::cout << " - using previously computed nearest neighbors\n";
      return;
    }
  else
    {
      if(!quiet_) 
	std::cout << " - finding " << k_ << " nearest neighbors "
		  << "of each training point.\n";
      cmpnbrs_(true); //true means we're doing only training data
    }
}

/************************************************************
  compute_test_nbrs()
************************************************************/
void LLE::compute_test_nbrs(){
  if(!has_test_data_)
    {
      compute_training_nbrs();
    }
  else if(test_nbrs_.nrows() >= k_)
    {
      //already computed enough neighbors
      return;
    }
  else
    {
      if(!quiet_)
      {
      std::cout << " - finding " << k_ << " nearest neighbors "
		<< "of each test point.\n";
      }
      cmpnbrs_(false); //false means compute neighbors of test data
    }
}

/************************************************************
  compute_training_weights()
************************************************************/
void LLE::compute_training_weights(){
  cmpweight_(true);
}

/************************************************************
  compute_test_weights()
************************************************************/
void LLE::compute_test_weights(){
  if(has_test_data_)
    cmpweight_(false);
  else
    throw TrainingError("Cannot compute_test_weights() : no "
			"test data specified\n");
}

  
/************************************************************
  compute_sigma()
************************************************************/
void LLE::compute_sigma()
{
  if (!quiet_)
    std::cout << "LLE::compute_sigma():\n";
  if(has_sigma_){
    if (!quiet_)
      std::cout << " - using previously computed values of sigma\n";
  }else{
    compute_d_out();
    check_inputs();
    
    
    if (!quiet_)
      std::cout << " - Computing reconstruction errors of " 
		<< Ntrain_ <<" points in " << d_in_ << " dimensions.\n";
    
    compute_training_nbrs();
    
    Matrix<double> neighborhood(d_in_,k_);
    sigma_.reallocate(Ntrain_);
    
    //------------------------------------------------
    // Compute optimal reconstruction of each point from its neighborhood
    for (size_t i=0;i<Ntrain_;i++)
      {
	for(int j=0;j<k_;j++){
	  neighborhood.col(j) = 
	    training_data_.col( training_nbrs_(j,i) ) - training_data_.col(i);
	}
	
	//find singular values of neighborhood
	SVD nbrs_SVD(neighborhood,false);
	
	//find reconstruction error based on unused variance
	for(int j=d_out_;j<std::min(d_in_,k_);j++)
	  sigma_(i) += nbrs_SVD.S(j) * nbrs_SVD.S(j);
      }
    has_sigma_ = true;
  }
}



/************************************************************
  clear_sigma()
************************************************************/
void LLE::clear_sigma(){
  if(!has_sigma_)
    throw TrainingError("cannot clear_sigma : sigma has not been computed\n");
  sigma_.reallocate(0);
  has_sigma_ = false;
}


/************************************************************
  compute_d_out()
************************************************************/
int LLE::compute_d_out()
{
  double var_total = 0.0;
  int N = Ntrain_;

  if(learn_d_out_){
    check_inputs();
    
    if (!quiet_)
      std::cout << "LLE::compute_d_out():\n"
		<< " - Computing dimensionality of " << N <<" points in " 
		<< d_in_ << " dimensions\n"
		<< "    using " << k_ << " neighbors, with var = " 
		<< var_ << "\n";
    
    //------------------------------------------------
    // Compute training neighbors, if needed
    compute_training_nbrs();
    
    //------------------------------------------------
    // Find dimensionality at each neighborhood
    Matrix<double> neighborhood(d_in_,k_);
    Vector<int> dim_array(N);
    
    Vector<double> colj;
    Vector<double> col_nji;
    Vector<double> coli;
    
    for (size_t i=0;i<N;i++)
      {
	for(int j=0;j<k_;j++){
	  neighborhood.col(j) = 
	    training_data_.col(training_nbrs_(j,i) ) - training_data_.col(i);
	}
	
	//find singular values of neighborhood
	// determine how many are needed to sum to var
	SVD nbrs_SVD(neighborhood,false);
	
	for(int j=0;j<nbrs_SVD.S.size();j++)
	  nbrs_SVD.S(j) *= nbrs_SVD.S(j);
	
	nbrs_SVD.S /= nbrs_SVD.S.SumElements();
	
	//find number of dimensions of nbrs_SVD.S needed to add to var
	dim_array(i)=d_in_;
	int dim;
	for (dim=1;dim<d_in_;dim++){
	  if (nbrs_SVD.S(dim-1)>var_)
	    { 
	      dim_array(i)=dim;
	      break;
	    }
	  nbrs_SVD.S(dim) += nbrs_SVD.S(dim-1);
	}
	var_total += nbrs_SVD.S(dim-1);
      }
    
    var_total /= N;
    
    //find mean and standard deviation of dim_array
    double d = 0.0;
    double d2 = 0.0;
    for(int i=0;i<N;i++){
      d += dim_array(i);
      d2 += pow( dim_array(i),2 );
    }
    d /= N;
    d2 /= N;
    float sig = sqrt(d2-d*d);
    
    //find d_out.  Round down if it will stay within sig of d
    //  round up otherwise
    if ( (d<floor(d)+0.5) && (int(floor(d)) == int(ceil(d-sig))) )
      d_out_ = int(floor(d));
    else
      d_out_ = int(ceil(d));
    
    if (!quiet_){
      printf (" - Intrinsic dimensionality = %i (%.2f +/- %.2f)\n",
	      d_out_,d,sig);
      printf ("    for a variance of %.2f (avg %.2f conserved)\n",
	      var_,var_total);
    }
    learn_d_out_ = false;
  }

  return d_out_;
}

  
/************************************************************
  cmp_training_proj_()
************************************************************/
void LLE::cmp_training_proj_(){
  if (!quiet_)
    std::cout << "LLE::compute_training_projection()\n"
	      << " - Performing LLE on " << Ntrain_ <<" points in " 
	      << d_in_ << "->"<< d_out_ << " dimensions.\n";
  
  //-------------------------------------------------
  // Find k nearest neighbors to each point
  compute_training_nbrs();
  
  //-------------------------------------------------
  // Find weight matrix
  compute_training_weights();
  
  Matrix<double>& W=weight_matrix_;
  
  //------------------------------------------------
  //projection is the null space of (W-I)
  //  by the Rayleigh-Ritz theorem, this is given by the
  //   eigenvectors corresponding to the d+1 smallest
  //   eigenvalues of (W-I)^T * (W-I)
  
  //let W = (W-I)
  Vector<double>Wdiag = W.diag();
  Wdiag -= 1.0;
  
  Matrix<double,SYM> Cov = W.Transpose() * W;
  
  //clear the memory: W is no longer needed
  W.reallocate(0,0);
  
  if (!quiet_)
    std::cout << " - Finding null space of weight matrix with ARPACK\n";
  //now find eigenvectors of W...

  
  if (!quiet_)
    std::cout << "   + ";
  EIGS_AR C_EIGS(Cov,d_out_,"SM",1);

  if(!quiet_) std::cout << "copying training proj\n";

  //copy values to training_proj_
  training_proj_.reallocate(d_out_,Ntrain_);
  training_proj_ = C_EIGS.evecs.Transpose();
  
  if (!quiet_){
    std::cout << " - Success!!\n";
  }

  has_training_proj_ = true;
}

  
/************************************************************
  cmp_test_proj_()
************************************************************/
void LLE::cmp_test_proj_(){
  if (!quiet_)
    std::cout << "LLE::compute_test_projection()\n"
	      << " - Projecting " << Ntest_ <<" points in " 
	      << d_in_ << "->"<< d_out_ << " dimensions.\n";
  
  compute_projection();
  test_proj_.reallocate(d_out_,Ntest_);
    
  //-------------------------------------------------
  // Find k nearest neighbors to each test point
  compute_test_nbrs();
    
  //-------------------------------------------------
  // Find weight matrix
  compute_test_weights();
    
  Matrix<double>& W = weight_matrix_;

  //------------------------------------------------
  // Multiply weights by projections to get new projection
  if(!quiet_) 
    std::cout << " - Multiplying weights by training projection\n";

  test_proj_ = training_proj_ * W.Transpose();

  //clear the memory
  weight_matrix_.reallocate(0,0);

  has_test_proj_ = true;
}


/************************************************************
  whiten()
************************************************************/
void LLE::whiten()
{
  //SVD gives data = U * S * V^T
  
  if(!quiet_)
    std::cout << "LLE::whiten()\n"
	      << " - centering data\n";
  
  //center matrix around data mean
  for(int i=0;i<d_in_;i++){
    double S = training_data_.row(i).SumElements() / Ntrain_;
    for(int j=0;j<Ntrain_;j++)
      training_data_(i,j) -= S;
  }
  
  if(!quiet_)
    std::cout << " - whitening data with an svd\n";

  SVD data_SVD(training_data_,true,true); // overwrite data with SVD

  training_data_ *= sqrt(Ntrain_-1);
}


/************************************************************
  ID()
************************************************************/
std::string LLE::ID(){
  std::ostringstream oss;
  
  oss << "LLE" << "K" << k_ << "D" << d_out_;
  return oss.str();
}

/************************************************************
  update_[training/test]_file()
************************************************************/
void LLE::update_training_file()
{
  if(has_training_proj_)
    training_file_.append_image(training_proj_,ID());
  if(has_sigma_)
    training_file_.append_image(sigma_,sigma_ID());
  if(training_nbrs_.ncols() >= 0)
    training_file_.append_image(training_nbrs_,neighbors_ID());
}

void LLE::update_test_file()
{
  if(has_test_proj_)
    test_file_.append_image(test_proj_,"proj"+ID());
  else
    throw TrainingError("update_test_file() : test_proj "
			"has not yet been computed\n");
}

/************************************************************
  dump()
************************************************************/
void LLE::dump(std::ostream& o){
  o << "LLE object:\n"
    << "  k = " << k_ << "\n"
    << "  d_in = " << d_in_ << "\n"
    << "  d_out = " << d_out_ << "\n"
    << "  var = " << var_ << "\n"
    << "  Ntrain = " << Ntrain_ << "\n"
    << "  Ntest = " << Ntest_ << "\n";
}
  

void LLE::compute_training_projection(){
  if(!has_training_proj_){
    compute_d_out();
    check_inputs();
    cmp_training_proj_();
    has_training_proj_ = true;
  }else{
    if (!quiet_)
      std::cout << "LLE::compute_training_projection()\n"
		<< " - Using previously computed projection.\n";
  }
}

void LLE::compute_projection(const Matrix<double>& test_data){
  if ( (&test_data) != (&test_data_) )
    set_test_data(test_data);
  compute_test_projection();
}

void LLE::set_hessian(bool hessian){
  std::cerr << "set_hessian() is undefined\n";
}

void LLE::set_r(double r){
  std::cerr << "set_r() is undefined\n";
}
