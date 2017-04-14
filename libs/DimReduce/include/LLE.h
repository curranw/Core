#ifndef LLE_H
#define LLE_H
#include "MatVec.h"
#include "fitstools.h"
#include <string>


/***************************************************
  LLE class to compute the d_out - dimensional 
   Locally Linear Embedding of a matrix using 
   the k nearest neighbors

    Implementation based on the algorithm outlined in
     'An Introduction to Locally Linear Embedding'
        by L. Saul and S. Roweis, 2001

    Using imrovements suggested in
     'Locally Linear Embedding for Classification'
        by D. deRidder and R.P.W. Duin, 2002

    Created by Jake Vanderplas
     vanderplas@astro.washington.edu
     November 2008
****************************************************/
class LLE{
 protected:
  fitstools training_file_;
  fitstools test_file_;

  Matrix<double> training_data_;
  Matrix<double> training_proj_;
  Matrix<int> training_nbrs_;
  
  Matrix<double> test_data_;
  Matrix<double> test_proj_;
  Matrix<int> test_nbrs_;
  
  Matrix<double> weight_matrix_;
  Vector<double> sigma_;

  int Ntrain_;  //number of training points
  int Ntest_;  //number of test points
  int k_;     //number of nearest neighbors
  int d_in_;  //input dimension (training and test)
  int d_out_; //output dimension

  double var_;//variance for learning d_out
  
  bool recalc_;
  bool learn_d_out_;
  bool quiet_;
  
  bool has_training_data_;
  bool has_training_proj_;
  bool has_test_data_;
  bool has_test_proj_;
  bool has_sigma_;

  
  /************************************************************
    cmpnbrs_(bool train)
     helper function for computing nearest neighbors.

      if train==true, then determine the k nearest neighbors of
       the training sample within the training sample, and save
       them to training_nbrs_
      
      if train==false, then determine the k nearest neighbors of
       the test sample within the training sample, and save them
       to test_neighbors_
  ************************************************************/
  virtual void cmpnbrs_(bool train);

  /************************************************************
    cmpweight_(bool train)
     helper function for computing weight matrix.

      if train==true, then use training_nbrs to compute the 
       training weight matrix
      
      if train==false, then use test_nbrs to compute the 
       test weight matrix
  ************************************************************/
  virtual void cmpweight_(bool train);
  
  /************************************************************
    cmp_training_proj_()
     compute the projection of the training data
  ************************************************************/
  virtual void cmp_training_proj_();

  /************************************************************
    cmp_test_proj_()
     compute the projection of given test_data
  ************************************************************/
  void cmp_test_proj_();
  
  /************************************************************
    compute_training_nbrs()
     if neighbors are already computed, do nothing.
     if neighbors need to be determined, find the k nearest
      neighbors of each training point and save them 
      to training_nbrs_
  ************************************************************/
  void compute_training_nbrs();
  
  /************************************************************
    compute_test_nbrs()
     if neighbors are already computed, do nothing.
     if neighbors need to be determined, find the k nearest
      neighbors of each test point and save them to test_nbrs_
  ************************************************************/
  void compute_test_nbrs();
  
  /************************************************************
    compute_training_weights()
  ************************************************************/
  void compute_training_weights();
  
  /************************************************************
    compute_test_weights()
  ************************************************************/
  void compute_test_weights();

  /************************************************************
    init()
     initializes all class data
  ************************************************************/
  void init();
  
  /************************************************************
   *  set_training_data(const Matrix<double>& training_data)
   *  set_training_proj(const Matrix<double>& training_proj)
   *  set_test_data(const Matrix<double>& test_data)
   *  set_test_proj(const Matrix<double>& test_proj)
   ************************************************************/
  void set_training_data(const Matrix<double>& training_data);
  void set_training_proj(const Matrix<double>& training_proj);
  void set_test_data(const Matrix<double>& test_data);
  void set_test_proj(const Matrix<double>& test_proj);
  
 public:
  LLE();

  LLE(const Matrix<double>& training_data);

  LLE(const std::string& filename){init(); load_fits(filename);}

  /************************************************************
   *  load_fits(const std::string& fits_file)
   *    Loads training_data_ and possibly training_proj_
   *     Load primary HDU of specified file, and load the 
   *      projection given by suffix, if included.
   ************************************************************/
  virtual void load_fits(const std::string& fits_file);
  
  /************************************************************
   *  load_training_data(const std::string& fits_file)
   *  load_training_proj(const std::string& fits_file)
   *  load_test_data(const std::string& fits_file)
   *  load_test_proj(const std::string& fits_file)
   *  load_sigma(const std::string& fits_file)
   *
   *     Load the specified matrix from the given file
   *      fits_file is the path to the desired file, plus 
   *      a suffix giving the extension. 
   *      (see fitstools.h for more on fits files and extensions)
   ************************************************************/
  void load_training_data(const std::string& fits_file);
  void load_training_proj(const std::string& fits_file);
  void load_test_data(const std::string& fits_file);
  void load_test_proj(const std::string& fits_file);
  void load_sigma(const std::string& fits_file);

  
  /************************************************************
   *  clear_training_data(const std::string& fits_file)
   *  clear_training_proj(const std::string& fits_file)
   *  clear_test_data(const std::string& fits_file)
   *  clear_test_proj(const std::string& fits_file)
   *
   *     Clear the specified matrix. 
   *      (see fitstools.h for more on fits files and extensions)
   ************************************************************/
  void clear_training_data();
  void clear_training_proj();
  void clear_test_data();
  void clear_test_proj();

  
  bool has_training_data() const{return has_training_data_;}
  bool has_training_proj() const{return has_training_proj_;}
  bool has_test_data() const{return has_test_data_;}
  bool has_test_proj() const{return has_test_proj_;}
  bool has_sigma() const{return has_sigma_;}

  /************************************************************
     set_*()  :  Functions to set internal variables
   ************************************************************/
  void set_k(int k){k_=k;}
  void set_d_out(int d_out){d_out_ = d_out;}
  void set_var(double var){var_ = var; learn_d_out_=true;}
  void set_quiet(double quiet){quiet_ = quiet;}
  void set_recalc(double recalc){recalc_ = recalc;}
  
  //these are used when inherited by RLLE:
  virtual void set_hessian(bool);
  virtual void set_r(double);


  /************************************************************
   *  check_inputs()  :  throw exception if the inputs
   *                      are not good
   ************************************************************/
  virtual void check_inputs();

  
  /************************************************************
    compute_sigma()
     creates a vector of reconstruction errors.
     for each point, find the optimal linear reconstruction
     based on its k nearest neighbors, and determine the 
     reconstruction error based on this
  ************************************************************/
  void compute_sigma();
  void clear_sigma();
  
  /************************************************************
    compute_d_out()
     use the local variance of each neighborhood to compute
     d_out such that, the conserved variance v_i of each 
     neighborhood i satisfies 
        mean(v_i) >= var_
  ************************************************************/
  int compute_d_out();
  
  /************************************************************
    whiten()
     whiten the training matrix
  ************************************************************/
  void whiten();

  /************************************************************
    update_*()
     saves the projection to the original fits file
  ************************************************************/
  virtual void update_training_file();
  void update_test_file();

  /************************************************************
    ID()
     returns a unique ID string for the procedure, ie LLEK15D2
  ************************************************************/
  virtual std::string ID();
  std::string sigma_ID(){return ID()+"SIG";}
  std::string neighbors_ID(){return "NBRS";}

  /************************************************************
    dump()
  ************************************************************/
  virtual void dump(std::ostream& o = std::cout);

  /************************************************************
    public routines to compute projections
  ************************************************************/
  void compute_training_projection();
  void compute_projection(){compute_training_projection();}
  
  void compute_test_projection(){cmp_test_proj_();};
  void compute_projection(const Matrix<double>& test_data);

  /************************************************************
    routines to access test_proj_ and training_proj_
  ************************************************************/
  const Matrix<double>& test_projection() const{return test_proj_;}
  const Matrix<double>& training_projection() const{return training_proj_;}
};

#endif //LLE_H
