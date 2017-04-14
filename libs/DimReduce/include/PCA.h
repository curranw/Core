#ifndef PCA_H
#define PCA_H

#include "MatVec.h"
#include "fitstools.h"
#include <string>
#include <sstream>

/************************************************************
 * PCA class to compute the principal component analysis 
 *  of a matrix
 ************************************************************/

enum PCA_TYPE{SVD_PCA = 0, 
	      EIG_PCA = 1, 
	      AREIG_PCA = 2};


/******************************************************
PCA

  perform a PCA on data

******************************************************/
class PCA{
protected:
  fitstools training_file_;
  
  Matrix<double> training_data_;
  Vector<double> training_data_mean_;
  Matrix<double> eigenvectors_;
  Vector<double> singular_values_;
  Matrix<double> training_proj_;

  Matrix<double> training_reconstruction_;
  
  int Ntrain_;  //number of training points
  int d_in_;  //input dimension (training and test)
  int d_out_; //output dimension

  double var_;//variance for learning d_out
  double total_variance_;
  
  bool learn_d_out_;
  bool quiet_;
  
  bool has_training_data_;
  bool has_eigs_;
  bool has_training_proj_;

  virtual void init();
  
  virtual void cmp_mean_Xc_(Matrix<double>& X_centered);
  void cmp_output_dim_(Matrix<double>& X_centered);
  void cmp_projection_SVD_(Matrix<double>& X_centered);
  void cmp_projection_EIG_(Matrix<double>& X_centered);
  void cmp_projection_AREIG_(Matrix<double>& X_centered);
  void cmp_proj_from_eigs_(Matrix<double>& X_centered);

public:
  PCA();

  PCA(const Matrix<double>& training_data);

  PCA(const std::string& filename);

  virtual void load_fits(const std::string& fits_file);
  virtual void update_training_file();
  
  void load_training_data(const std::string& fits_file);
  void load_training_proj(const std::string& fits_file);
  
  void set_training_data(const Matrix<double>& training_data);
  void set_training_proj(const Matrix<double>& training_proj);
  
  void clear_training_data();
  void clear_training_proj();
  
  bool has_training_data() const{return has_training_data_;}
  bool has_training_proj() const{return has_training_proj_;}
  
  void set_d_out(int d_out){d_out_ = d_out; learn_d_out_=false;}
  void set_var(double var){var_ = var; learn_d_out_=true;}
  void set_quiet(double quiet){quiet_ = quiet;}

  int d_out(){return d_out_;}
  double var(){return var_;}
  
  void check_inputs();

  virtual void compute(PCA_TYPE type = SVD_PCA);

  const Matrix<double>& reconstruct(int neigs = -1);
  
  std::string ID();
  std::string evecs_ID(){return ID() + "EVECS";}
  std::string mean_ID(){return ID() + "MEAN";}
  std::string weights_ID(){return ID() + "MEAN";}
  
  const Matrix<double>& training_projection() const{return training_proj_;}
  const Matrix<double>& eigenvectors() const{return eigenvectors_;}
  const Vector<double>& singular_values() const{return singular_values_;}
  const Vector<double>& training_data_mean() const{return training_data_mean_;}
  
};


/******************************************************
 * WPCA
 *  perform a weighted PCA on data
 ******************************************************/

class WPCA : public PCA{
protected:
  bool has_weights_;
  Vector<double> weights_;
  
  void init();
  void cmp_mean_Xc_(Matrix<double>& X_centered);

public:
  WPCA();
  WPCA(const Matrix<double>& training_data,
       const Vector<double>& weights);
  
  void load_fits(const std::string& fits_file);
  void update_training_file();
  
  void load_weights(const std::string& fits_file);
  void set_weights(const Vector<double>& training_weights);
  void clear_weights();
  bool has_weights() const{return has_weights_;}

  const Vector<double>& weights() const{return weights_;}

  std::string weights_ID(){return ID() + "WEIGHTS";}
};


/******************************************************
IRWPCA - Iteratively Reweighted PCA

 based on algorithm outlined in
     'Robust Locally Linear Embedding'
      by Hong Chang & Dit-Yan Yeung, 2005

******************************************************/
class IRWPCA : public WPCA{
private:
  int max_iter_;
  double tol_;

public:
  void load_fits(const std::string& fits_file);
  explicit IRWPCA(int max_iter=1000,
		  double tol=1E-8);
  void compute(PCA_TYPE type = SVD_PCA);

  int max_iter() const{return max_iter_;}
  double tol() const{return tol_;}
  void set_max_iter(int max_iter){max_iter_ = max_iter;}
  void set_tol(double tol){tol_ = tol;}
};

/***********************************************************
  IterException:
    a custom exception class for when IRWPCA reaches
    maximum iterations
************************************************************/
class IterException: public std::exception
{
public:
  IterException(const std::string& s = "IRWPCA: Max iterations reached."){msg=s;}
  IterException(const int MAXITER, const double tol=0.0)
    {
      std::stringstream s;
      s << "IRWPCA: Max iterations (" << MAXITER << ") reached";
      if(tol>0)
	s<<" before reaching tol = " << tol;
      s << std::endl;
      msg = s.str();
    }
  ~IterException() throw() {}
  virtual const char* what() const throw(){return msg.c_str();}
private:
  std::string msg;
};


#endif /* PCA_H */
