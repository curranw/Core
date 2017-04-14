#ifndef RLLE_H
#define RLLE_H
#include "LLE.h"
#include "MatVec.h"
#include <math.h>
#include <string>


/***************************************************
    RLLE class to compute the d_out - dimensional 
     Robust Locally Linear Embedding of a matrix 
     using the k nearest neighbors

    Two different algorithms are available, both based on
     algorithms outlined in
     'Robust Locally Linear Embedding'
      by Hong Chang & Dit-Yan Yeung, 2005
****************************************************/
class RLLE_ : public LLE{
 protected:
  Vector<double> r_scores_;
  bool has_r_scores_;
  void set_r_scores(const Vector<double>& r_scores);
  
 public:
  RLLE_() : LLE() , has_r_scores_(false){};
  RLLE_(const Matrix<double>& training_data) : 
    LLE(training_data) , has_r_scores_(false){}
  RLLE_(const std::string& filename) : 
    LLE(filename) , has_r_scores_(false){}
  
  /***************************************************
     compute_r_scores()
       computes reliability scores for the points in 
        training_data_
  ****************************************************/
  void compute_r_scores();
  void load_fits(const std::string& fits_file);
  void update_training_file();
  std::string r_scores_ID(){return "R"+LLE::ID()+"R";}

  void load_r_scores(const std::string& fits_file);
  void clear_r_scores();
};


class RLLE1 : public RLLE_{
 private:
  bool hessian_;
  double r_;

  void cmp_training_proj_();
 public:
  RLLE1() : 
    RLLE_() , r_(-1.) , hessian_(false){}
  RLLE1(const Matrix<double>& training_data) : 
    RLLE_(training_data) , r_(-1.) , hessian_(false){}
  RLLE1(const std::string& filename) : 
    RLLE_(filename) , r_(-1.) , hessian_(false){}

  void check_inputs();
  
  void set_hessian(bool hessian){hessian_=hessian;}
  void set_r(double r){r_=r;}

  std::string ID();
  
  virtual void dump(std::ostream& o = std::cout)
  {LLE::dump(o);o<<"  r = "<<r_<< "\n";}
};


class RLLE2 : public RLLE_{
 private:
  int r_;

  void cmp_training_proj_();
  void compute_weighted_neighbors();
 public:
  RLLE2() : RLLE_() , r_(-1){}
  RLLE2(const Matrix<double>& training_data) : RLLE_(training_data) , r_(-1){}
  RLLE2(const std::string& filename) : RLLE_(filename) , r_(-1){}

  void check_inputs();

  void set_r(double r){r_ = int(r);}

  std::string ID();
  
  virtual void dump(std::ostream& o = std::cout)
  {LLE::dump(o);o<<"  r = "<<r_<< "\n";}
};


#endif //RLLE_H
