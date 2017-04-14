#ifndef FITSTOOLS_H
#define FITSTOOLS_H

#include "MatVec.h"
#include "fitsio.h"
#include <string>
#include <vector>

/*******************************************************
 split_fits_title()
  given filename = '/path/to/myfile.fits[LABEL]'
  on return, filename = "/path/to/myfile.fits"
             label = "LABEL"
  returns true if [LABEL] is specified, false otherwise
*******************************************************/
bool split_fits_title(std::string& filename);

bool split_fits_title(std::string& filename,
              std::string& label);

/*******************************************************
 combine_fits_title()
  opposite of split_fits_title()
*******************************************************/
void combine_fits_title(std::string& filename,
            const std::string& label);

void combine_fits_title(std::string& filename,
            int hdunum);



/*******************************************************
 cfitsio_datatag()
   given a value of a certain data type, this returns
    the correct cfitsio data tag.
*******************************************************/
inline int cfitsio_datatag(char*){return TSTRING;}
inline int cfitsio_datatag(const std::string&){return TSTRING;}
inline int cfitsio_datatag(int){return TINT;}
inline int cfitsio_datatag(float){return TFLOAT;}
inline int cfitsio_datatag(double){return TDOUBLE;}

/*******************************************************
 fitstools class
*******************************************************/
class fitstools{
 private:
  std::string filename_;
  bool fileopen_;

  bool quiet_;
  fitsfile* fptr_;
  int numhdus_;

  mutable int status_;

  std::vector<std::string> extnames_;

  void init();

  bool is_valid_fits() const;

  int hdunum(const std::string& extname) const;
  void update_num_hdus();
  void update_extnames();

  int current_hdutype() const;
  int current_hdunum() const;
  std::string current_extname() const;

 public:
  fitstools(){init();}
  fitstools(const std::string& filename){init(); open_file(filename);}
  ~fitstools(){if(fileopen_) close_file();}

  template<class T>
  void write_key(const std::string& keyname,
         const T& keyval,
         const std::string& comment = "");

  template<class T>
  const T& read_key(const std::string& keyname,
            T& keyval) const;

  void open_file(const std::string& filename);

  void close_file();

  void set_quiet(bool quiet){quiet_=quiet;}

  bool ext_in_file(const std::string& extname) const;

  void move_to_hdu(int hdunum);
  void move_to_hdu(const std::string& extname);

  void read_image(Matrix<double>& data);
  void read_image(Vector<double>& data);
  void read_image(Matrix<int>& data);
  void read_image(Vector<int>& data);

  void read_column(const std::string& colname,
           Vector<double>& data);
  void read_column(const std::string& colname,
           Vector<int>& data);

  void append_image(const Matrix<double>& data,
            const std::string& extname);
  void append_image(const Matrix<int>& data,
            const std::string& extname);
  void append_image(const Vector<double>& data,
            const std::string& extname);
  void append_image(const Vector<int>& data,
            const std::string& extname);

  void delete_current_hdu();

  int current_hdu_dimension() const;

  std::string current_hdu_info() const;

  int num_hdus(){return numhdus_;}

  const std::vector<std::string>& extnames() const{return extnames_;}

};



/*****************************************************
FITSerror
*****************************************************/
class FITSerror : public std::exception {
 public:
  FITSerror() throw(): exception()
    {
      msg_=new char[100];
      fits_read_errmsg(msg_);
    }
  FITSerror(std::string msg){msg_ = &msg[0];}
  ~FITSerror() throw(){}
  const char * what() const throw(){return msg_;}
 private:
  char* msg_;
};

/*******************************************************
  read_key
*******************************************************/
template<class T>
const T& fitstools::read_key(const std::string& keyname,
                 T& keyval) const{
  if( fits_read_key(fptr_,cfitsio_datatag(keyval),
            const_cast<char*>(keyname.c_str()),
            // xxx bdilday
            //            keyname.c_str(),
            &keyval,NULL,&status_) ){
    throw FITSerror();
  }
  return keyval;
}


/*******************************************************
  write_key
*******************************************************/
template<class T>
void fitstools::write_key(const std::string& keyname,
              const T& keyval,
              const std::string& comment/* = ""*/){
  if( fits_write_key(fptr_,cfitsio_datatag(keyval),
             const_cast<char*>(keyname.c_str()),
             const_cast<T*>(&keyval),
             const_cast<char*>(comment.c_str()),
             &status_) ){
    throw FITSerror();
  }
}

#endif /* FITSTOOLS_H */
