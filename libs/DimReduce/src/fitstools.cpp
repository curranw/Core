#include "fitstools.h"
#include "DimReduceExcept.h"
#include <iostream>
#include <sstream>

/*******************************************************
  split_fits_title
*******************************************************/

bool split_fits_title(std::string& filename){
  std::string label;
  return split_fits_title(filename,label);
}

bool split_fits_title(std::string& filename,
              std::string& label){
  int start;
  int end = filename.size()-1;
  while(filename[end]==' ')
    end--;
  if(filename[end]==']'){
    for(start=end-1;start>=0;start--)
      if(filename[start] == '[')
    break;
    if(start==0)
      throw FileReadError("invalid fits file format",filename);

    label = filename.substr(start+1,end-start-1);
    filename = filename.substr(0,start);
    return true;
  }else{
    label="";
    return false;
  }
}


/*******************************************************
  combine_fits_title
*******************************************************/
void combine_fits_title(std::string& filename,
            const std::string& label){
  filename += ('['+label+']');
}

void combine_fits_title(std::string& filename,
            int hdunum){
  std::ostringstream oss;
  oss << '+' << hdunum;
  filename += oss.str();
}

/*******************************************************
  init()
*******************************************************/
void fitstools::init(){
  filename_ = "";
  fileopen_ = false;
  quiet_ = false;
  fptr_ = 0;
  status_ = 0;
  numhdus_ = 0;
  extnames_.clear();
}

/*******************************************************
  hdunum
*******************************************************/
int fitstools::hdunum(const std::string& extname) const{
  for(int i=0;i<numhdus_;i++){
    if(extname==extnames_[i])
      return i+1;
  }
  return -1;
}

bool fitstools::ext_in_file(const std::string& extname) const{
  return hdunum(extname)>0;
}

int fitstools::current_hdunum() const{
  int num;
  return fits_get_hdu_num(fptr_,&num);
}

int fitstools::current_hdutype() const{
  int T;
  int status = 0;
  if( fits_get_hdu_type(fptr_,&T,&status) ){
    std::cout << "error here\n";
    throw FITSerror();
  }
  return T;
}

std::string fitstools::current_extname() const{
  return extnames_[current_hdunum()-1];
}

/*******************************************************
  update_*
*******************************************************/
void fitstools::update_extnames(){
  int this_hdu = current_hdunum();

  extnames_.resize(numhdus_);
  char value[100];
  int hdutype;
  for(int i=1;i<=numhdus_;i++){
    if ( fits_movabs_hdu(fptr_, i, &hdutype, &status_) )
      throw FITSerror();

    if( fits_read_key(fptr_,TSTRING,"EXTNAME",
              value,NULL,&status_) ){
      if( status_ == KEY_NO_EXIST ){
    if(i==1)
      extnames_[i-1] = "PRIMARY";
    else if(i==2)
      extnames_[i-1] = "INFO";
    else
      extnames_[i-1] = "";
    status_ = 0;
      }else{
    throw FITSerror();
      }
    }else{
      extnames_[i-1] = value;
    }
  }

  /* move back to original hdunum */
  if ( fits_movabs_hdu(fptr_, this_hdu, &hdutype, &status_) )
    throw FITSerror();
}

/*******************************************************
  read_key
*******************************************************/
template<>
const std::string& fitstools::read_key(const std::string& keyname,
                       std::string& keyval) const{
  char value[80];
  if( fits_read_key(fptr_,TSTRING,
            const_cast<char*>(keyname.c_str()),
            value,NULL,&status_) ){
    throw FITSerror();
  }
  keyval = value;
  return keyval;
}


/*******************************************************
  write_key
*******************************************************/
template<>
void fitstools::write_key(const std::string& keyname,
              const std::string& keyval,
              const std::string& comment/* = ""*/){
  if( fits_write_key(fptr_,TSTRING,
             const_cast<char*>(keyname.c_str()),
             const_cast<char*>(keyval.c_str()),
             const_cast<char*>(comment.c_str()),
             &status_) ){
    throw FITSerror();
  }
}

/*******************************************************
  open_file
*******************************************************/
void fitstools::open_file(const std::string& filename){
  if(fileopen_)
    throw FITSerror("open_file(): cannot open file: file is already open.\n");

  filename_ = filename;

  /* open fits file */
  if ( fits_open_file(&fptr_, filename_.c_str(),
              READWRITE, &status_) )
    throw FITSerror();

  split_fits_title(filename_);

  /* get number of hdus */
  if( fits_get_num_hdus(fptr_,&numhdus_,&status_) )
    throw FITSerror();

  update_extnames();

  fileopen_ = true;
}

/*******************************************************
  close_file
*******************************************************/
void fitstools::close_file(){
  if(!fileopen_)
    throw FITSerror("Cannot close file: file is not open");

  if ( fits_close_file(fptr_, &status_) )
    throw FITSerror();

  fileopen_ = false;
}

/*******************************************************
  is_valid_fits
*******************************************************/

bool fitstools::is_valid_fits() const{
  /* make sure HDU[1] is a valid 2D image, and
     find number of data points */
  int status;

  if ( fits_movabs_hdu(fptr_, 1, 0, &status) )
    throw FITSerror();

  /* get the number of dimensions in the array  */
  int N_primary;
  int naxis;

  //check status of HDU[N>=2]
  char value[80];
  for(int i=1;i<=numhdus_;i++){
    if ( fits_movabs_hdu(fptr_, i, 0, &status) )
      throw FITSerror();

    //must have a valid EXTNAME
    if( fits_read_key(fptr_,TSTRING,"EXTNAME",
              value,NULL,&status) ){
      if( status == KEY_NO_EXIST ){
    if(i>2)
      return false;//i=2 does not need EXTNAME
    status = 0;
      }
      else
    throw FITSerror();
    }

    naxis = current_hdu_dimension();

    if(i==1){
      if(naxis!=2) //first HDU must be the data matrix
    return false;
      //second dimension is N_primary
      read_key("NAXIS2",N_primary);
      continue;
    }

    /*make sure size matches data size
    // note: removed this because PCA eigs may not be
    //       the same size as the data dimension

    int nvals;

    if(naxis==1){ // dimension is 1 - let naxis be the length of the array
      if( fits_read_key(fptr_,TINT,"NAXIS1",&nvals,NULL,&status) )
    throw FITSerror();
    }
    else if(naxis==2){
      // dimension is 2 - let naxis be the length of the second dimension
      if( fits_read_key(fptr_,TINT,"NAXIS2",&nvals,NULL,&status) )
    throw FITSerror();
    }
    else if(nvals != N_primary)
      return false;
    */
  }
  return true;
}

/*******************************************************
 move_to_hdu
*******************************************************/
void fitstools::move_to_hdu(int hdunum){
  if(hdunum<1 || hdunum>numhdus_)
    throw FITSerror("move_to_hdu: hdunum out of range");
  if ( fits_movabs_hdu(fptr_, hdunum, NULL, &status_) )
    throw FITSerror();
}

void fitstools::move_to_hdu(const std::string& extname){
  if(extname == "PRIMARY")
    move_to_hdu(1);
  else if(extname == "INFO")
    move_to_hdu(2);
  else{
    if(!ext_in_file(extname))
      throw FITSerror("move_to_hdu: extname not in file");
    if( fits_movnam_hdu(fptr_,ANY_HDU,const_cast<char*>(extname.c_str()),
            0,&status_) )
      throw FITSerror();
  }
}


/*******************************************************
read_image
  reads matrix from an image hdu of a fits file
*******************************************************/
void fitstools::read_image(Matrix<double>& data){
  if(!quiet_)
    std::cout << "fitstools::read_image():\n"
          << " - reading image " << filename_
          << "[" << current_extname() << "]\n";

  /* get the number of dimensions in the array  */
  int naxis = current_hdu_dimension();

  /* get the size of the array dimensions */
  long* naxes = new long[naxis];
  if ( fits_get_img_size(fptr_, naxis, naxes, &status_) )
    throw FITSerror();

  /* construct the matrix */
  int D;
  int N;
  if(naxis==2){
    D = naxes[0];
    N = naxes[1];
  }
  else if(naxis==1){
    D = 1;
    N = naxes[0];
  }
  else
    throw FITSerror("fitstools::read_image(): image must have 1 "
            "or 2 dimensions\n");

  delete [] naxes;

  if(!quiet_)
    std::cout << "    > [" << D << " x " << N
          << "] matrix." << std::endl;
  data.reallocate(D,N,0.0);

  long npixels  = D*N; /* number of pixels in the image */
  float nullval  = 0;  /* don't check for null values in the image */
  int anynull;

  if ( fits_read_img(fptr_, TDOUBLE, 1, npixels, &nullval,
             data.arr(), &anynull, &status_) )
    throw FITSerror();
}

void fitstools::read_image(Vector<double>& data){
  Matrix<double> M;
  read_image(M);
  if(M.nrows() != 1)
    throw FITSerror("read_image(): extension is not a vector!");

  data.viewof( M.row(0) );
  data.set_mem_allocated(true);
  M.set_mem_allocated(false);
}


void fitstools::read_image(Matrix<int>& data){
  if(!quiet_)
    std::cout << "fitstools::read_image():\n"
          << " - reading image " << filename_
          << "[" << current_extname() << "]\n";

  /* get the number of dimensions in the array  */
  int naxis = current_hdu_dimension();

  /* get the size of the array dimensions */
  long* naxes = new long[naxis];
  if ( fits_get_img_size(fptr_, naxis, naxes, &status_) )
    throw FITSerror();

  /* construct the matrix */
  int D;
  int N;
  if(naxis==2){
    D = naxes[0];
    N = naxes[1];
  }
  else if(naxis==1){
    D = 1;
    N = naxes[0];
  }
  else
    throw FITSerror("fitstools::read_image(): image must have 1 "
            "or 2 dimensions\n");

  delete [] naxes;

  if(!quiet_)
    std::cout << "    > [" << D << " x " << N
          << "] matrix." << std::endl;
  data.reallocate(D,N,0);

  long npixels  = D*N; /* number of pixels in the image */
  float nullval  = 0;  /* don't check for null values in the image */
  int anynull;

  if ( fits_read_img(fptr_, TINT, 1, npixels, &nullval,
             data.arr(), &anynull, &status_) )
    throw FITSerror();
}

void fitstools::read_image(Vector<int>& data){
  Matrix<int> M;
  read_image(M);
  if(M.nrows() != 1)
    throw FITSerror("read_image(): extension is not a vector!");

  data.viewof( M.row(0) );
  data.set_mem_allocated(true);
  M.set_mem_allocated(false);
}

/*******************************************************
 read_column
  reads vector from a table hdu of a fits file
*******************************************************/
void fitstools::read_column(const std::string& colname,
         Vector<double>& data){
  if(!quiet_)
    std::cout << "read_column():\n"
          << " - reading column \'" << colname
          << "\' from " << filename_
          << "[" << current_extname() << "]\n";


  /* determine colnum of the key */
  int colnum;
  if( fits_get_colnum(fptr_,CASEINSEN,const_cast<char*>(colname.c_str()),
              &colnum,&status_) )
    throw FITSerror();

  /* determine number of rows */
  long nrows;
  if( fits_get_num_rows(fptr_,&nrows,&status_) )
    throw FITSerror();

  data.reallocate(nrows,0.0);

  /*  read the columns */
  long frow      = 1;
  long felem     = 1;
  char* strnull = " ";
  int anynull;
  if( fits_read_col(fptr_, TDOUBLE, colnum, frow, felem, nrows,
            strnull, data.arr(), &anynull, &status_) )
    throw FITSerror();
}

void fitstools::read_column(const std::string& colname,
         Vector<int>& data){
  if(!quiet_)
    std::cout << "read_column():\n"
          << " - reading column \'" << colname
          << "\' from " << filename_
          << "[" << current_extname() << "]\n";


  /* determine colnum of the key */
  int colnum;
  if( fits_get_colnum(fptr_,CASEINSEN,const_cast<char*>(colname.c_str()),
              &colnum,&status_) )
    throw FITSerror();

  /* determine number of rows */
  long nrows;
  if( fits_get_num_rows(fptr_,&nrows,&status_) )
    throw FITSerror();

  data.reallocate(nrows,0);

  /*  read the columns */
  long frow      = 1;
  long felem     = 1;
  char* strnull = " ";
  int anynull;
  if( fits_read_col(fptr_, TINT, colnum, frow, felem, nrows,
            strnull, data.arr(), &anynull, &status_) )
    throw FITSerror();
}

/*******************************************************
append_image
  appends matrix to an existing fits file as a new HDU
*******************************************************/
void fitstools::append_image(const Matrix<double>& data,
                 const std::string& extname){
  if(!quiet_)
    std::cout << "fitstools::append_image():\n"
          << " - Appending [" << data.nrows() << " x " << data.ncols()
          << "] matrix to file: " << filename_ << "["
          << extname << "]\n";

  /* delete old fit with same EXTNAME, if it exists */
  std::string this_ext = current_extname();

  if( ext_in_file(extname) ){
    std::cout << "    > Deleting old fit\n";
    move_to_hdu(extname);
    delete_current_hdu();
    std::cout << "    > Success\n";
  }

  /* append data to end of HDU */
  int bitpix = -64; //data is 64-bit double
  long naxes[] = {data.nrows(),data.ncols()};
  if( fits_create_img(fptr_, bitpix, 2, naxes, &status_) )
    throw FITSerror();

  numhdus_++;
  move_to_hdu(numhdus_);

  /* write data from matrix */
  long fpixel[] = {1,1}; //starting pixel values
  long nelements = data.nrows() * data.ncols();
  const double* dataptr = data.arr();

  if( fits_write_pix(fptr_,TDOUBLE,fpixel,nelements,
             const_cast<double*>(dataptr),&status_) )
    throw FITSerror();

  /* write dataname key */
  write_key("EXTNAME",extname,"type of transform");

  //need to close and reopen, to update the fits pointer
  close_file();
  open_file(filename_);
  move_to_hdu(this_ext);
}


void fitstools::append_image(const Vector<double>& data,
          const std::string& extname){
  if(!quiet_)
    std::cout << "fitstools::append_image():\n"
          << " - Appending rank-" << data.size()
          << " vector to file: " << filename_ << "["
          << extname << "]\n";

  /* delete old fit with same EXTNAME, if it exists */
  std::string this_ext = current_extname();
  if( ext_in_file(extname) ){
    std::cout << "    > Deleting old fit\n";
    move_to_hdu(extname);
    delete_current_hdu();
  }

  /* append data to end of HDU */
  int bitpix = -64; //data is 64-bit double
  long naxes = data.size();
  if( fits_create_img(fptr_, bitpix, 1, &naxes, &status_) )
    throw FITSerror();

  numhdus_++;

  if ( fits_movabs_hdu(fptr_, numhdus_, 0, &status_) )
    throw FITSerror();

  /* write data from matrix */
  long fpixel[] = {1,1};
  long nelements = data.size();
  const double* dataptr = data.arr();

  if( fits_write_pix(fptr_,TDOUBLE,fpixel,nelements,
             const_cast<double*>(dataptr),&status_) )
    throw FITSerror();


  /* write dataname key */
  if( fits_write_key(fptr_,TSTRING,"EXTNAME",
             const_cast<char*>(extname.c_str()),
             "type of transform",&status_) )
    throw FITSerror();

  //need to close and reopen, to update the fits pointer
  close_file();
  open_file(filename_);
  move_to_hdu(this_ext);
}


void fitstools::append_image(const Matrix<int>& data,
          const std::string& extname){
  if(!quiet_)
    std::cout << "fitstools::append_image():\n"
          << " - Appending [" << data.nrows() << " x " << data.ncols()
          << "] matrix to file: " << filename_ << "["
          << extname << "]\n";

  /* delete old fit with same EXTNAME, if it exists */
  std::string this_ext = current_extname();

  if( ext_in_file(extname) ){
    std::cout << "    > Deleting old fit\n";
    move_to_hdu(extname);
    delete_current_hdu();
  }

  /* append data to end of HDU */
  int bitpix = 32; //data is 32-bit int
  long naxes[] = {data.nrows(),data.ncols()};
  if( fits_create_img(fptr_, bitpix, 2, naxes, &status_) )
    throw FITSerror();

  numhdus_++;
  move_to_hdu(numhdus_);

  /* write data from matrix */
  long fpixel[] = {1,1};
  long nelements = data.nrows() * data.ncols();
  const int* dataptr = data.arr();

  if( fits_write_pix(fptr_,TINT,fpixel,nelements,
             const_cast<int*>(dataptr),&status_) )
    throw FITSerror();

  /* write dataname key */
  if( fits_write_key(fptr_,TSTRING,"EXTNAME",
             const_cast<char*>(extname.c_str()),
             "type of transform",&status_) )
    throw FITSerror();

  //need to close and reopen, to update the fits pointer
  close_file();
  open_file(filename_);
  move_to_hdu(this_ext);
}


void fitstools::append_image(const Vector<int>& data,
          const std::string& extname){
  if(!quiet_)
    std::cout << "fitstools::append_image():\n"
          << " - Appending rank-" << data.size()
          << " vector to file: " << filename_ << "["
          << extname << "]\n";

  /* delete old fit with same EXTNAME, if it exists */
  std::string this_ext = current_extname();
  if( ext_in_file(extname) ){
    std::cout << "    > Deleting old fit\n";
    move_to_hdu(extname);
    delete_current_hdu();
  }

  /* append data to end of HDU */
  int bitpix = 32; //data is 32-bit int
  long naxes = data.size();
  if( fits_create_img(fptr_, bitpix, 1, &naxes, &status_) )
    throw FITSerror();

  numhdus_++;

  if ( fits_movabs_hdu(fptr_, numhdus_, 0, &status_) )
    throw FITSerror();

  /* write data from matrix */
  long fpixel[] = {1,1};
  long nelements = data.size();
  const int* dataptr = data.arr();

  if( fits_write_pix(fptr_,TINT,fpixel,nelements,
             const_cast<int*>(dataptr),&status_) )
    throw FITSerror();


  /* write dataname key */
  if( fits_write_key(fptr_,TSTRING,"EXTNAME",
             const_cast<char*>(extname.c_str()),
             "type of transform",&status_) )
    throw FITSerror();

  //need to close and reopen, to update the fits pointer
  close_file();
  open_file(filename_);
  move_to_hdu(this_ext);
}


/*******************************************************
 delete_current_hdu
  delete the current HDU, and move to the first
*******************************************************/
void fitstools::delete_current_hdu(){
  int num = current_hdunum();
  if(num==1 || num==2)
    throw FITSerror("Cannot delete first or second HDU\n");
  if( fits_delete_hdu(fptr_,0,&status_) )
    throw FITSerror();
  close_file();
  open_file(filename_);
}

/*******************************************************
 current_hdu_dimension()
*******************************************************/
int fitstools::current_hdu_dimension() const{
  int naxis;
  int status=0;

  if( fits_get_img_dim(fptr_,&naxis,&status) )
    throw FITSerror();

  return naxis;
}

/*******************************************************
 current_hdu_info()
*******************************************************/
std::string fitstools::current_hdu_info() const{
  std::ostringstream oss;
  int status=0;
  oss << "HDU[" << current_hdunum() << "]: " << current_extname();
  int L = current_extname().size();
  for(int i=L;i<20;i++)
    oss<<' ';
  oss << "\t";

  int T = current_hdutype();
  if(T==IMAGE_HDU){
    int naxis;
    if(fits_get_img_dim(fptr_,&naxis,&status) ){
      throw FITSerror();
    }
    long* naxes = new long[naxis];
    if(fits_get_img_size(fptr_,naxis,naxes,&status) ){
      throw FITSerror();
    }
    oss << "(";
    for(int i=0;i<naxis;i++){
      oss << naxes[i];
      if(i<naxis-1)
    oss << " x ";
    }
    oss << ") image\n";
  }
  else{
    long nrows;
    int ncols;
    if(fits_get_num_rows(fptr_,&nrows,&status) )
      throw FITSerror();
    if(fits_get_num_cols(fptr_,&ncols,&status) )
      throw FITSerror();

    oss<< "(" << nrows << " x " << ncols << ") table\n";
  }
  return oss.str();
}
