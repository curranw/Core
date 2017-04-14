#ifndef DATA_H
#define DATA_H
#include <string>
#include "MatVec.h"

/*******************************************************
WriteMatrixFile
  writes a matrix to a file
*******************************************************/
void WriteMatrixFile(const Matrix<double>& data,
		     const std::string filename,
		     const std::string comment="",
		     bool quiet=false);

/*******************************************************
ReadMatrixFile
  reads the output of WriteFile
*******************************************************/
void ReadMatrixFile(Matrix<double>& data,
		    const std::string filename,
		    bool quiet=false);

/*******************************************************
WriteVectorFile
  writes a matrix to a file
*******************************************************/
void WriteVectorFile(const Vector<double>& data,
		     const std::string filename,
		     const std::string comment="",
		     bool quiet=false);

/*******************************************************
ReadVectorFile
  reads the output of WriteFile
*******************************************************/
void ReadVectorFile(Vector<double>& data,
		    const std::string filename,
		    bool quiet=false);

#endif //DATA_H
