#include "Data.h"
#include "DimReduceExcept.h"
#include <fstream>
#include <iostream>
#include <string>

/********************************************************
 my_ifstream type
  - inherits all member functions of ifstream, but
  - operator>> is overloaded such that it will skip the 
     remainder of the line any time '#' is encountered.
********************************************************/
class my_ifstream : public std::ifstream{
public:
  my_ifstream() : std::ifstream(){}
  my_ifstream ( const char * filename, 
		std::ios_base::openmode mode = std::ios_base::in ) 
    : std::ifstream(filename,mode){}
};

template <class T>
my_ifstream& operator>>(my_ifstream& fin, T& x){
  char c;
  std::string s;
  std::streampos i = fin.tellg();

  std::ifstream* fin_ptr = &fin; // ifstream pointer to my_ifstream

  if( !(*fin_ptr>>c ).fail() ){
    if(c=='#'){ //skip the rest of the line
      std::getline((*fin_ptr), s, '\n');
      fin>>x; //recursively call this function
    }else{
      fin.seekg(i,std::ios_base::beg);
      (*fin_ptr)>>x; //call the ifstream version of operator>>
    }
  }
  return fin;
}

/*******************************************************
WriteMatrixFile
  writes a matrix to a file
*******************************************************/
void WriteMatrixFile(const Matrix<double>& data,
		     const std::string filename,
		     const std::string comment/*=""*/,
		     bool quiet/*=false*/)
{
  if(!quiet)
    std::cout << "WriteMatrixFile():\n"
	      << " - Writing [" << data.nrows() << " x " << data.ncols() 
	      << "] matrix to file: \'" << filename << "\'\n";

  std::ofstream fout(filename.c_str());
  
  //write comment string
  for(int i=0;i<comment.size();i++){
    if(i==0)
      fout << "#";
    fout << comment[i];
    if(comment[i]=='\n')
      fout<<"#";
    if(i+1==comment.size())
      fout<<"\n";
  }

  fout << "N_POINTS:\t" << data.ncols() << std::endl;
  fout << "DIMENSION:\t" << data.nrows() << std::endl;
  fout << "DATA:" << std::endl;
  for(int i=0;i<data.ncols();i++){
    for(int j=0;j<data.nrows();j++){
      fout << data(j,i) << "\t";
    }
    fout << std::endl;
  }
  fout.close();
}


/*******************************************************
ReadMatrixFile
  reads the output of WriteMatrixFile
*******************************************************/
void ReadMatrixFile(Matrix<double>& data,
		    const std::string filename,
		    bool quiet/*=false*/)
{
  //declare variables
  int N = 0;
  int D = 0;
  std::string str;
  my_ifstream fin;
  
  //open file and check that it exists
  fin.open(filename.c_str());
  if(!fin)
    throw FileReadError("Could not open file",filename);
  
  if(!quiet)
    std::cout << "ReadMatrixFile():\n"
	      << " - Reading file \'" << filename << "\': ";
  
  //read file word by word until DATA is reached
  fin >> str;
  while(!fin.eof() and (str!="DATA:")) // Read line by line
    {
      if (str=="N_POINTS:"){
	fin >> N;
      }
      else if (str=="DIMENSION:"){
	fin >> D;
      }
      fin >> str;
    }
  
  if(!quiet)
    std::cout << "[" << D << " x " << N 
	      << "] matrix." << std::endl;
  
  //check to make sure file format is correct
  // and D and N have been defined
  if (fin.eof())
    throw FileReadError("Keyword \"DATA\" expected:",filename);

  if (N==0)
    throw FileReadError("Nonzero keyword \"N_POINTS\" expected:",filename);

  if (D==0)
    throw FileReadError("Nonzero keyword \"DIMENSION\" expected:",filename);
  
  //reallocate a [D x N] matrix
  if(data.nrows()!=D || data.ncols()!=N)
    data.reallocate(D,N,0.0);

  char c;
  
  for(int j=0;j<N;j++){ //iterate through points
    for(int i=0;i<D;i++){ //iterate through dimensions
      fin >> data(i,j);
      if (fin.eof())
	throw FileReadError("More data expected:",filename);
      if (fin.fail())
	throw FileReadError("Data must be all floats:",filename);
    }
  }

  fin>>str;
  if (!fin.eof()){
    std::cerr << "----------------------------------------------\n"
	      << "| Warning: file has more data than expected: |\n"
	      << "|  suggest checking N_POINTS and DIMENSION   |\n" 
	      << "----------------------------------------------\n";
  }
  
  fin.close();
}

/*******************************************************
WriteVectorFile
  writes a vector to a file
*******************************************************/
void WriteVectorFile(const Vector<double>& data,
		     const std::string filename,
		     const std::string comment/*=""*/,
		     bool quiet/*=false*/)
{
  if(!quiet)
    std::cout << "WriteVectorFile():\n"
	      << " - Writing rank-" << data.size() 
	      << " vector to file: \'" << filename << "\'\n";
  
  std::ofstream fout(filename.c_str());
  
  //write comment string
  for(int i=0;i<comment.size();i++){
    if(i==0)
      fout << "#";
    fout << comment[i];
    if(comment[i]=='\n')
      fout<<"#";
    if(i+1==comment.size())
      fout<<"\n";
  }
  
  fout << "N_POINTS:\t" << data.size() << std::endl;
  fout << "DATA:" << std::endl;
  for(int i=0;i<data.size();i++)
    fout << data(i) << std::endl;
  fout.close();
}


/*******************************************************
ReadVectorFile
  reads the output of WriteVectorFile
*******************************************************/
void ReadVectorFile(Vector<double>& data,
		    const std::string filename,
		    bool quiet/*=false*/)
{
  //declare variables
  int N = 0;
  std::string str;
  my_ifstream fin;
  
  //open file and check that it exists
  fin.open(filename.c_str());
  if(!fin)
    throw FileReadError("File could not be opened:",filename);
  
  if(!quiet)
    std::cout << "ReadVectorFile():\n"
	      << " - Reading file \'" << filename << "\': ";
  
  //read file word by word until DATA is reached
  fin >> str;
  while(!fin.eof() and (str!="DATA:")) // Read line by line
    {
      if (str=="N_POINTS:"){
	fin >> N;
	if(!quiet)
	  std::cout << "rank-" << N << " vector." << std::endl;
      }
      else if (str=="DIMENSION:"){
	std::cerr << "ReadVectorFile: Warning: " << filename 
		  << " is a Matrix file!  Will read as vector.\n";
      }
      fin >> str;
    }
  
  
  //check to make sure file format is correct
  // and D and N have been defined
  if (fin.eof())
    throw FileReadError("Keyword \"DATA\" expected",filename);

  if (N==0)
    throw FileReadError("Nonzero keyword \"N_POINTS\" expected",filename);
  
  //create new size-N Vector
  data.reallocate(N,0.0);
  
  for(int i=0;i<N;i++){ //iterate through points
    fin >> data(i);
    if (fin.fail())
      throw FileReadError("Data must be all floats:",filename);
    if (fin.eof())
      throw FileReadError("Keyword N must match number of points:",filename);
  }
    
  fin>>str;
  if (!fin.eof()){
    std::cerr << "----------------------------------------------\n"
	      << "| Warning: file has more data than expected: |\n"
	      << "|  suggest checking N_POINTS                 |\n" 
	      << "----------------------------------------------\n";
  }
  
  fin.close();
}
