#ifndef DOC_H
#define DOC_H


/******************************************************
  Documentation for the various dimensionality reduction
   algorithms.  Used by the main DimReduce.cpp program
********************************************************/
#include <string.h>

bool IsValidMethod(const std::string& method){
  return (method=="LLE" ||
	  method=="HLLE" ||
	  method=="RLLE1" ||
	  method=="RLLE2" ||
	  method=="LLEsig");
}

void PrintMethodHelp(const std::string& method);

void BadArg(char arg=' ', 
	    const std::string& argval = "", 
	    const std::string& method = "")
{
  std::cerr << "Error: -" << arg << " " << argval
	    << " : \n  Unrecognized argument.\n";
  PrintMethodHelp(method);
  std::exit(-1);
}

void BadArg(const std::string& arg, 
	    const std::string& method="")
{
  std::cerr << "Error: " << arg
	    << " : \n  Unrecognized argument.\n";
  PrintMethodHelp(method);
  std::exit(-1);
}

void PrintWarning(const std::string & method, 
		  const char & arg);

/************************************************************/

std::string infile_doc(){ 
  return
    "|        <infile>     : specifies the location of the data matrix  |\n"
    "|                        (required)                                |\n";
}

std::string h_doc(){
  return
    "|        -h           : print description of given method.  If no  |\n"
    "|                        method is specified, print this screen.   |\n";
}

std::string i_doc(){
  return
    "|        -i           : print program info and quit                |\n";
}

std::string k_doc(){
  return
    "|        -k <kval>    : use <kval> nearest neighbors               |\n";
}

std::string d_doc(){
  return
    "|        -d <d_out>   : set output dimension to <d_out>            |\n"
    "|                        (if <d_out> is not set, it will be        |\n"
    "|                         computed using <var> )                   |\n";
}

std::string v_doc(){
  return
    "|        -v <var>     : compute <d_out> to preserve an             |\n"
    "|                       average variance of <var>.  Used if        |\n"
    "|                       d_out is unspecified.                      |\n";
}

std::string f_doc(){
  return
    "|        -f           : force recalulate of already                |\n"
    "|                        calculated quantities                     |\n";
}

std::string w_doc(){
  return
    "|        -w           : whiten data before projecting              |\n";
}

std::string q_doc(){
  return 
    "|        -q           : quiet - supress output                     |\n";
}

std::string r1_doc(){
  return
    "|        -r <r_cut>   : set R-score cutoff to <r_cut>              |\n"
    "|                        (default r_cut = 0)                       |\n";
}

std::string H_doc(){
  return
    "|        -H           : compute with HLLE rather than normal LLE   |\n";
}

std::string r2_doc(){
  return
    "|        -r <rval>    : set r-value to <rval>.  k+r nearest        |\n"
    "|                       neighbors will be found, and the k most-   |\n"
    "|                       reliable will be used for the projection   |\n"
    "|                        (default rval = floor(k/5) )              |\n";
}

std::string t_doc(){
  return
    "|        -t <training_data> :                                      |\n"
    "|                       train on this data (rather than training   |\n"
    "|                        on each specific infile)                  |\n";
}   

std::string LLE_doc(){
  return
    "====================================================================\n"
    "| DimReduce LLE routine                                            |\n"
    "|   This performs the Locally Linear Embedding algorithm described |\n"
    "|    in [1], with improvements suggested in [2]                    |\n"
    "|                                                                  |\n"
    "|   LLE computes a dimension-<d_out> embedding of a                |\n"
    "|    dimension-<d_in> dataset which preserves neighborhoods        |\n"
    "|    each data point                                               |\n"
    "|                                                                  |\n"
    "|   Arguments for LLE algorithm:                                   |\n"
    + infile_doc() + k_doc() + d_doc() 
    + v_doc() + t_doc() + w_doc() + q_doc() + f_doc() +
    "|                                                                  |\n"
    "|  [1] Saul,L. & Roewis,S., 'An Introduction to Locally Linear     |\n"
    "|       Embedding', 2001                                           |\n"
    "|  [2] deRidder,D. & Duin, R.P.W., 'Locally Linear Embedding       |\n"
    "|       For Classification', 2002                                  |\n"
    "====================================================================\n";
}

std::string HLLE_doc(){
  return
    "====================================================================\n"
    "| DimReduce HLLE routine                                           |\n"
    "|   This performs the Hessian Locally Linear Embedding algorithm   |\n"
    "|    described in [1]                                              |\n"
    "|                                                                  |\n"
    "|   HLLE computes a dimension-<d_out> embedding of a               |\n"
    "|    dimension-<d_in> dataset which preserves neighborhoods        |\n"
    "|    each data point. It is similar to the LLE routine, but        |\n"
    "|    uses a local Hessian estimator to preserve angles within      |\n"
    "|    the projected neighborhoods.                                  |\n"
    "|                                                                  |\n"
    "|   Arguments for HLLE algorithm:                                  |\n"
    + infile_doc() + k_doc() + d_doc() 
    + v_doc() + t_doc() + w_doc() + q_doc() + f_doc() +
    "|                                                                  |\n"
    "|  [1] Grimes,C. & Donoho,D., 'Hessian Eigenmaps: New Locally      |\n"
    "|       Linear Embedding Techniques for High Dimensional Data',    |\n"
    "|       2003                                                       |\n"
    "====================================================================\n";
}

std::string RLLE1_doc(){
  return
    "====================================================================\n"
    "| DimReduce RLLE1 routine                                          |\n"
    "|   This performs a Robust Locally Linear Embedding algorithm as   |\n"
    "|    described in [1].  Reliability scores are computed for each   |\n"
    "|    data point using an iteratively reweighted least-squares      |\n"
    "|    analysis within each neighborhood, and taking into account    |\n"
    "|    the number of neighborhoods each point belongs to.  (H)LLE    |\n"
    "|    is then performed on all points with reliability greater than |\n"
    "|    r_cutoff.                                                     |\n"
    "|                                                                  |\n"
    "|   Arguments for RLLE1 algorithm:                                 |\n"
    + infile_doc() + k_doc() + d_doc() + v_doc() 
    + r1_doc() + H_doc() + t_doc() + w_doc() + q_doc() + f_doc() +
    "|                                                                  |\n"
    "|  [1] Chang, H. & Yeung, D., 'Robust Locally Linear Embedding',   |\n"
    "|       2005                                                       |\n"
    "====================================================================\n";
}

std::string RLLE2_doc(){
  return
    "====================================================================\n"
    "| DimReduce RLLE2 routine                                          |\n"
    "|   This performs a second Robust Locally Linear Embedding         |\n"
    "|    algorithm as described in [1].  Reliability scores are        |\n"
    "|    computed for each datapoint using an iteratively reweighted   |\n"
    "|    least-squares analysis within each neighborhood, and taking   |\n"
    "|    into account the number of neighborhoods each point belongs   |\n"
    "|    to.  RLLE2 then finds the k+r nearest neighbors of each point |\n"
    "|    and uses the k nearest to find a weighted LLE projection of   |\n"
    "|    the data.                                                     |\n"
    "|                                                                  |\n"
    "|   Arguments for RLLE2 algorithm:                                 |\n"
    + infile_doc() + k_doc() + d_doc() + v_doc() 
    + r2_doc() + H_doc() + t_doc() + w_doc() + q_doc() + f_doc() +
    "|                                                                  |\n"
    "|  [1] Chang, H. & Yeung, D., 'Robust Locally Linear Embedding',   |\n"
    "|       2005                                                       |\n"
    "====================================================================\n";
}

std::string LLEsig_doc(){
  return
    "====================================================================\n"
    "| DimReduce LLEsig routine                                         |\n"
    "|   Given a set of points, an output dimension, and a number of    |\n"
    "|    nearest neighbors, this outputs a vector of reconstruction    |\n"
    "|    errors for each point based on its neighborhood.  Those       |\n"
    "|    points with small reconstruction errors contribute less to    |\n"
    "|    the overall fit.  This diagnostic can be used to select the   |\n"
    "|    most important subset of data for training.                   |\n"
    "|                                                                  |\n"
    "|   Arguments for LLEsig algorithm:                                |\n"
    + infile_doc() + k_doc() + d_doc() + q_doc() + f_doc() +
    "====================================================================\n";
}

std::string info_str(){
  return
    "===============================================\n"
    "|  Written by Jake VanderPlas                 |\n"
    "|    Astronomy Department,                    |\n"
    "|    University of Washington                 |\n"
    "|                                             |\n"
    "|  http://www.astro.washington.edu/vanderplas |\n"
    "|                                             |\n"
    "|  email: vanderplas@astro.washington.edu     |\n"
    "===============================================\n";
}

std::string doc_str(){
  return
    "====================================================================\n"
    "| Usage:                                                           |\n"
    "|   DimReduce <infile> -m <method>                                 |\n"
    "|                                                                  |\n"
    "|    <method> is one of the following:                             |\n"
    "|         LLE (default), HLLE, RLLE1, RLLE2, LLEsig                |\n"
    "|                                                                  |\n"
    "|  General arguments:                                              |\n"
    + h_doc() + i_doc() +
    "|                                                                  |\n"
    "| For description and usage of a particular method, type           |\n"
    "|     DimReduce -m <method> -h                                     |\n"
    "====================================================================\n";
}

void PrintMethodHelp(const std::string& method){
  if(method=="LLE")
    std::cout << LLE_doc();
  else if(method == "HLLE")
    std::cout << HLLE_doc();
  else if(method == "RLLE1")
    std::cout << RLLE1_doc();
  else if(method == "RLLE2")
    std::cout << RLLE2_doc();
  else if(method == "LLEsig")
    std::cout << LLEsig_doc();
  else
    std::cout << doc_str();
}


#endif /* DOC_H */
