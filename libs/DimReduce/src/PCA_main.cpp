//#include "PCA.h"
//#include <string>
//#include <sstream>


///************************************************************
// PCA_main
//************************************************************/


//std::string doc_str(){
//  return
//    "------------------------------------------------------------\n"
//    "  PCA:                                                      \n"
//    "   usage: PCA filelist -m [method] -d [d_out] -v [var] -q   \n"
//    "                       -s -e -a                             \n"
//    "------------------------------------------------------------\n";
//}


//void BadArg(char arg=' ',
//	    const std::string& argval = "")
//{
//  std::cerr << "Error: -" << arg << " " << argval
//	    << " : \n  Unrecognized argument.\n";
//  std::cerr << doc_str();
//  std::exit(-1);
//}

//void BadArg(const std::string& arg)
//{
//  std::cerr << "Error: " << arg
//	    << " : \n  Unrecognized argument.\n";
//  std::cerr << doc_str();
//  std::exit(-1);
//}

//int main(int argc,char *argv[]){
//  if (argc==1){
//    std::cerr << doc_str();
//    std::exit(0);
//  }

//  char arg;
//  std::istringstream iss;

//  std::vector<std::string> infile_list;

//  std::string method="";

//  PCA_TYPE PT = SVD_PCA;

//  PCA* PCAobj;
  
//  //first go through arguments to determine method being used
//  for(int i=1;i<argc;i++)
//    {
//      if (argv[i][0]=='-' && argv[i][1]=='m')
//	{
//	  iss.clear();
//	  iss.str(argv[++i]);
//	  if( (iss >> method).fail() )
//	    BadArg(arg,argv[i]);
//	  break;
//	}
//    }

//  if(method=="")
//    PCAobj = new PCA;
//  else if(method=="PCA")
//    PCAobj = new PCA;
//  else if(method=="IRWPCA")
//    PCAobj = new IRWPCA;
//  else if(method=="WPCA")
//    PCAobj = new WPCA;
//  else{
//    std::cerr << "Unrecognized method: " << method << "\n";
//    std::exit(-1);
//  }

//  // go through arguments and find the needed info
//  for(int i=1;i<argc;i++)
//    {
//      if (argv[i][0]=='-')
//	{
//	  arg = argv[i][1];
	  
//	  if (arg=='i')
//	    {
//	      std::cout << doc_str();
//	      std::exit(0);
//	    }

//	  else if(arg=='q')
//	    {
//	      PCAobj->set_quiet(true);
//	      continue;
//	    }

//	  else if(arg=='s')
//	    {
//	      PT = SVD_PCA;
//	      continue;
//	    }

//	  else if(arg=='e')
//	    {
//	      PT = EIG_PCA;
//	      continue;
//	    }

//	  else if(arg=='a')
//	    {
//	      PT = AREIG_PCA;
//	      continue;
//	    }
	  
//	  //-----------------------------------
//	  //arguments with associated value
//	  if (i==argc-1)
//	    BadArg(argv[i]);
	  
//	  //open stream to associated value
//	  iss.clear();
//	  iss.str(argv[++i]);
	  
//	  switch(arg){
	    
//	  case 'd' :
//	    int d;
//	    if( (iss >> d).fail() )
//	      BadArg(arg,argv[i]);
//	    PCAobj->set_d_out(d);
//	    break;
	    
//	  case 'v' :
//	    double var;
//	    if( (iss >> var).fail() )
//	      BadArg(arg,argv[i]);
//	    PCAobj->set_var(var);
//	    break;

//	  case 'm':
//	    break;
	      
//	  default :
//	    BadArg(arg,argv[i]);
	    
//	  }//switch
//	}//if

//      else
//	{
//	  infile_list.push_back(std::string(argv[i]) );
//	}
//    }//for

//  for(int i=0;i<argc;i++)
//    std::cout << argv[i] << " ";
//  std::cout << "\n";

//  /* compute PCA               */
//  for(int i=0;i<infile_list.size();i++){
//    std::cout << "--------------------------------------------------\n";
//    std::cout << infile_list[i] << "\n";
//    try
//      {
//	PCAobj->load_fits(infile_list[i]);
//	PCAobj->compute(PT);
//	PCAobj->update_training_file();
//      }
//    catch(std::exception& e)
//      {
//	std::cerr << "Exception raised: " << e.what() << "\n"
//		  << "  skipping " << infile_list[i] << "\n";
//      }
    
//    if( PCAobj->has_training_data() )
//      PCAobj->clear_training_data();
//  }
//  std::cout << "--------------------------------------------------\n";
//}






