//#include "LLE.h"
//#include "HLLE.h"
//#include "RLLE.h"
//#include <string>
//#include <sstream>
//#include <vector>

//#include "doc.h"

//int main(int argc,char *argv[]){
//  if (argc==1){
//    std::cerr << doc_str();
//    std::exit(0);
//  }

//  char arg;
//  std::istringstream iss;

//  std::vector<std::string> infile_list;

//  std::string method="";

//  std::string training_file;
//  bool use_training_file = false;

//  LLE* LLEobj;
  
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
//    LLEobj = new LLE;
//  else if(method=="LLE")
//    LLEobj = new LLE;
//  else if(method=="HLLE")
//    LLEobj = new HLLE;
//  else if(method=="RLLE1")
//    LLEobj = new RLLE1;
//  else if(method=="RLLE2")
//    LLEobj = new RLLE2;
//  else if(method=="LLEsig")
//    LLEobj = new LLE;
//  else{
//    std::cerr << "Unrecognized method: " << method << "\n";
//    std::exit(-1);
//  }

//  //now go through arguments and find the rest of the needed info
//  for(int i=1;i<argc;i++)
//    {
//      if (argv[i][0]=='-')
//	{
//	  arg = argv[i][1];
	  
	  
	  
//	  if (arg=='h')
//	    {
//	      PrintMethodHelp(method);
//	      std::exit(0);
//	    }
	  
//	  else if (arg=='i')
//	    {
//	      std::cout << info_str();
//	      std::exit(0);
//	    }

//	  else if(arg=='q')
//	    {
//	      LLEobj->set_quiet(true);
//	      continue;
//	    }

//	  else if(arg=='f')
//	    {
//	      LLEobj->set_recalc(true);
//	      continue;
//	    }

//	  else if(arg=='H')
//	    {
//	      LLEobj->set_hessian(true);
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
//	  case 'm' :
//	    //method has already been read
//	    break;
	    
//	  case 'k' :
//	    int k;
//	    if( (iss >> k).fail() )
//	      BadArg(arg,argv[i]);
//	    LLEobj->set_k(k);
//	    break;
	    
//	  case 'd' :
//	    int d;
//	    if( (iss >> d).fail() )
//	      BadArg(arg,argv[i]);
//	    LLEobj->set_d_out(d);
//	    break;
	    
//	  case 'v' :
//	    double var;
//	    if( (iss >> var).fail() )
//	      BadArg(arg,argv[i]);
//	    LLEobj->set_var(var);
//	    break;
	    
//	  case 'r' :
//	    double r;
//	    if( (iss >> r).fail() )
//	      BadArg(arg,argv[i]);
//	    LLEobj->set_r(r);
//	    break;
	    
//	  case 't' :
//	    if(method=="LLEsig")
//	      BadArg(arg,argv[i],method);
	    
//	    if(use_training_file){
//	      std::cerr << "Warning: duplicate training file! "
//			<< "Will only use the first\n";
//	    }else{
//	      if( (iss >> training_file).fail() )
//		BadArg(arg,argv[i]);
//	      use_training_file = true;
//	      break;
//	    }
	    
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

//  if(infile_list.size()==0)
//    PrintMethodHelp(method);
    

//  /* projecting data on other training */
//  if(use_training_file)
//    {
//      std::cout << "--------------------------------------------------\n";
//      LLEobj->load_fits(training_file);
//      LLEobj->compute_projection();
      
//      for(int i=0;i<infile_list.size();i++){
//	std::cout << "--------------------------------------------------\n";
//	try
//	  {
//	    LLEobj->load_test_data(infile_list[i]);
//	    LLEobj->compute_test_projection();
//	    LLEobj->update_test_file();
//	  }
//	catch(std::exception& e)
//	  {
//	    std::cerr << "Exception raised: " << e.what() << "\n"
//		      << "  skipping " << infile_list[i] << "\n";
//	  }
	
//	if( LLEobj->has_test_data() )
//	  LLEobj->clear_test_data();
//      }
//      std::cout << "--------------------------------------------------\n";
      
//    }

//  /* normal training procedure */
//  else
//    {
//      for(int i=0;i<infile_list.size();i++){
//	std::cout << "--------------------------------------------------\n";
//	try
//	  {
//	    LLEobj->load_fits(infile_list[i]);
	    
//	    if(method=="LLEsig")
//	      LLEobj->compute_sigma();
//	    else
//	      LLEobj->compute_projection();
	    
//	    LLEobj->update_training_file();
//	  }
//	catch(std::exception& e)
//	  {
//	    std::cerr << "Exception raised: " << e.what() << "\n"
//		      << "  skipping " << infile_list[i] << "\n";
//	  }
	  
//	if( LLEobj->has_training_data() )
//	  LLEobj->clear_training_data();
//      }
//      std::cout << "--------------------------------------------------\n";
//    }
  
//  delete LLEobj;
//}
