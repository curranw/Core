//#include "fitstools.h"
//#include <string>

//std::string doc_str(){
//  return
//    "====================================================================\n"
//    "| fits_stat routine                                                |\n"
//    "|  This lists the contents of fits files                           |\n"
//    "|     Usage: fits_stat filename1 filename2 filename3 ...           |\n"
//    "====================================================================\n";
//}

//int main(int argc,char *argv[]){
//  if(argc<2){
//    std::cerr << doc_str();
//    std::exit(0);
//  }
  
//  for(int i=1;i<argc;i++){
//    std::string filename(argv[i]);
//    std::cout << "------------------------------"
//	      << "------------------------------\n"
//	      << " " << filename << "\n";
    
//    try{
//      fitstools ffile(filename);
//      int N = ffile.num_hdus();
//      for(int i=1;i<=N;i++){
//	ffile.move_to_hdu(i);
//	std::cout << "  " <<  ffile.current_hdu_info();
//      }
//    }catch(std::exception& e){
//      std::cerr << "   Error: " << e.what() << "\n"
//		<< "           file may not be a valid fits file\n";
//    }
//  }
//  std::cout << "------------------------------"
//	    << "------------------------------\n";
//  return 0;
//}
