#ifndef FILENAMES_H
#define FILENAMES_H

#include <string>
#include <iostream>

std::string add_preext(std::string s,std::string ext)
{
  int i = s.rfind('.');
  
  if(i==-1)
    return s+"-"+ext;

  std::string s_ret = "";
  s_ret += s.substr(0,i);
  s_ret += "-" + ext;
  s_ret += s.substr(i,s.size()-i);
    
  return s_ret;
}

std::string file_ext(std::string filename)
{
  int i = filename.rfind('.');
  
  if(i==-1)
    return "";

  return filename.substr(i+1,filename.size()-i);
}

#endif /*FILENAMES_H*/
