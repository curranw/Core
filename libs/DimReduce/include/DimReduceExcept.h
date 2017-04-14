#ifndef DIMREDUCE_EXCEPT_H
#define DIMREDUCE_EXCEPT_H

#include <string>

/************************************************************/

class TrainingError : public std::exception {
 public:
  TrainingError(const std::string& msg) throw(): exception(), msg_(msg) {}
  ~TrainingError() throw(){}
  const char * what() const throw() { return msg_.c_str(); }
 private:
  std::string msg_;
};

/************************************************************/

class FileReadError : public std::exception {
public:
  FileReadError(const std::string& msg, const std::string& filename) throw()
    : exception(), msg_(msg), filename_(filename) {}
  ~FileReadError() throw(){}
  const char * what() const throw() { return ("FileError: " + filename_ + ": " + msg_).c_str(); }
private:
  std::string msg_;
  std::string filename_;
};

/************************************************************/

class FitsKeyError : public std::exception {
public:
  FitsKeyError(const std::string& filename, const std::string& key) throw()
    : exception(), filename_(filename), key_(key) {}
  ~FitsKeyError() throw(){}
  const char * what() const throw() {return ("FitsKeyError: " + key_ + 
					     " not found in " + filename_).c_str(); }
private:
  std::string key_;
  std::string filename_;
};


/************************************************************/

#endif /* DIMREDUCE_EXCEPT_H */
