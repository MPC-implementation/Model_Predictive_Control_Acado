#ifndef LOGGING_H
#define LOGGING_H
#include "json.hpp"
#include <fstream>

using namespace std;
class Log
{
  public:
  Log( string path);
  void StartLogging(float sig);
  


  private:
  float signal;
  string path;
  bool flgInit =false;
};

#endif /* LOGGING_H */
