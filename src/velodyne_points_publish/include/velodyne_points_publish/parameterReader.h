#ifndef PARAMETERREADER_H
#define PARAMETERREADER_H

#include <map>
#include <iostream>
#include <string>
#include <fstream>

class PARAMETER_READER
{
public:
  PARAMETER_READER(const std::string filename);
  std::string getValue(const std::string key);
private:
  std::map<std::string,std::string> data;
};

#endif // PARAMETERREADER_H
