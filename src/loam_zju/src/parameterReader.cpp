#include "loam_zju/parameterReader.h"

using namespace std;

PARAMETER_READER::PARAMETER_READER(const string filename)
{
  ifstream fin(filename.c_str());
  if(!fin)
  {
    cerr<<"### Warnning: Parameter file does not exist."<<endl;
    return;
  }

  while(!fin.eof())
  {
    string str;
    getline(fin, str);
    if(str[0] == '#')  // comment
      continue;

    int pos = str.find("=");
    if(pos == -1)  // "=" not found
      continue;
    string key = str.substr(0, pos);
    string value = str.substr(pos+1, str.length());
    data[key] = value;

    if(!fin.good())
      break;
  }
}

string PARAMETER_READER::getValue(const string key)
{
  map<string, string>::iterator iter = data.find(key);
  if(iter == data.end())
  {
    cerr<<"### Error: parameter name "<< key <<" not found!"<<endl;
    return string("NOT_FOUND");
  }
  cout<<" "<<key<<": "<<iter->second<<endl;
  return iter->second;
}
