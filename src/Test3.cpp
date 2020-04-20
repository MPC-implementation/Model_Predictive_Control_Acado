#include <iostream>
#include <fstream>
#include "json.hpp"
#include <vector>
using json = nlohmann::json;
using namespace std;
json merge(const json &a, const json &b)
{
    json result = a.flatten();
    json tmp = b.flatten();

    for (json::iterator it = tmp.begin(); it != tmp.end(); ++it)
    {
        result[it.key()] = it.value();
    }

    return result.unflatten();
}


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


Log::Log(string path_) : path(path_) {} //  constructor, set logging signal

void Log::StartLogging(float sig)
{
    json signal_ = {sig};  // Type conversion to json
    if (flgInit == false)   // Only start logging
    {
        flgInit = true;
        //remove file and create logging file
        std::ofstream fileO(path);
        fileO << signal_;
    }
    else  // After start logging
    {
        std::ifstream fileI(path);
     //   vector<float> a;
      //  a.push_back(fileI);
        
        json jf = json::parse(fileI);


        jf.insert(jf.end(), signal_.begin(), signal_.end());

        std::ofstream fileO(path);
        fileO << jf;
    }
}



int main()
{
    auto jsonObjects = json::array();

    json t = {3 ,6};
    cout << t <<endl;
    jsonObjects.push_back(t);

    std::ofstream file2("../logging/Logging4.json");
    file2 << t;
    file2.close();

    std::ifstream fileR("../logging/Logging4.json");
    json t2 = {4};
    json jf = json::parse(fileR);
    cout<<jf<<endl;
    
    jf.insert(jf.end(), t2.begin(), t2.end());
    std::ofstream file("../logging/Logging4.json");
    file << jf;
    file.close();


    std::ifstream fileR2("../logging/Logging4.json");
    json t3 = {5};
    json jf2 = json::parse(fileR2);

    cout << jf2 <<endl;
    jf2.insert(jf2.end(), t3.begin(), t3.end());
    cout << jf2 <<endl;



    Log logSteering( "../logging/LoggingTest.json");
    logSteering.StartLogging(2);
    logSteering.StartLogging(3);



    // std::ofstream file3("../logging/Logging4.json");
    // file3 << jf2[0];
    // file3.close();
    
    return 0;
}