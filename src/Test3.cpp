#include <iostream>
#include <fstream>
#include "json.hpp"
#include <vector>
using json = nlohmann::json;
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


int main()
{
    auto jsonObjects = json::array();
    std::vector<double> b;
    b.push_back(4);
    json t = {3}; 
    t.push_back(6);
    jsonObjects.push_back(t);

    std::ofstream file2("../logging/Logging4.json");
    file2 << jsonObjects;
    file2.close();
    



     std::ifstream fileR("../logging/Logging4.json");


     json jf = json::parse(fileR);
     
// //    std::vector<double> a;
//   //  a.push_back(3);
//   //  json v2 = a;
//    // v2.insert(v2.end(), b.begin(), b.end());
     jsonObjects.push_back(jf.flatten());
     //jsonObjects.push_back(t);

   std::ofstream file("../logging/Logging4.json");
//  //   std::cout<<jf<<std::endl;
 file << jsonObjects;

    return 0;
}