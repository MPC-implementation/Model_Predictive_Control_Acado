#include "logging.h"

using json = nlohmann::json;
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
        json jf = json::parse(fileI);
        jf.insert(jf.end(), signal_.begin(), signal_.end());
        std::ofstream fileO(path);
        fileO << jf;
    }
}

