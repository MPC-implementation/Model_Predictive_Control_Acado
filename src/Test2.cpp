#include "json.hpp"
#include <fstream>

using jsonf = nlohmann::json;

int main() {
    jsonf jsonfile;



    jsonfile["foo"] = "bar";

    std::ofstream file("key.json");
    file << jsonfile;
}