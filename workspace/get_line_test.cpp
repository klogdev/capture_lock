#include <fstream>
#include <string>
#include "string.h"
using namespace std;

int main(int argc, char** argv){
    std::string path = argv[1];
    std::ifstream file(path);
    CHECK(file.is_open()) << path;

    std::string line;
    std::string item;
    while (std::getline(file, line)) {
    StringTrim(&line);

    if (line.empty() || line[0] == '#') {
      continue;
    }

    std::stringstream line_stream1(line);
    std::cout << line << std::endl;
    // ID
    std::getline(line_stream1, item, ' ');
    std::cout << item << std::endl;
}
}