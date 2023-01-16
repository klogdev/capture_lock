#ifndef STRING_H_
#define STRING_H_

#include <string>
#include <vector>

bool IsNotWhiteSpace(const int character);
void StringTrim(const std::string& str);
void StringLeftTrim(std::string* str);
void StringRightTrim(std::string* str);

#endif  // STRING_H_
