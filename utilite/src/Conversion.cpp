#include <sstream>
#include <string.h>
#include <stdlib.h>

#include "Conversion.h"
#include "Stl.h"

std::string uReplaceChar(const std::string & str, char before, char after) {
	std::string result = str;
	for(unsigned int i=0; i<result.size(); ++i)
	{
		if(result[i] == before)
		{
			result[i] = after;
		}
	}
	return result;
}

std::string uReplaceChar(const std::string & str, char before, const std::string & after) {
	std::string s;
	for(unsigned int i=0; i<str.size(); ++i)
	{
		if(str.at(i) != before)
		{
			s.push_back(str.at(i));
		}
		else
		{
			s.append(after);
		}
	}
	return s;
}

std::string uToUpperCase(const std::string & str) {
	std::string result = str;
	for(unsigned int i=0; i<result.size(); ++i)
	{
		// only change case of ascii characters ('a' to 'z')
		if(result[i] >= 'a' && result[i]<='z')
		{
			result[i] = result[i] - 'a' + 'A';
		}
	}
	return result;
}

std::string uToLowerCase(const std::string & str) {
	std::string result = str;
	for(unsigned int i=0; i<result.size(); ++i)
	{
		// only change case of ascii characters ('A' to 'Z')
		if(result[i] >= 'A' && result[i]<='Z')
		{
			result[i] = result[i] - 'A' + 'a';
		}
	}
	return result;
}

int uStr2Int(const std::string & str) {
	return atoi(str.c_str());
}

float uStr2Float(const std::string & str) {
	float value = 0.0f;
	std::istringstream istr(uReplaceChar(str, ',', '.').c_str());
	istr.imbue(std::locale("C"));
	istr >> value;
	return value;
}

double uStr2Double(const std::string & str) {
	double value = 0.0;
	std::istringstream istr(uReplaceChar(str, ',', '.').c_str());
	istr.imbue(std::locale("C"));
	istr >> value;
	return value;
}

bool uStr2Bool(const char * str) {
	return !(str && (uStrContains(uToLowerCase(str), "false") || strcmp(str, "0") == 0));
}

bool uStr2Bool(const std::string & str) {
	return !(uStrContains(uToLowerCase(str), "false") || str.compare("0") == 0);
}

std::string uNumber2Str(unsigned int number) {
	std::stringstream s;
	s << number;
	return s.str();
}

std::string uNumber2Str(int number) {
	std::stringstream s;
	s << number;
	return s.str();
}

std::string uNumber2Str(float number) {
	std::stringstream s;
	s << number;
	return s.str();
}

std::string uNumber2Str(double number) {
	std::stringstream s;
	s << number;
	return s.str();
}

std::string uBool2Str(bool boolean) {
	std::string s;
	if (boolean) {
		s = "true";
	} else {
		s = "false";
	}
	return s;
}
