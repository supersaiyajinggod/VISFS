#include <sstream>
#include <string.h>
#include <stdlib.h>
#include <cmath>

#include "Conversion.h"

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

std::vector<int> uUlVector2Int(const std::vector<std::size_t> & vector) {
	std::vector<int> vectorInt;
	vectorInt.resize(vector.size());
	for (std::size_t i = 0; i < vector.size(); ++i) {
		vectorInt[i] = static_cast<int>(vector[i]);
	}
	return vectorInt;
}

std::vector<std::size_t> uIntVector2Ul(const std::vector<int> & vector) {
	std::vector<std::size_t> vectorUl;
	vectorUl.resize(vector.size());
	for (std::size_t i = 0; i < vector.size(); ++i) {
		vectorUl[i] = static_cast<int>(vector[i]);
	}
	return vectorUl;
}

boost::posix_time::ptime uTimeDouble2Boost(const double time) {
	int64_t sec64 = static_cast<int64_t>(floor(time));
    if (sec64 < 0 || sec64 > std::numeric_limits<uint32_t>::max())
    	std::cout << "Time is out of dual 32-bit range" << std::endl;
    uint32_t sec = static_cast<uint32_t>(sec64);
	uint32_t nsec = static_cast<uint32_t>(round((time-sec) * 1e9));
	sec += (nsec / 1000000000ul);
    nsec %= 1000000000ul;
#if defined(BOOST_DATE_TIME_HAS_NANOSECONDS)
    return boost::posix_time::from_time_t(sec) + boost::posix_time::nanoseconds(nsec);
#else
    return boost::posix_time::from_time_t(sec) + boost::posix_time::microseconds(nsec/1000);
#endif
}