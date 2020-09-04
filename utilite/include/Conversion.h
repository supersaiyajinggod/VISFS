#ifndef CONVERSION_H
#define CONVERSION_H

#include <string>

/** \brief Replace old characters in a string to new ones.
  * \param[in] str The string. 
  * \param[in] before The character to be replaced by the new one.
  * \param[in] after The new character replacing the old character.
  * \return The modified string.
  * \author eddy
  * \code
  * std::string str = "Hello";
  * uReplaceChar(str, 'l', 'p');
  * // The results is str = "Heppo";
  * \endcode
  */
std::string uReplaceChar(const std::string & str, char before, char after);


/** \brief Replace old characters in a string with the specified string.
  * \param[in] str The string. 
  * \param[in] before The character to be replaced by the new one.
  * \param[in] after The new string replacing the old character.
  * \return The modified string.
  * \author eddy
  * \code
  * std::string str = "Hello";
  * uReplaceChar(str, 'o', "oween");
  * // The results is str = "Helloween";
  * \endcode
  */
std::string uReplaceChar(const std::string & str, char before, const std::string & after);

/** \brief Transform characters from a string to upper case.
  * \param[in] str The string to convert. 
  * \return The modified string.
  * \author eddy
  */
std::string uToUpperCase(const std::string & str);

/** \brief Transform characters from a string to lower case.
  * \param[in] str The string to convert. 
  * \return The modified string.
  * \author eddy
  */
std::string uToLowerCase(const std::string & str);

/** \brief Convert a string to an integer
  * \param[in] str The string to convert. 
  * \return The integer.
  * \author eddy
  */
int uStr2Int(const std::string & str);

/** \brief Convert a string to a float independent of the locale (comma/dot).
  * \param[in] str The string to convert in a float. 
  * \return The float number.
  * \author eddy
  */
float uStr2Float(const std::string & str);

/** \brief Convert a string to a double independent of the locale (comma/dot).
  * \param[in] str The string to convert in a float. 
  * \return The double number.
  * \author eddy
  */
double uStr2Double(const std::string & str);

/** \brief Convert a string to a boolean. 
  * \param[in] str The string to convert in a boolean. 
  * \return The boolean.
  * \author eddy
  */
bool uStr2Bool(const char * str);
bool uStr2Bool(const std::string & str);

/**
 * Convert a number (unsigned int) to a string.
 * @param number the number to convert in a string
 * @return the string
 */
std::string uNumber2Str(unsigned int number);
/**
 * Convert a number (int) to a string.
 * @param number the number to convert in a string
 * @return the string
 */
std::string uNumber2Str(int number);
/**
 * Convert a number (float) to a string.
 * @param number the number to convert in a string
 * @return the string
 */
std::string uNumber2Str(float number);
/**
 * Convert a number (double) to a string.
 * @param number the number to convert in a string
 * @return the string
 */
std::string uNumber2Str(double number);

/**
 * Convert a bool to a string.
 * The format used is "true" and "false".
 * @param boolean the boolean to convert in a string
 * @return the string
 */
std::string uBool2Str(bool boolean);

#endif  // CONVERSION_H