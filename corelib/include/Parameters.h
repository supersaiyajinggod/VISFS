#ifndef VISFS_PARAMETERS
#define VISFS_PARAMETERS

#include <iostream>
#include <string>
#include <map>

namespace VISFS {

typedef std::map<std::string, std::string> ParametersMap;
typedef std::pair<std::string, std::string> ParametersPair;

/**
 * Macro used to create parameter's key and default value.
 * This macro must be used only in the Parameters class definition (in this file).
 * They are automatically added to the default parameters map of the class Parameters.
 * Example:
 * @code
 *         //for PARAM(Video, ImageWidth, int, 640), the output will be :
 *         public:
 *             static std::string kVideoImageWidth() {return std::string("Video/ImageWidth");}
 *             static int defaultVideoImageWidth() {return 640;}
 *         private:
 *             class DummyVideoImageWidth {
 *             public:
 *                 DummyVideoImageWidth() {parameters_.insert(ParametersPair("Video/ImageWidth", "640"));}
 *             };
 *             DummyVideoImageWidth dummyVideoImageWidth;
 * @endcode
 */
#define VISFS_PARAM(PREFIX, NAME, TYPE, DEFAULT_VALUE, DESCRIPTION) \
    public: \
        static std::string k##PREFIX##NAME() {return std::string(#PREFIX "/" #NAME);} \
        static TYPE default##PREFIX##NAME() {return (TYPE)DEFAULT_VALUE;} \
        static std::string type##PREFIX##NAME() {return std::string(#TYPE);} \
    private: \
        class Dummy##PREFIX##NAME { \
        public: \
            Dummy##PREFIX##NAME() {parameters_.insert(ParametersPair(#PREFIX "/" #NAME, #DEFAULT_VALUE)); \
                                   parametersType_.insert(ParametersPair(#PREFIX "/" #NAME, #TYPE)); \
                                   descriptions_.insert(ParametersPair(#PREFIX "/" #NAME, DESCRIPTION));} \
        }; \
        Dummy##PREFIX##NAME dummy##PREFIX##NAME
// end define PARAM

/**
 * It's the same as the macro PARAM but it should be used for string parameters.
 * Macro used to create parameter's key and default value.
 * This macro must be used only in the Parameters class definition (in this file).
 * They are automatically added to the default parameters map of the class Parameters.
 * Example:
 * @code
 *         //for PARAM_STR(Video, TextFileName, "Hello_world"), the output will be :
 *         public:
 *             static std::string kVideoFileName() {return std::string("Video/FileName");}
 *             static std::string defaultVideoFileName() {return "Hello_world";}
 *         private:
 *             class DummyVideoFileName {
 *             public:
 *                 DummyVideoFileName() {parameters_.insert(ParametersPair("Video/FileName", "Hello_world"));}
 *             };
 *             DummyVideoFileName dummyVideoFileName;
 * @endcode
 */
#define VISFS_PARAM_STR(PREFIX, NAME, DEFAULT_VALUE, DESCRIPTION) \
    public: \
        static std::string k##PREFIX##NAME() {return std::string(#PREFIX "/" #NAME);} \
        static std::string default##PREFIX##NAME() {return DEFAULT_VALUE;} \
        static std::string type##PREFIX##NAME() {return std::string("string");} \
    private: \
        class Dummy##PREFIX##NAME { \
        public: \
            Dummy##PREFIX##NAME() {parameters_.insert(ParametersPair(#PREFIX "/" #NAME, DEFAULT_VALUE)); \
                                   parametersType_.insert(ParametersPair(#PREFIX "/" #NAME, "string")); \
                                   descriptions_.insert(ParametersPair(#PREFIX "/" #NAME, DESCRIPTION));} \
        }; \
        Dummy##PREFIX##NAME dummy##PREFIX##NAME
// end define PARAM

/**
 * Macro used to create parameter's key and default value.
 * This macro must be used only in the Parameters class definition (in this file).
 * They are automatically added to the default parameters map of the class Parameters.
 * Example:
 * @code
 *         //for PARAM(Video, ImageWidth, int, 640), the output will be :
 *         public:
 *             static std::string kVideoImageWidth() {return std::string("Video/ImageWidth");}
 *             static int defaultVideoImageWidth() {return 640;}
 *         private:
 *             class DummyVideoImageWidth {
 *             public:
 *                 DummyVideoImageWidth() {parameters_.insert(ParametersPair("Video/ImageWidth", "640"));}
 *             };
 *             DummyVideoImageWidth dummyVideoImageWidth;
 * @endcode
 */
#define VISFS_PARAM_COND(PREFIX, NAME, TYPE, COND, DEFAULT_VALUE1, DEFAULT_VALUE2, DESCRIPTION) \
    public: \
        static std::string k##PREFIX##NAME() {return std::string(#PREFIX "/" #NAME);} \
        static TYPE default##PREFIX##NAME() {return COND?DEFAULT_VALUE1:DEFAULT_VALUE2;} \
        static std::string type##PREFIX##NAME() {return std::string(#TYPE);} \
    private: \
        class Dummy##PREFIX##NAME { \
        public: \
            Dummy##PREFIX##NAME() {parameters_.insert(ParametersPair(#PREFIX "/" #NAME, COND?#DEFAULT_VALUE1:#DEFAULT_VALUE2)); \
                                   parametersType_.insert(ParametersPair(#PREFIX "/" #NAME, #TYPE)); \
                                   descriptions_.insert(ParametersPair(#PREFIX "/" #NAME, DESCRIPTION));} \
        }; \
        Dummy##PREFIX##NAME dummy##PREFIX##NAME
// end define PARAM


/**
 * Class Parameters.
 * This class is used to manage all custom parameters
 * we want in the application. It was designed to be very easy to add
 * a new parameter (just by adding one line of code).
 * The macro PARAM(PREFIX, NAME, TYPE, DEFAULT_VALUE) is
 * used to create a parameter in this class. A parameter can be accessed after by
 * Parameters::defaultPARAMETERNAME() for the default value, Parameters::kPARAMETERNAME for his key (parameter name).
 * The class provides also a general map containing all the parameter's key and
 * default value. This map can be accessed anywhere in the application by
 * Parameters::getDefaultParameters();
 * Example:
 * @code
 *         //Defining a parameter in this class with the macro PARAM:
 *         PARAM(Video, ImageWidth, int, 640);
 *
 *         // Now from anywhere in the application (Parameters is a singleton)
 *         int width = Parameters::defaultVideoImageWidth(); // theDefaultValue = 640
 *         std::string theKey = Parameters::kVideoImageWidth(); // theKey = "Video/ImageWidth"
 *         std::string strValue = Util::value(Parameters::getDefaultParameters(), theKey); // strValue = "640"
 * @endcode
 * @see getDefaultParameters()
 * TODO Add a detailed example with simple classes
 */

class Parameters {

public:
    virtual ~Parameters();

	/** \brief Get all default parameters. 
      * \return default parameters.
	  * \author eddy
      */
    static const ParametersMap & getDefaultParameters() { return parameters_; }

	/** \brief Get parameter's type. 
      * \param[in] Key of parameter. 
      * \return Parameter type.
	  * \author eddy
      */
    static std::string getType(const std::string & _paramKey);

	/** \brief Get parameter's description. 
      * \param[in] Key of parameter. 
      * \return Parameter description.
	  * \author eddy
      */
    static std::string getDescription(const std::string & _paramKey);


	/** \brief Parse bool type parameters from parameter map by key. 
      * \param[in] Parameter map.
      * \param[in] Key of parameter.
      * \param[out] Bool type value of parameter.
      * \return Ture: Find key and return the value, False: Not find the key.
	  * \author eddy
      */
    static bool parse(const ParametersMap & _parameters, const std::string & _key, bool & _value);

	/** \brief Parse int type parameters from parameter map by key. 
      * \param[in] Parameter map.
      * \param[in] Key of parameter.
      * \param[out] Int type value of parameter.
      * \return Ture: Find key and return the value, False: Not find the key.
	  * \author eddy
      */
    static bool parse(const ParametersMap & _parameters, const std::string & _key, int & _value);

	/** \brief Parse unsigned int type parameters from parameter map by key. 
      * \param[in] Parameter map.
      * \param[in] Key of parameter.
      * \param[out] Unsigned int type value of parameter.
      * \return Ture: Find key and return the value, False: Not find the key.
	  * \author eddy
      */
    static bool parse(const ParametersMap & _parameters, const std::string & _key, unsigned int & _value);

	/** \brief Parse float type parameters from parameter map by key. 
      * \param[in] Parameter map.
      * \param[in] Key of parameter.
      * \param[out] Float type value of parameter.
      * \return Ture: Find key and return the value, False: Not find the key.
	  * \author eddy
      */
    static bool parse(const ParametersMap & _parameters, const std::string & _key, float & _value);

	/** \brief Parse double type parameters from parameter map by key. 
      * \param[in] Parameter map.
      * \param[in] Key of parameter.
      * \param[out] Double type value of parameter.
      * \return Ture: Find key and return the value, False: Not find the key.
	  * \author eddy
      */
    static bool parse(const ParametersMap & _parameters, const std::string & _key, double & _value);

	/** \brief Parse string type parameters from parameter map by key. 
      * \param[in] Parameter map.
      * \param[in] Key of parameter.
      * \param[out] String type value of parameter.
      * \return Ture: Find key and return the value, False: Not find the key.
	  * \author eddy
      */
    static bool parse(const ParametersMap & _parameters, const std::string & _key, std::string & _value);

	/** \brief Parse ParametersMap type parameters from parameter map by key. 
      * \param[in] Parameter map.
      * \param[out] ParametersMap type value of parameter.
      * \return Ture: Find key and return the value, False: Not find the key.
	  * \author eddy
      */
    static bool parse(const ParametersMap & _parameters, ParametersMap & _parametersOut);

private:
    Parameters();

    static ParametersMap parameters_;
    static ParametersMap parametersType_;
    static ParametersMap descriptions_;
    static Parameters instance_;

    static std::map<std::string, std::pair<bool, std::string>> removedParameters_;
    static ParametersMap backwardCompatibilityMap_;
};

}   // namespace

#endif