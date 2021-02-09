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
    VISFS_PARAM(System,     SensorStrategy,         int,    0,      "System use sensors type: 0 stereo, 1 rgbd, 2 stereo + wheel.");
    VISFS_PARAM(System,     WheelOdometryFreq,      int,    100,    "The frequence of wheel odometry.");
    VISFS_PARAM(System,     Monitor,                bool,   false,  "Monitor");
	VISFS_PARAM(System,		CLAHE,					bool,	false,	"CLAHE");
    VISFS_PARAM(System,     LogLevel,               int,    1,      "0-DEBUG, 1-INFO, 2-WARN, 3-ERROR, 5-FATAL");
    VISFS_PARAM(System,     LogOnConsole,           bool,   false,  "Display the log on the console.");
    VISFS_PARAM_STR(System, LogFolder,              "~/.VISFS/logs","");

    VISFS_PARAM(Tracker,    MaxFeatures,            int,    300,    "The maximum number of key points will be generated.");
    VISFS_PARAM(Tracker,    QualityLevel,           double, 0.01,  "");
    VISFS_PARAM(Tracker,    MinDistance,            int,    40,     "");
    VISFS_PARAM(Tracker,    FlowBack,               bool,   true,   "Perform backward optical flow to improve feature tracking accuracy.");
    VISFS_PARAM(Tracker,    MaxDepth,               float,  10.0,   "Max depth of the features (0 means no limit).");
    VISFS_PARAM(Tracker,    MinDepth,               float,  0.2,    "Min depth of the features (0 means no limit).");
    VISFS_PARAM(Tracker,    FlowWinSize,            int,    21,     "Size of the search window at each pyramid level.");
    VISFS_PARAM(Tracker,    FlowIterations,         int,    30,     "Termination criteria of the max interation times.");
    VISFS_PARAM(Tracker,    FlowEps,                float,  0.01,   "Termination criteria of the search window moves by less than criteria.epsilon");
    VISFS_PARAM(Tracker,    FlowMaxLevel,           int,    3,      "Maximal pyramid level number; if set to 0, pyramids are not used (single level)");
    VISFS_PARAM(Tracker,    CullByFundationMatrix,  bool,   false,   "Use fundation matrix to cull out the outliers in the result of feature match.");
    VISFS_PARAM(Tracker,    FundationPixelError,    float,  1.0,    "Threshold of fundation matrix calculate error.");

    VISFS_PARAM(LocalMap,   MapSize,                int,    5,      "The size of Local map. The value means the quantity of signatures that we are estimating.");
    VISFS_PARAM(LocalMap,   MinParallax,            float,  60.0,   "Keysignature selection threshold (pixel).");
    VISFS_PARAM(LocalMap,   MinTranslation,         double, 0.5,    "Min distance condition to judge key signature.");

    VISFS_PARAM(Estimator,  MinInliers,             int,    12,     "Minimal inliers between two images.");
    VISFS_PARAM(Estimator,  PnPIterations,          int,    50,     "Maximal interation times in ransac.");
    VISFS_PARAM(Estimator,  PnPReprojError,         float,  2,      "PnP reprojection error.");
    VISFS_PARAM(Estimator,  PnPFlags,               int,    1,      "PnP flags: 0=Iterative, 1=EPNP, 2=P3P.");
    VISFS_PARAM(Estimator,  RefineIterations,       int,    5,      "Number of iterations used to refine the transformation found by RANSAC. 0 means that the transformation is not refined.");
    VISFS_PARAM(Estimator,  ToleranceTranslation,   double, 0.32,   "The max translation percentage difference between all sensors. The lower, we trust other sensor more.");
    VISFS_PARAM(Estimator,  ToleranceRotation,      double, 0.40,   "The max rotation percentage difference between all sensors. The lower, we trust other sensor more.");
    VISFS_PARAM(Estimator,  Force3DoF,              bool,   false,  "Force 3 degrees-of-freedom transform (3Dof: x,y and yaw). Parameters z, roll and pitch will be set to 0.");

    VISFS_PARAM(Optimizer,  Iterations,             int,      10,   "Optimization iterations.");
    VISFS_PARAM(Optimizer,  Solver,                 int,       0,   "0=csparse 1=cholmod 2=pcg 3=Eigen");
    VISFS_PARAM(Optimizer,  Optimizer,              int,       0,   "0=Levenberg 1=GaussNewton");
    VISFS_PARAM(Optimizer,  PixelVariance,          double,  1.5,   "Pixel variance used for bundle adjustment.");
    VISFS_PARAM(Optimizer,  RobustKernelDelta,      double,  8.0,   "Robust kernel delta used for bundle adjustment (0 means don't use robust kernel). Observations with chi2 over this threshold will be ignored in the second optimization pass.");

    VISFS_PARAM(Map,        2dNumRangeData,         int,    90,     "The limits used to insert range data into new submaps, when reaches the limits, the new map will use to scan-match, the old need to destory.");
    VISFS_PARAM(Map,        2dGridType,             int,     0,     "0=Probability map.");
    VISFS_PARAM(Map,        2dResolution,           double, 0.05,   "The resolution of the map");
    VISFS_PARAM(Map,        2dInsertFreeSpace,      bool,   true,   "Automatic insert the free status between origin and hit.");
    VISFS_PARAM(Map,        2dHitProbability,       double, 0.55,   "");
    VISFS_PARAM(Map,        2dMissProbability,      double, 0.49,   "");

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