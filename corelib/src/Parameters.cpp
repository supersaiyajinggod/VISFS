#include "Parameters.h"
#include "Conversion.h"

namespace VISFS {

ParametersMap Parameters::parameters_;
ParametersMap Parameters::parametersType_;
ParametersMap Parameters::descriptions_;
Parameters Parameters::instance_;
std::map<std::string, std::pair<bool, std::string> > Parameters::removedParameters_;
ParametersMap Parameters::backwardCompatibilityMap_;

Parameters::Parameters() {}

Parameters::~Parameters() {}

std::string Parameters::getType(const std::string & _paramKey) {
    std::string type;
    ParametersMap::iterator iter = parametersType_.find(_paramKey);
    if (iter != parametersType_.end()) {
        type = iter->second;
    } else {
        std::cout << "Parameters " << _paramKey << " doesn't exist!" << std::endl;
    }
    return type;
}

std::string Parameters::getDescription(const std::string & _paramKey) {
    std::string description;
    ParametersMap::iterator iter = descriptions_.find(_paramKey);
    if (iter != descriptions_.end()) {
        description = iter->second;
    } else {
        std::cout << "Parameters " << _paramKey << " doesn't exist!" << std::endl;
    }
    return description;
}

bool Parameters::parse(const ParametersMap & _parameters, const std::string & _key, bool & _value) {
    ParametersMap::const_iterator iter = _parameters.find(_key);
    if (iter != _parameters.end()) {
        _value = uStr2Bool(iter->second.c_str());
        return true;
    }
    return false;
}

bool Parameters::parse(const ParametersMap & _parameters, const std::string & _key, int & _value) {
    ParametersMap::const_iterator iter = _parameters.find(_key);
    if (iter != _parameters.end()) {
        _value = uStr2Int(iter->second.c_str());
        return true;
    }
    return false;
}

bool Parameters::parse(const ParametersMap & _parameters, const std::string & _key, unsigned int & _value) {
    ParametersMap::const_iterator iter = _parameters.find(_key);
    if (iter != _parameters.end()) {
        _value = uStr2Int(iter->second.c_str());
        return true;
    }
    return false;    
}

bool Parameters::parse(const ParametersMap & _parameters, const std::string & _key, float & _value) {
    ParametersMap::const_iterator iter = _parameters.find(_key);
    if (iter != _parameters.end()) {
        _value = uStr2Float(iter->second.c_str());
        return true;
    }
    return false;       
}

bool Parameters::parse(const ParametersMap & _parameters, const std::string & _key, double & _value) {
    ParametersMap::const_iterator iter = _parameters.find(_key);
    if (iter != _parameters.end()) {
        _value = uStr2Double(iter->second.c_str());
        return true;
    }
    return false;  	
}

bool Parameters::parse(const ParametersMap & _parameters, const std::string & _key, std::string & _value) {
    ParametersMap::const_iterator iter = _parameters.find(_key);
    if (iter != _parameters.end()) {
        _value = iter->second;
        return true;
    }
    return false;  		
}

bool Parameters::parse(const ParametersMap & _parameters, ParametersMap & _parametersOut) {
	for (ParametersMap::iterator iter = _parametersOut.begin(); iter != _parametersOut.end(); ++iter) {
		ParametersMap::const_iterator jter = _parameters.find(iter->first);
		if (jter != _parameters.end()) {
			iter->second = jter->second;
		}
	}
}

}   // namespace