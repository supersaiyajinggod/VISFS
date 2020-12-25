#ifndef LOG_H
#define LOG_H

#define BOOST_LOG_DYN_LINK

#include <iostream>
#include <fstream>
#include <string>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/core/null_deleter.hpp>
#include <boost/log/common.hpp>
#include <boost/log/exceptions.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/attributes.hpp>
#include <boost/log/sinks.hpp>
#include <boost/log/sources/logger.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/utility/manipulators/add_value.hpp>
#include <boost/log/utility/formatting_ostream.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/console.hpp>
#include <boost/log/attributes/timer.hpp>
#include <boost/log/support/date_time.hpp>

enum SeverityLevel {
    Debug,
    Info,
    Warning,
    Error,
    Fatal
};

// TODO: Set different channels, to write different module logs to different files.
BOOST_LOG_GLOBAL_LOGGER(logger, boost::log::sources::severity_logger_mt<SeverityLevel>)

#define LOG(level)	BOOST_LOG_SEV(logger::get(), level)
#define LOG_DEBUG	LOG(Debug)
#define LOG_INFO	LOG(Info)
#define LOG_WARN	LOG(Warning)
#define LOG_ERROR	LOG(Error)
#define LOG_FATAL	LOG(Fatal)

class Logger {
public:
	Logger(SeverityLevel _level, bool _console = false, std::string  _storeFolder = "logs");

private:
	SeverityLevel level_;
	bool console_;
	std::string storeFloder_;
};

#endif  //LOG_H