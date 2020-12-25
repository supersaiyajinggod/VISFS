#include "Log.h"

namespace logging = boost::log;
namespace attrs = boost::log::attributes;
namespace src = boost::log::sources;
namespace sinks = boost::log::sinks;
namespace expr = boost::log::expressions;
namespace keywords = boost::log::keywords;

// The formatting logic for the severity level
template<typename CharT, typename TraitsT>
inline std::basic_ostream<CharT, TraitsT> & operator<< (std::basic_ostream<CharT, TraitsT> & strm, SeverityLevel lvl) {
    static const char* const str[] = {
        "DEBUG",
        "INFO",
        "WARNING",
        "ERROR",
        "FATAL"
    };

    if (static_cast<std::size_t>(lvl) < (sizeof(str) / sizeof(*str)))
        strm << str[lvl];
    else
        strm << static_cast<int>(lvl);
    return strm;
}

BOOST_LOG_GLOBAL_LOGGER_INIT(logger, src::severity_logger_mt<SeverityLevel>) {
    src::severity_logger_mt<SeverityLevel> lg;
    return lg;
}

Logger::Logger(SeverityLevel _level, bool _console, std::string _storeFolder) {
	// Create a text file sink
    typedef sinks::synchronous_sink<sinks::text_file_backend> file_sink;
    boost::shared_ptr<file_sink> fSink(new file_sink(
        keywords::target = "%Y%m%d_%H%M%S_%5N.log",		// file name pattern
        keywords::rotation_size = 50 * 1024 * 1024		// rotation size, in characters, 50M
    ));

    // Set up where the rotated files will be stored
    fSink->locked_backend()->set_file_collector(sinks::file::make_collector(
        keywords::target = _storeFolder,				// where to store rotated files
        keywords::max_size = 1000 * 1024 * 1024,		// maximum total size of the stored files, in bytes, 1G
        keywords::min_free_space = 1000 * 1024 * 1024,	// minimum free space on the drive, in bytes, 1G
        keywords::max_files = 10						// maximum number of stored files
    ));

    // Upon restart, scan the target directory for files matching the file_name pattern
    fSink->locked_backend()->scan_for_files();

    typedef sinks::synchronous_sink<sinks::text_ostream_backend> text_sink;
    boost::shared_ptr<text_sink> tSink(new text_sink);
	if (_console) {
		text_sink::locked_backend_ptr pBackend = tSink->locked_backend();
		// Next we add streams to which logging records should be output
		boost::shared_ptr<std::ostream> pStream(&std::clog, boost::null_deleter());
		pBackend->add_stream(pStream);

		tSink->set_formatter(expr::stream
			<< "[" << expr::attr<SeverityLevel>("Severity")
			// << "] [" << expr::format_date_time<attrs::timer::value_type>("Uptime", "%O:%M:%S")
			<< "] [" << expr::format_date_time<boost::posix_time::ptime>("TimeStamp", "%Y-%m-%d,%H:%M:%S.%f")
			<< "] " << expr::message
		);

		logging::core::get()->add_sink(tSink);       
	}

	fSink->set_formatter(expr::stream
		<< "[" << expr::attr<SeverityLevel>("Severity")
		// << "] [" << expr::format_date_time<attrs::timer::value_type>("Uptime", "%O:%M:%S")
		<< "] [" << expr::format_date_time<boost::posix_time::ptime>("TimeStamp", "%Y-%m-%d,%H:%M:%S.%f")
		<< "] " << expr::message	
	);

	logging::core::get()->add_sink(fSink);

	logging::core::get()->add_global_attribute("TimeStamp", attrs::local_clock());
	logging::core::get()->set_filter(
		expr::attr<SeverityLevel>("Severity").or_default(Info) >= _level
	);

}