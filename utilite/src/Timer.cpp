#include <iostream>

#include "Timer.h"


UTimer::UTimer() {
    start();
}

UTimer::~UTimer() {}

void UTimer::start() {
    timeStart_ = boost::posix_time::microsec_clock::universal_time();
    timeStop_ = timeStart_;
}

void UTimer::stop() {
    timeStop_ = boost::posix_time::microsec_clock::universal_time();
}

void UTimer::restart() {
    timeStart_ = boost::posix_time::microsec_clock::universal_time();
    timeStop_ = timeStart_;    
}

boost::posix_time::millisec_posix_time_system_config::time_duration_type UTimer::elapsed() {
    boost::posix_time::ptime now = boost::posix_time::microsec_clock::universal_time();
    boost::posix_time::millisec_posix_time_system_config::time_duration_type duration = now - timeStart_;
    return duration;
}

boost::posix_time::millisec_posix_time_system_config::time_duration_type UTimer::elapsed(const std::string & _string) {
    boost::posix_time::ptime now = boost::posix_time::microsec_clock::universal_time();
    boost::posix_time::millisec_posix_time_system_config::time_duration_type duration = now - timeStart_;
    std::cout << _string << "  " << duration << std::endl;
    return duration;    
}

static double UTimer::now() {
    boost::posix_time::ptime now = boost::posix_time::microsec_clock::universal_time();
    // boost::posix_time::ptime now(boost::posix_time::microsec_clock::local_time());
    boost::posix_time::ptime const EPOCH(boost::gregorian::date(1970,1,1));
    boost::posix_time::time_duration delta(now - EPOCH);
    return (delta.total_microseconds() / 1000000.0);
}

static boost::posix_time::ptime UTimer::now(int i = 1) {
    boost::posix_time::ptime now = boost::posix_time::microsec_clock::universal_time();
}

std::string getCurrentReadableTime() {
    // std::string strTime = boost::posix_time::to_iso_string(boost::posix_time::second_clock::local_time());
    // int pos = strTime.find('T');
    // strTime.replace(pos, 1, std::string("-"));
    // strTime.replace(pos + 3, 0, std::string(":"));
    // strTime.replace(pos + 6, 0, std::string(":"));
    std::string strTime = boost::posix_time::to_iso_extended_string(boost::posix_time::second_clock::local_time());
    return strTime;
}


