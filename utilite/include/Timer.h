#ifndef TIMER_H
#define TIMER_H

#include <string>
#include <boost/date_time/posix_time/posix_time.hpp>

/** \brief This class is a tool of time.
 *  \code
 *  Utimer timer;
 *  timer.start();
 *  // do some work
 *  timer.stop();
 *  double time = timer.now();
 *  \endcode
 *  \author eddy
 */

class UTimer {
public:
    UTimer();
    ~UTimer();

    static double now();
    static boost::posix_time::ptime now(int i = 1);

    void start();
    void stop();
    void restart();

    boost::posix_time::millisec_posix_time_system_config::time_duration_type elapsed();
    boost::posix_time::millisec_posix_time_system_config::time_duration_type elapsed(const std::string & _string);

private:
    boost::posix_time::ptime timeStart_;
    boost::posix_time::ptime timeStop_;
};

#endif  // TIMER_H