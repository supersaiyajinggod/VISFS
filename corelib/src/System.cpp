#include "System.h"

namespace VISFS {

void System::inputStereoImage(const double time_, const cv::Mat & imageLeft_, const cv::Mat & imageRight_) {
    boost::lock_guard<boost::mutex> lock(mutexDataBuf_);
    stereoImageBuf_.emplace(std::make_tuple(time_, imageLeft_, imageRight_));
}

System::System() {

}

}   // namespace