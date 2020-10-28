#include "Monitor.h"
#include "Stl.h"

namespace VISFS {

Monitor::Monitor(const ParametersMap & _parameters) :
    sensorStrategy_(Parameters::defaultSystemSensorStrategy()) {
    
    Parameters::parse(_parameters, Parameters::kSystemSensorStrategy(), sensorStrategy_);
}

void Monitor::addSignature(const Signature & _signature) {
    boost::lock_guard<boost::mutex> lock(mutexDataRW_);
    signatureBuf_.emplace(_signature);
}

void Monitor::threadProcess() {
    while (1) {
        Signature signature;

        if (!signatureBuf_.empty()) {
            {
                boost::lock_guard<boost::mutex> lock(mutexDataRW_);
                signature = signatureBuf_.front();
                signatureBuf_.pop();
            }

            process(signature);
        }

        boost::this_thread::sleep(boost::get_system_time() + boost::posix_time::milliseconds(5));
    }
}

void Monitor::process(Signature & _signature) {
    if (sensorStrategy_ == 0 || sensorStrategy_ == 2) {                 // Stereo && Stereo + wheel
        cv::Mat imageLeft = _signature.getImageLeft();
        cv::Mat imageRight = _signature.getImageRight();
        std::vector<cv::KeyPoint> kptsMatchesFormer = uValues(_signature.getKeyPointsMatchesFormer());
        std::vector<cv::KeyPoint> kptsNewExtract = uValues(_signature.getKeyPointsNewExtract());
        std::map<std::size_t, cv::KeyPoint> kptsMatchesLeftToRight = _signature.getKeyPointsMatchesImageRight();
        std::map<std::size_t, cv::Point3f> words3d = _signature.getWords3d();

        cv::Mat stitch;
        cv::hconcat(imageLeft, imageRight, stitch);
        
        const float cols = static_cast<float>(imageLeft.cols);
        for (auto kpt : kptsMatchesFormer) {
            cv::circle(stitch, kpt.pt, 2, cv::Scalar(0, 0, 255), 1);
        } 
        for (auto kpt : kptsNewExtract) {
            cv::circle(stitch, kpt.pt, 2, cv::Scalar(255, 0, 0), 1);
        }
        assert(kptsMatchesLeftToRight.size() == words3d.size());
        for (auto iter = kptsMatchesLeftToRight.begin(); iter != kptsMatchesLeftToRight.end(); ++iter) {
            auto id = iter->first;
            iter->second.pt.x += cols;
            cv::circle(stitch, iter->second.pt, 2, cv::Scalar(176, 48, 96), 1);
            auto jter = words3d.find(id);
            if (jter != words3d.end()) {
                if (std::isfinite(jter->second.z)) {
                    std::stringstream ss;
                    ss.precision(4);
                    std::string text;
                    ss << jter->second.x;
                    ss >> text;
                    cv::putText(stitch, text, iter->second.pt, cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0, 255, 0), 2);
                }
            }
        }

        cv::namedWindow("Monitor", CV_WINDOW_NORMAL);
        cv::imshow("Monitor", stitch);
        cv::waitKey(5);

    } else if (sensorStrategy_ == 1) {          // RGBD

    }
}

}   // namespace