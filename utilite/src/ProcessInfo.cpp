#include <fstream>
#include <stdlib.h>

#include "ProcessInfo.h"
#include "Stl.h"

UProcessInfo::UProcessInfo() {}

UProcessInfo::~UProcessInfo() {}

long int UProcessInfo::getMemoryUsage() {
    long int memoryUsage = -1;

    std::fstream file("/proc/self/status", std::fstream::in);
    if (file.is_open()) {
        std::string bytes;
        while (std::getline(file, bytes)) {
            if (bytes.find("VmRSS") != bytes.npos) {
                std::list<std::string> strs = uSplit(bytes, ' ');
                if (strs.size() > 1) {
                    memoryUsage = atol(uValueAt(strs, 1).c_str()) * 1024;
                }
                break;
            }
        }
        file.close();
    }

    return memoryUsage;
}

