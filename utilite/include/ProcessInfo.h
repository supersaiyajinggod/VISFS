#ifndef PROCESSINFO_H
#define PROCESSINFO_H

class UProcessInfo {
public:
    UProcessInfo();
    virtual ~UProcessInfo();

    /** \brief Get the memory used by the current process.
     *  \return The number of bytes used by the current process.
     *  \author eddy
     */
    static long int getMemoryUsage();    
};

#endif  // UPROCESSINFO