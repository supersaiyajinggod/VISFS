#ifndef _MEMORY_
#define _MEMORY_

#include <stdlib.h>

template <typename T>
T ** createDynamicArray2D(int _row, int _col) {
    T ** p;
    p = (T **) malloc(_row * sizeof(T *));
    for (int i = 0; i < _row; ++i) {
        p[i] = (T *) malloc(_col * sizeof(T));
    }
    return p;
}

template <typename T>
void realeaseDynamicArray2D(T ** _p, int _row) {
    for (int i = 0; i < _row; ++i) {
        free(_p[i]);
    }
    free(_p);
}

#endif