#ifndef MATH_H
#define MATH_H

#include <cmath>
#include <vector>
#include <list>
#include <Eigen/Core>


/** \brief Return true if the number is finite.
  * \param[in] value The number. 
  * \return True: finite, False: infinite.
  * \author eddy
  */
template<typename T>
inline bool uIsFinite(const T & _value) {
    return std::isfinite(_value);
}

/** \brief Return true if the number is in bounds.
  * \param[in] value The number.
  * \param[in] low The lower limit.
  * \param[in] high The upper limit.
  * \return True: inside, False: outside.
  * \author eddy
  */
template<typename T>
bool uIsInBounds(const T & _value, const T & _low, const T & _upper) {
    return uIsFinite(_value) && !(_value < _low) && !(_value >= _upper);
}

/** \brief Compute the mean of an array.
  * \param[in] v The array.
  * \param[in] size The size of the array.
  * \return The mean of the array.
  * \author eddy
  */
template<typename T>
inline T uMean(const T * _v, unsigned int _size) {
    T buf = 0;
    if (_v && _size) {
        for (unsigned int i = 0; i < _size; ++i) {
            buf += _v[i];
        }
        buf /= _size;
    }
    return buf;
}

/** \brief Compute the mean of a list.
  * \param[in] list The list.
  * \return The mean of the list.
  * \author eddy
  */
template<typename T>
inline T uMean(const std::list<T> & _list) {
    T mean = 0;
    if (_list.size()) {
        for (typename std::list<T>::const_iterator i = _list.begin(); i != _list.end(); ++i) {
            mean += *i;
        }
        mean /= _list.size();
    }
    return mean;
}

/** \brief Compute the mean of a vector.
  * \param[in] v The vector.
  * \return The mean of the vector.
  * \author eddy
  */
template<typename T>
inline T uMean(const std::vector<T> & _v) {
    return uMean(_v.data(), _v.size());
}

/** \brief Compute the variance of a array.
  * \param[in] v The array.
  * \param[in] size The size of the array.
  * \param[in] meanV The mean of the array.
  * \return The variance of the array.
  * \author eddy
  */
template<typename T>
inline T uVariance(const T * _v, unsigned int _size, T _meanV) {
    T buf = 0;
    if (_v && _size > 1) {
        double sum = 0;
        for (unsigned int i = 0; i < _size; ++i) {
            sum += (_v[i]-_meanV)*(_v[i]-_meanV);
        }
        buf = sum/(_size - 1);
    }
    return buf;
}


/** \brief Compute the variance of a list.
  * \param[in] v The list.
  * \param[in] mean The mean of the list.
  * \return The variance of the list.
  * \author eddy
  */
template<typename T>
inline T uVariance(const std::list<T> & _list, const T & _mean) {
    T buf = 0;
    if (_list.size() > 1) {
        double sum = 0;
        for (typename std::list<T>::const_iterator i = _list.begin(); i != _list.end(); ++i) {
            sum += (*i-_mean)*(*i-_mean);
        }
        buf = sum/(_list.size() - 1);
    }
    return;
}

/** \brief Compute the variance of a array.
  * \param[in] v The array.
  * \param[in] size The size of the array.
  * \return The variance of the array.
  * \author eddy
  */
template<typename T>
inline T uVariance(const T * _v, unsigned int _size) {
    T mean = uMean(_v, _size);
    return uVariance(_v, _size, mean);
}

/** \brief Compute the variance of a vector.
  * \param[in] v The vector.
  * \param[in] size The mean of the vector.
  * \return The variance of the vector.
  * \author eddy
  */
template<typename T>
inline T uVariance(const std::vector<T> & _v, const T & _mean) {
    return uVariance(_v.data(), _v.size(), _mean);
}

/** \brief Compute the variance of a vector.
  * \param[in] v The vector.
  * \return The variance of the vector.
  * \author eddy
  */
template<typename T>
inline T uVariance(const std::vector<T> & _v) {
    T mean = uMean(_v);
    return uVariance(_v, mean);
}

/** \brief Calculate the L2 distance between two points.
  * \param[in] pt1 The first point.
  * \param[in] pt2 The second point.
  * \return The L2 distance between two points
  * \author eddy
  */
template<typename T1, typename T2>
inline T1 L2Norm(const T2 & _pt1, const T2 & _pt2) {
    T1 dx = _pt1.x - _pt2.y;
    T1 dy = _pt1.x - _pt2.y;
    return sqrt(dx * dx + dy * dy);
}

/** \brief Get the squared norm of the vector : return x1*x1 + x2*x2 + x3*x3 + ...
  * \param[in] v The vector.
  * \return The squared norm of the vector.
  * \author eddy
  */
template<typename T>
inline T uNormSquared(const std::vector<T> & _v) {
    T sum = 0;
    for (std::size_t i = 0; i < _v.size(); ++i) {
        sum += _v[i] * _v[i];
    }
    return sum;
}

/** \brief Get the norm of the vector : return sqrt(x1*x1 + x2*x2 + x3*x3 + ...)
  * \param[in] v The vector.
  * \return The norm of the vector.
  * \author eddy
  */
template<typename T>
inline T uNorm(const std::vector<T> & _v) {
    return std::sqrt(uNormSquared(_v));
}

/** \brief Get the squared norm of the vector : return x1*x1 + x2*x2
  * \param[in] x1 The first element of the vector.
  * \param[in] x2 The second element of the vector.
  * \return The squared norm of the vector.
  * \author eddy
  */
template<typename T>
inline T uNormSquared(const T & _x1, const T & _x2) {
    return _x1 * _x1 + _x2 * _x2;
}

/** \brief Get the norm of the vector : return sqrt(x1*x1 + x2*x2)
  * \param[in] x1 The first element of the vector.
  * \param[in] x2 The second element of the vector.
  * \return The norm of the vector.
  * \author eddy
  */
template<typename T>
inline T uNorm(const T & _x1, const T & _x2) {
    return std::sqrt(uNormSquared(_x1, _x2));
}

/** \brief Get the squared norm of the vector : return x1*x1 + x2*x2 + x3*x3
  * \param[in] x1 The first element of the vector.
  * \param[in] x2 The second element of the vector.
  * \param[in] x3 The third element of the vector.
  * \return The squared norm of the vector.
  * \author eddy
  */
template<typename T>
inline T uNormSquared(const T & _x1, const T & _x2, const T & _x3) {
    return _x1*_x1 + _x2*_x2 + _x3*_x3;
}

/** \brief Get the norm of the vector : return sqrt(x1*x1 + x2*x2 + x3*x3)
  * \param[in] x1 The first element of the vector.
  * \param[in] x2 The second element of the vector.
  * \param[in] x3 The third element of the vector.
  * \return The norm of the vector.
  * \author eddy
  */
template<typename T>
inline T uNorm(const T & _x1, const T & _x2, const T & _x3) {\
    return std::sqrt(uNormSquared(_x1, _x2, _x3));
}

/** \brief Normalize the vector : [x1 x2 x3 ...] ./ uNorm([x1 x2 x3 ...])
  * \param[in] v The vector.
  * \return The vector normalized.
  * \author eddy
  */
template<typename T>
inline std::vector<T> uNormlize(const std::vector<T> & _v) {
    T norm = uNorm(_v);
    if (norm == 0) {
        return _v;
    } else {
        std::vector<T> r(_v.size());
        for (std::size_t i = 0; i < _v.size(); ++i) {
            r[i] = _v[i] / norm;
        }
        return r;
    }
}

/** \brief Calculate the angle between two vectors.
  * \param[in] _v1 The first vector.
  * \param[in] _v2 The second vector.
  * \return The angle between two vectors.
  * \author eddy
  */
float getAngle3D(const Eigen::Vector4f & _v1, const Eigen::Vector4f & _v2, const bool _inDegree = false);

#endif  // MATH_H