#ifndef MATH_H
#define MATH_H

#include <cmath>
#include <vector>
#include <list>
#include <Eigen/Core>
#include <Eigen/Geometry>


/** \brief Clamps 'value' to be in the range [_min, _max]
  * \param[in] value The number.
  * \param[in] min The minimums number can achieve.
  * \param[in] max The maximums number can achieve.
  * \return The clamped number.
  * \author eddy
  */
template <typename T>
T uClamp(const T _value, const T _min, const T _max) {
  if (_value > _max) {
    return _max;
  }
  if (_value < _min) {
    return _min;
  }
  return _value;
}

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
    T1 dx = _pt1.x - _pt2.x;
    T1 dy = _pt1.y - _pt2.y;
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

/** \brief Convert a small rotation at omega angular velocity to a quaternion.
  * \param[in] omega The angular velocity.
  * \return The quaternion.
  * \author eddy
  */
template <typename Derived>
Eigen::Quaternion<typename Derived::Scalar> deltaQ(const Eigen::MatrixBase<Derived> & _omega) {
	Eigen::Quaternion<Derived::Scalar> dq;
	Eigen::Matrix<Derived::Scalar, 3, 1> halfTheta = _omega;
	halfTheta /= static_cast<typename Derived::Scalar>(2.0);
	dq.w() = static_cast<typename Derived::Scalar>(1.0);
	dq.x() = halfTheta.x();
	dq.y() = halfTheta.y();
	dq.z() = halfTheta.z();
	return dq;
}

/** \brief Convert a 3 dimension vector to skew symmetric matrix.
  * \param[in] q The 3 dimension vector.
  * \return The skew symmetric matrix.
  * \author eddy
  */
template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> skewSymmetric(const Eigen::MatrixBase<Derived> & _q) {
	Eigen::Matrix<typename Derived::Scalar, 3, 3> ans;
	ans << typename Derived::Scalar(0), -_q(2), _q(1),
		_q(2), typename Derived::Scalar(0), -_q(0),
		-_q(1), _q(0), typename Derived::Scalar(0);
	return ans;
}

/** \brief Check a quaternion is positive.
  * \param[in] q A quaternion.
  * \return A positive quaternion.
  * \author eddy
  */
template <typename Derived>
Eigen::Quaternion<typename Derived::Scalar> QuaternionPositify(const Eigen::QuaternionBase<Derived> & _q) {
  	Eigen::Quaternion<typename Derived::Scalar> q;
  	q = _q;
	if (q.w() < 0) {
		q.coeffs() *= -1;
	}
	q.normalize();
	return q;
}

/** \brief Calculate left multiplication operator of quaternion.
  * \param[in] q The quaternion.
  * \return The left multiplication operator.
  * \author eddy
  */
template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 4, 4> QuaternionLeft(const Eigen::QuaternionBase<Derived> & _q) {
	Eigen::Quaternion<typename Derived::Scalar> pq = QuaternionPositify(_q);
	Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
	ans(0, 0) = pq.w(), ans.template block<1, 3>(0, 1) = -pq.vec().transpose();
	ans.template block<3, 1>(1, 0) = pq.vec(), ans.template block<3, 3>(1, 1) = pq.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() + skewSymmetric(pq.vec());
	return ans;
}

/** \brief Calculate right multiplication operator of quaternion.
  * \param[in] q The quaternion.
  * \return The right multiplication operator.
  * \author eddy
  */
template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 4, 4> QuaternionRight(const Eigen::QuaternionBase<Derived> & _q) {
	Eigen::Quaternion<typename Derived::Scalar> pq = QuaternionPositify(_q);
	Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
	ans(0, 0) = pq.w(), ans.template block<1, 3>(0, 1) = -pq.vec().transpose();
	ans.template block<3, 1>(1, 0) = pq.vec(), ans.template block<3, 3>(1, 1) = pq.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() - skewSymmetric(pq.vec());
	return ans;
}

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> expSO3(const Eigen::MatrixBase<Derived> & w) {
	const typename Derived::Scalar d2 = w[0]*w[0] + w[1]*w[1] + w[2]*w[2];
	const typename Derived::Scalar d = sqrt(d2);
	
	Eigen::Matrix<typename Derived::Scalar, 3, 3> W, ans;
	W << static_cast<typename Derived::Scalar>(0.0), -w[2], w[1], w[2], static_cast<typename Derived::Scalar>(0.0),
			-w[0], -w[1], w[0], static_cast<typename Derived::Scalar>(0.0);
	
	Eigen::Matrix<typename Derived::Scalar, 3, 3> identity;
	identity << static_cast<typename Derived::Scalar>(1.0), static_cast<typename Derived::Scalar>(0.0), static_cast<typename Derived::Scalar>(0.0), static_cast<typename Derived::Scalar>(0.0), static_cast<typename Derived::Scalar>(1.0),
			static_cast<typename Derived::Scalar>(0.0), static_cast<typename Derived::Scalar>(0.0), static_cast<typename Derived::Scalar>(0.0), static_cast<typename Derived::Scalar>(1.0);

	if (d < static_cast<typename Derived::Scalar>(1e-5)) {
		Eigen::Matrix<typename Derived::Scalar, 3, 3> res = identity + W + 0.5*W*W;
		const Eigen::Quaternion<typename Derived::Scalar> q = Eigen::Quaternion<typename Derived::Scalar>(res);
		return q.normalized().toRotationMatrix();
	} else {
		Eigen::Matrix<typename Derived::Scalar, 3, 3> res = identity + W*sin(d)/d + W*W*(static_cast<typename Derived::Scalar>(1.0)-cos(d))/d2;
		const Eigen::Quaternion<typename Derived::Scalar> q = Eigen::Quaternion<typename Derived::Scalar>(res);
		return q.normalized().toRotationMatrix();
	}
}

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 1> logSO3(const Eigen::Matrix<typename Derived::Scalar, 3, 3> & R) {
	const typename Derived::Scalar tr = R(0, 0) + R(1, 1) + R(2, 2);
	Eigen::Matrix<typename Derived::Scalar, 3, 1> w;
	w << (R(2, 1)-R(1, 2))/2, (R(0,2)-R(2,0))/2, (R(1,0)-R(0,1))/2;
	const typename Derived::Scalar costheta = (tr - static_cast<typename Derived::Scalar>(1.0))*static_cast<typename Derived::Scalar>(0.5);
	if (costheta > 1 || costheta < -1)
		return w;
	const typename Derived::Scalar theta = acos(costheta);
	const typename Derived::Scalar s = sin(theta);
	if (fabs(s) < static_cast<typename Derived::Scalar>(1e-5)) {
		return w;
	} else {
		return theta*w/s;
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