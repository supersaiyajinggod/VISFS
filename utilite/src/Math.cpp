#include "Math.h"

float getAngle3D (const Eigen::Vector4f & _v1, const Eigen::Vector4f & _v2, const bool _inDegree) {
  // Compute the actual angle
  float rad = _v1.normalized ().dot(_v2.normalized());
  if (rad < -1.0)
    rad = -1.0;
  else if (rad >  1.0)
    rad = 1.0;
  return (_inDegree ? acos(rad) * 180.0 / M_PI : acos(rad));
}