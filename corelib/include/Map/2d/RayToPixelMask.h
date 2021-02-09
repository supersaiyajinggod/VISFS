#ifndef VISFS_MAP_2D_RAYTOPIXEL_MASK_H_
#define VISFS_MAP_2D_RAYTOPIXEL_MASK_H_

#include <vector>
#include <Eigen/Core>

namespace VISFS {
namespace Map {

/** \brief Compute all pixels that contain some part of the line segement connecting 'begin' and 'end'.
 *          'scaledBegin' and 'scaledEnd' are scaled by 'subpixelScale'. 'scaledBegin' and 'scaledEnd'
 *           are expected to be greater than zero.
 * \param[in] scaledBegin The scaled begin point.
 * \param[in] scaledEnd The scaled end point.
 * \param[in] subpixelScale The coefficient of scale.
 * \return The values are in pixels and not scaled.
 * \author eddy
 */
std::vector<Eigen::Array2i> rayToPixelMask(const Eigen::Array2i & _scaledBegin, const Eigen::Array2i & _scaledEnd, int subpixelScale);

}   // namespace Map 
}   // namespace VISFS

#endif  // VISFS_MAP_2D_RAYTOPIXEL_MASK_H_