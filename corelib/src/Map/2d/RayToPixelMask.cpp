#include "Map/2d/RayToPixelMask.h"
#include <iostream>
namespace VISFS {
namespace Map {

namespace {

bool isEqual(const Eigen::Array2i & lhs, const Eigen::Array2i & rhs) {
    return ((lhs - rhs).matrix().lpNorm<1>() == 0);
}

}

std::vector<Eigen::Array2i> rayToPixelMask(const Eigen::Array2i & _scaledBegin, const Eigen::Array2i & _scaledEnd, int subpixelScale) {
    assert(_scaledBegin.x() >= 0);
    assert(_scaledBegin.y() >= 0);
    assert(_scaledEnd.y() >= 0);
    // For simplicity, we order sacledBegin and scaledEnd by their x coordinate.
    if (_scaledBegin.x() > _scaledEnd.x()) {
        return rayToPixelMask(_scaledEnd, _scaledBegin, subpixelScale);
    }

    std::vector<Eigen::Array2i> pixelMask;
    // Special case: vertical line.
    if (_scaledBegin.x() / subpixelScale == _scaledEnd.x() / subpixelScale) {
        Eigen::Array2i current(_scaledBegin.x() / subpixelScale, std::min(_scaledBegin.y(), _scaledEnd.y()) / subpixelScale);
        pixelMask.emplace_back(current);
        const int endY = std::max(_scaledBegin.y(), _scaledEnd.y()) / subpixelScale;
        for (; current.y() <= endY; ++current.y()) {
            if (!isEqual(pixelMask.back(), current))
                pixelMask.emplace_back(current);
        }
        return pixelMask;
    }

    const int64_t dx = _scaledEnd.x() - _scaledBegin.x();
    const int64_t dy = _scaledEnd.y() - _scaledBegin.y();
    const int64_t denominator = 2 * subpixelScale * dx;

    Eigen::Array2i current = _scaledBegin / subpixelScale;
    pixelMask.emplace_back(current);

    // To represent subpixel centers, we use a factor of 2 * 'subpixelScale' in
    // the denominator.
    // +-+-+-+ -- 1 = (2 * subpixelScale) / (2 * subpixelScale)
    // | | | |
    // +-+-+-+
    // | | | |
    // +-+-+-+ -- top edge of first subpixel = 2 / (2 * subpixelScale)
    // | | | | -- center of first subpixel = 1 / (2 * subpixelScale)
    // +-+-+-+ -- 0 = 0 / (2 * subpixelScale)

    // The center of the subpixel part of 'scaledEnd.y()' assuming the
    // 'denominator', i.e., subY / denominator is in (0, 1).

    int64_t subY = (2 * (_scaledBegin.y() % subpixelScale) + 1) * dx;
    const int firstPixel = 2 * subpixelScale - 2 * (_scaledBegin.x() % subpixelScale) - 1;
    const int lastPixel = 2 * (_scaledEnd.x() % subpixelScale) + 1;
    const int endX = std::max(_scaledBegin.x(), _scaledEnd.x()) / subpixelScale;

    subY += dy * firstPixel;
    if (dy > 0) {
        while (true) {
            if (!isEqual(pixelMask.back(), current)) pixelMask.emplace_back(current);
            while (subY > denominator) {
                subY -= denominator;
                ++current.y();
                if (!isEqual(pixelMask.back(), current)) pixelMask.emplace_back(current);
            }
            ++current.x();
            if (subY == denominator) {
                subY -= denominator;
                ++current.y();
            }
            if (current.x() == endX) {
                break;
            }
            // Move from one pixel border to the next.
            subY += dy * 2 * subpixelScale;
        }
        // Move from the pixel border on the right to scaledEnd.
        subY += dy * lastPixel;
        if (!isEqual(pixelMask.back(), current)) pixelMask.emplace_back(current);
        while (subY > denominator) {
            subY -= denominator;
            ++current.y();
            if (!isEqual(pixelMask.back(), current)) pixelMask.emplace_back(current);
        }
        assert(current.y() == _scaledEnd.y() / subpixelScale);
        return pixelMask;
    }

    // Same for lines non-ascending in y coordinates.
    while (true) {
        if (!isEqual(pixelMask.back(), current)) pixelMask.emplace_back(current);
        while (subY < 0) {
            subY += denominator;
            --current.y();
            if (!isEqual(pixelMask.back(), current)) pixelMask.emplace_back(current);
        }
        ++current.x();
        if (subY == 0) {
            subY += denominator;
            --current.y();
        }
        if (current.x() == endX) {
            break;
        }
        subY += dy * 2 * subpixelScale;
    }
    subY += dy * lastPixel;
    if (!isEqual(pixelMask.back(), current)) pixelMask.emplace_back(current);
    while (subY < 0) {
        subY += denominator;
        --current.y();
        if (!isEqual(pixelMask.back(), current)) pixelMask.emplace_back(current);
    }
    assert(current.y() == _scaledEnd.y() / subpixelScale);
    return pixelMask;
}

}   // namespace Map 
}   // namespace VISFS