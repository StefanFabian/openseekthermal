// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef OPENSEEKTHERMAL_DEAD_PIXEL_MASK_HPP
#define OPENSEEKTHERMAL_DEAD_PIXEL_MASK_HPP

#include <cstddef>
#include <cstdint>
#include <vector>

namespace openseekthermal
{

/*!
 * One dead pixel together with the precomputed gauss-weighted neighbours used
 * to inpaint it. Neighbours that are themselves dead are pre-excluded so that
 * the per-frame inner loop is a flat weighted sum.
 */
struct DeadPixelEntry {
  uint32_t index = 0;            //!< y * width + x of the dead pixel
  uint8_t neighbor_count = 0;    //!< number of valid neighbours (0..8)
  uint32_t neighbors[8] = { 0 }; //!< flat indices of non-dead 3x3 neighbours
  uint8_t weights[8] = { 0 };    //!< matching gauss weights
};

/*!
 * Sparse representation of a dead-pixel mask. Built once per camera and
 * applied per-frame; touches only the dead pixels themselves.
 *
 * Loaded as part of a CameraCalibration; see camera_calibration.hpp.
 */
class DeadPixelMask
{
public:
  DeadPixelMask() = default;

  /*!
   * Build from a width*height boolean mask. Non-zero entry = dead.
   * Precomputes neighbour lists (excluding any neighbour also flagged dead).
   * @throws std::invalid_argument if mask.size() != width*height.
   */
  DeadPixelMask( int width, int height, const std::vector<bool> &mask );

  /*!
   * Build from a list of (x, y) dead-pixel coordinates.
   * @throws std::out_of_range if any coordinate is outside [0, width) × [0, height).
   */
  DeadPixelMask( int width, int height, const std::vector<std::pair<int, int>> &dead_coords );

  /*!
   * Apply the inpainting in-place to a width*height host-endian uint16
   * frame buffer. No-op for entries with zero valid neighbours.
   */
  void apply( uint16_t *frame ) const;

  int width() const { return width_; }

  int height() const { return height_; }

  size_t deadPixelCount() const { return entries_.size(); }

  const std::vector<DeadPixelEntry> &entries() const { return entries_; }

private:
  int width_ = 0;
  int height_ = 0;
  std::vector<DeadPixelEntry> entries_;
};

} // namespace openseekthermal

#endif // OPENSEEKTHERMAL_DEAD_PIXEL_MASK_HPP
