// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef OPENSEEKTHERMAL_DEAD_PIXEL_MASK_HPP
#define OPENSEEKTHERMAL_DEAD_PIXEL_MASK_HPP

#include <cstdint>
#include <filesystem>
#include <vector>

namespace openseekthermal
{

/*!
 * One dead pixel together with the precomputed gauss-weighted neighbours used
 * to inpaint it. Neighbours that are themselves dead are pre-excluded so that
 * the per-frame inner loop is a flat weighted sum.
 */
struct DeadPixelEntry {
  uint32_t index = 0;       //!< y * width + x of the dead pixel
  uint8_t neighbor_count = 0; //!< number of valid neighbours (0..8)
  uint32_t neighbors[8] = { 0 }; //!< flat indices of non-dead 3x3 neighbours
  uint8_t weights[8] = { 0 };    //!< matching gauss weights
};

/*!
 * Sparse representation of a dead-pixel mask. Built once per camera and
 * applied per-frame; touches only the dead pixels themselves.
 *
 * Use loadDeadPixelMaskPgm() to build from a PGM file produced by
 * the calibrate_dead_pixels tool.
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
   * Apply the inpainting in-place to a width*height little-endian uint16
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

/*!
 * Load an 8-bit P5 PGM dead-pixel mask written by calibrate_dead_pixels.
 * Convention: non-zero byte = dead.
 *
 * @param path Path to the PGM file.
 * @param expected_width Frame width the mask must match (use camera->getFrameWidth()).
 * @param expected_height Frame height the mask must match.
 * @throws std::runtime_error on I/O / format / dimension errors.
 */
DeadPixelMask loadDeadPixelMaskPgm( const std::filesystem::path &path, int expected_width,
                                    int expected_height );

} // namespace openseekthermal

#endif // OPENSEEKTHERMAL_DEAD_PIXEL_MASK_HPP
