// Copyright (c) 2026 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

// Dead / bad pixel detection shared by the calibration tooling.
//
// Detection is deliberately conservative, parameter-free, and stable run-to-run.
// Over a capture in which the camera is panned across a varied scene a pixel is
// flagged dead only if it is stuck relative to its neighbours — its temporal
// stddev is a tiny fraction of the LOCAL median stddev, and that local
// neighbourhood actually saw motion. Comparing against the local (not global)
// activity makes the result independent of how thoroughly each region was
// panned: an isolated quiet pixel among moving neighbours is stuck; a uniformly
// quiet region (not panned) is left alone rather than guessed at. Pixels in such
// regions cannot be judged and are reported as `unjudged` so the caller can warn
// that those areas were not actually checked.
//
// Pixels that read exactly 0x0000 / 0xFFFF are NOT added to the mask: the driver
// already replaces them with a 3x3 gaussian over valid neighbours on every frame
// (seek_thermal_camera.cpp::extractFrame), so masking them is redundant and, for
// marginal pixels that clip intermittently, makes the result unstable. Their
// count is reported for information only.
//
// Statistics are gathered from RAW frames (before that runtime inpaint) so that
// sentinel and stuck pixels are actually visible to the detector.

#ifndef OPENSEEKTHERMAL_TOOLS_DETECT_DEAD_PIXELS_HPP
#define OPENSEEKTHERMAL_TOOLS_DETECT_DEAD_PIXELS_HPP

#include <cstddef>
#include <cstdint>
#include <vector>

namespace openseekthermal::tools
{

//! Per-pixel temporal statistics over RAW thermal frames. Sentinel readings
//! (0x0000 / 0xFFFF) are excluded from mean/stddev and counted separately.
struct RawPixelStats {
  std::vector<double> mean;        //!< mean over non-sentinel samples
  std::vector<double> stddev;      //!< stddev over non-sentinel samples
  std::vector<int> valid_count;    //!< number of non-sentinel samples
  std::vector<int> sentinel_count; //!< number of 0x0000 / 0xFFFF samples
  int n = 0;                       //!< thermal frames captured
};

struct DeadPixelResult {
  std::vector<uint8_t> mask;     //!< width*height, 255 = dead, 0 = good (stuck pixels only)
  size_t total_dead = 0;         //!< pixels in the mask (== stuck pixels)
  size_t sentinel_info = 0;      //!< read 0/0xFFFF most frames; driver-inpainted, NOT masked
  size_t unjudged = 0;           //!< usable pixels whose neighbourhood saw too little motion to
                                 //!< evaluate the stuck test — NOT checked, coverage gap
  size_t judged = 0;             //!< usable pixels that were actually evaluated for stuckness
  double panning_activity = 0.0; //!< median local stddev (panning quality metric)
  double min_activity = 0.0;     //!< local activity below which a region is "not panned"
};

//! Flag dead pixels from accumulated raw statistics. No tunable parameters.
DeadPixelResult detectDeadPixels( const RawPixelStats &stats, int width, int height );

} // namespace openseekthermal::tools

#endif // OPENSEEKTHERMAL_TOOLS_DETECT_DEAD_PIXELS_HPP
