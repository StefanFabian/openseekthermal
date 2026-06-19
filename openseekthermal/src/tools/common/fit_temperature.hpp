// Copyright (c) 2026 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

// Two-point (ice / boil) temperature calibration math shared by the tooling.
//
// Forward model: T_C = c0 + c1 * raw. Two known-temperature anchors pin the
// line. Per anchor we sample a centered ROI on each THERMAL frame and take a
// robust percentile (low end for the cold anchor, high end for the hot anchor)
// to bias toward unvignetted central pixels, then average across frames.

#ifndef OPENSEEKTHERMAL_TOOLS_FIT_TEMPERATURE_HPP
#define OPENSEEKTHERMAL_TOOLS_FIT_TEMPERATURE_HPP

#include <cstdint>

namespace openseekthermal::tools
{

struct TwoPointAnchor {
  double anchor_raw = 0.0;      //!< per-frame robust percentile, averaged across frames
  double anchor_raw_mean = 0.0; //!< per-frame plain mean (informational, for comparison)
  double percentile_used = 0.0;
  int thermal_used = 0;
};

/*!
 * Accumulate one THERMAL frame's ROI percentile + mean into running sums.
 *
 * @param pixels Start of the host-endian-by-reinterpret raw wire pixel data
 *        (i.e. raw transfer buffer + header bytes). Values are read with
 *        le16toh since this is little-endian wire data, not driver output.
 * @param row_step_pixels Padded row stride in pixels (device._getRowStep()/2).
 * @param is_hot Selects the (100-P)th percentile (hot) vs the Pth (cold).
 * @returns false (frame skipped) if the ROI had no valid pixels.
 */
bool accumulateAnchorFrame( const uint16_t *pixels, int row_step_pixels, int rx, int ry, int rw,
                            int rh, double robust_percentile, bool is_hot, double &sum_pct,
                            double &sum_mean, int &kept );

//! Finalize the running sums from accumulateAnchorFrame into an anchor.
TwoPointAnchor finalizeAnchor( double sum_pct, double sum_mean, int kept, double robust_percentile,
                               bool is_hot );

struct TwoPointFit {
  double c0 = 0.0;
  double c1 = 0.0;
};

/*!
 * Closed-form two-point solve. `x_ref` is a centering constant used only for
 * numerical conditioning; it is absorbed into c0 so the returned coefficients
 * apply directly to raw counts.
 */
TwoPointFit solveTwoPoint( double raw_cold, double t_cold, double raw_hot, double t_hot,
                           int x_ref = 0x4000 );

//! Altitude-corrected boiling point in °C: 100 - 0.0028 * altitude_m.
double altitudeBoilingPointC( double altitude_m );

} // namespace openseekthermal::tools

#endif // OPENSEEKTHERMAL_TOOLS_FIT_TEMPERATURE_HPP
