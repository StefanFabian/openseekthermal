// Copyright (c) 2026 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

// Shared capture / device-selection / prompt helpers for the calibration tooling.
//
// Endianness note: grabFrame() returns host-endian uint16 (per its header
// contract), so the grabFrame-based accumulators read pixels directly. Only the
// _grabRawFrame raw-transfer path (captureAnchorLive) is little-endian wire data
// and goes through le16toh inside fit_temperature.

#ifndef OPENSEEKTHERMAL_TOOLS_CAPTURE_HELPERS_HPP
#define OPENSEEKTHERMAL_TOOLS_CAPTURE_HELPERS_HPP

#include "detect_dead_pixels.hpp"
#include "fit_temperature.hpp"
#include "openseekthermal/openseekthermal.hpp"
#include "terminal_preview.hpp"

#include <cstdint>
#include <iosfwd>
#include <string>
#include <vector>

namespace openseekthermal::tools
{

//! Non-interactive device selection. Serial takes precedence over port; with
//! neither set the first device is returned. Returns false (with err set) if no
//! device matches.
bool pickDevice( const std::string &serial, const std::string &port, SeekDevice &out,
                 std::string &err );

//! List devices and prompt for an index. Auto-selects when exactly one device
//! is present. Returns false (with err set) on no devices / invalid choice.
bool pickDeviceInteractive( SeekDevice &out, std::string &err, std::istream &in, std::ostream &os );

//! Drain and discard `frames` frames to let the camera reach equilibrium.
bool warmup( SeekThermalCamera &cam, int frames, std::string &err );

/*!
 * Average `frames` THERMAL frames pixel-wise into `sum` (skips CALIBRATION
 * frames). Frames are grabbed via `grabRawCountsFrame()`, i.e. accumulated in the
 * raw-count domain (pre temperature-mapping) so the result can be fit/applied as a
 * count-domain correction. The caller divides by `frames` to obtain the mean.
 * `status_label` prefixes the live preview progress line (e.g. "Capturing
 * vignette"). When `preview` is set each frame is rendered live in place via
 * `progress`; otherwise `progress`, when non-null, receives periodic
 * "captured N / M" lines.
 */
bool accumulateAverage( SeekThermalCamera &cam, int width, int height, int frames,
                        std::vector<uint64_t> &sum, int &skipped_cal, const std::string &status_label,
                        std::ostream *progress, bool preview, std::string &err );

/*!
 * Per-pixel temporal statistics over `frames` RAW THERMAL frames for dead-pixel
 * detection (skips CALIBRATION frames). Reads the raw transfer (little-endian
 * wire data, before the driver's 0/0xFFFF inpaint) so sentinel and stuck pixels
 * are visible; sentinel readings are excluded from mean/stddev and counted.
 */
bool captureWelfordRaw( SeekThermalCamera &cam, const SeekDevice &device, int width, int height,
                        int frames, RawPixelStats &out, std::ostream *progress, bool preview,
                        std::string &err );

/*!
 * Capture a live two-point anchor: grab raw transfers and accumulate the ROI
 * percentile until `frames` usable THERMAL frames are collected. ROI x/y/w/h are
 * in visible-image coordinates; the padded row stride is resolved from `device`.
 */
bool captureAnchorLive( SeekThermalCamera &cam, const SeekDevice &device, int rx, int ry, int rw,
                        int rh, double robust_percentile, bool is_hot, int frames,
                        TwoPointAnchor &out, std::ostream *progress, bool preview, std::string &err );

// --- stdin/stdout prompt helpers (blank input takes the default) -------------

int promptInt( std::istream &in, std::ostream &os, const std::string &label, int def );
double promptDouble( std::istream &in, std::ostream &os, const std::string &label, double def );
bool promptYesNo( std::istream &in, std::ostream &os, const std::string &label, bool def );
std::string promptLine( std::istream &in, std::ostream &os, const std::string &label,
                        const std::string &def );

} // namespace openseekthermal::tools

#endif // OPENSEEKTHERMAL_TOOLS_CAPTURE_HELPERS_HPP
