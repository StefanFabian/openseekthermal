// Copyright (c) 2026 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

// Dependency-free live terminal preview for the calibration wizard, so the user
// can aim the camera instead of capturing blind. Renders a downscaled,
// auto-contrast grayscale image using ANSI 24-bit color and Unicode half-block
// characters (two vertical pixels per character cell). Linux/POSIX only.

#ifndef OPENSEEKTHERMAL_TOOLS_TERMINAL_PREVIEW_HPP
#define OPENSEEKTHERMAL_TOOLS_TERMINAL_PREVIEW_HPP

#include "openseekthermal/openseekthermal.hpp"

#include <cstdint>
#include <iosfwd>
#include <string>
#include <termios.h>

namespace openseekthermal::tools
{

//! True when both stdin and stdout are interactive terminals.
bool isInteractiveTerminal();

//! RAII: switch stdin to non-canonical, no-echo, non-blocking reads while alive
//! (no-op when stdin is not a TTY); restores the previous mode on destruction.
class TerminalRawMode
{
public:
  TerminalRawMode();
  ~TerminalRawMode();
  TerminalRawMode( const TerminalRawMode & ) = delete;
  TerminalRawMode &operator=( const TerminalRawMode & ) = delete;

  bool active() const { return active_; }

private:
  bool active_ = false;
  termios saved_{};
};

/*!
 * Render one host-endian uint16 frame as a downscaled grayscale image. When
 * `home` is true the cursor is first restored to the anchor previously saved by
 * the caller (write "\033[s" once before the render loop), so the preview
 * redraws in place below any instructions without clearing the screen. Sentinel
 * pixels (0/0xFFFF) are excluded from the auto-contrast range. `stride_px` is the
 * row stride in pixels (== width for grabFrame output; the padded row step for a
 * raw transfer).
 */
void renderFrameAnsi( std::ostream &os, const uint16_t *pixels, int width, int height,
                      int stride_px, int target_cols, const std::string &status, bool home );

/*!
 * Show a live preview, grabbing and rendering frames until the user presses
 * Enter (returns true = start) or q / Esc (returns false = cancel). Falls back
 * to a single prompt + line read when not an interactive terminal.
 */
bool livePreviewUntilStart( SeekThermalCamera &cam, std::istream &in, std::ostream &os,
                            const std::string &instruction );

} // namespace openseekthermal::tools

#endif // OPENSEEKTHERMAL_TOOLS_TERMINAL_PREVIEW_HPP
