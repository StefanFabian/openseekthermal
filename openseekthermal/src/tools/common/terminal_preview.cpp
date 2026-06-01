// Copyright (c) 2026 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "terminal_preview.hpp"

#include <algorithm>
#include <istream>
#include <limits>
#include <ostream>
#include <sstream>
#include <unistd.h>
#include <vector>

namespace openseekthermal::tools
{

namespace
{
constexpr const char *kHalfBlock = "\xe2\x96\x80"; // U+2580 UPPER HALF BLOCK
}

bool isInteractiveTerminal() { return isatty( STDIN_FILENO ) != 0 && isatty( STDOUT_FILENO ) != 0; }

TerminalRawMode::TerminalRawMode()
{
  if ( !isatty( STDIN_FILENO ) )
    return;
  if ( tcgetattr( STDIN_FILENO, &saved_ ) != 0 )
    return;
  termios raw = saved_;
  raw.c_lflag &= static_cast<tcflag_t>( ~( ICANON | ECHO ) );
  raw.c_cc[VMIN] = 0; // non-blocking reads
  raw.c_cc[VTIME] = 0;
  if ( tcsetattr( STDIN_FILENO, TCSANOW, &raw ) == 0 )
    active_ = true;
}

TerminalRawMode::~TerminalRawMode()
{
  if ( active_ )
    tcsetattr( STDIN_FILENO, TCSANOW, &saved_ );
}

void renderFrameAnsi( std::ostream &os, const uint16_t *pixels, int width, int height,
                      int stride_px, int target_cols, const std::string &status, bool home )
{
  const int step = std::max( 1, ( width + target_cols - 1 ) / std::max( 1, target_cols ) );
  const int ow = width / step;
  const int oh = height / step;
  if ( ow <= 0 || oh <= 0 )
    return;

  // Downsample with block averaging, excluding sentinels.
  std::vector<float> img( static_cast<size_t>( ow ) * oh, 0.0f );
  float lo = std::numeric_limits<float>::max();
  float hi = std::numeric_limits<float>::lowest();
  for ( int oy = 0; oy < oh; ++oy ) {
    for ( int ox = 0; ox < ow; ++ox ) {
      double sum = 0.0;
      int cnt = 0;
      for ( int yy = 0; yy < step; ++yy ) {
        for ( int xx = 0; xx < step; ++xx ) {
          const uint16_t v =
              pixels[static_cast<size_t>( oy * step + yy ) * stride_px + ox * step + xx];
          if ( v == 0 || v == 0xffff )
            continue;
          sum += v;
          ++cnt;
        }
      }
      const float val = cnt ? static_cast<float>( sum / cnt ) : 0.0f;
      img[static_cast<size_t>( oy ) * ow + ox] = val;
      if ( cnt ) {
        lo = std::min( lo, val );
        hi = std::max( hi, val );
      }
    }
  }
  if ( !( hi > lo ) ) {
    lo = 0.0f;
    hi = 1.0f;
  }
  const float inv = 1.0f / ( hi - lo );

  std::ostringstream out;
  if ( home )
    out << "\033[u"; // restore to the anchor saved by the caller (keeps text above intact)
  if ( !status.empty() )
    out << "\033[2K" << status << "\n";
  auto gray = [&]( float v ) {
    return std::clamp( static_cast<int>( ( v - lo ) * inv * 255.0f ), 0, 255 );
  };
  for ( int ty = 0; ty * 2 + 1 < oh; ++ty ) {
    out << "\033[2K"; // clear to end of line to avoid stale chars
    const int yt = 2 * ty;
    const int yb = 2 * ty + 1;
    for ( int ox = 0; ox < ow; ++ox ) {
      const int gt = gray( img[static_cast<size_t>( yt ) * ow + ox] );
      const int gb = gray( img[static_cast<size_t>( yb ) * ow + ox] );
      out << "\033[38;2;" << gt << ";" << gt << ";" << gt << "m"
          << "\033[48;2;" << gb << ";" << gb << ";" << gb << "m" << kHalfBlock;
    }
    out << "\033[0m\n";
  }
  out << "\033[J"; // clear the rest of the screen below the image
  os << out.str();
  os.flush();
}

bool livePreviewUntilStart( SeekThermalCamera &cam, std::istream &in, std::ostream &os,
                            const std::string &instruction )
{
  if ( !isInteractiveTerminal() ) {
    os << instruction << "\n(press Enter to start, q to cancel): ";
    os.flush();
    std::string line;
    if ( !std::getline( in, line ) )
      return false;
    return line.empty() || ( line[0] != 'q' && line[0] != 'Q' );
  }

  TerminalRawMode raw;
  if ( !raw.active() ) {
    os << instruction << "\n(press Enter to start, q to cancel): ";
    os.flush();
    std::string line;
    if ( !std::getline( in, line ) )
      return false;
    return line.empty() || ( line[0] != 'q' && line[0] != 'Q' );
  }

  os << "\033[s"; // save cursor as the anchor; renders redraw in place below the instructions
  const std::string status = instruction + "   [Enter] start   [q] cancel";
  while ( true ) {
    unsigned char *buf = nullptr;
    size_t buf_size = 0;
    FrameHeader header;
    const GrabFrameResult res = cam.grabFrame( &buf, buf_size, &header );
    if ( res == GrabFrameResult::SUCCESS ) {
      const auto *px = reinterpret_cast<const uint16_t *>( buf );
      renderFrameAnsi( os, px, cam.getFrameWidth(), cam.getFrameHeight(), cam.getFrameWidth(), 48,
                       status, true );
    }
    delete[] buf;

    char c = 0;
    const ssize_t n = read( STDIN_FILENO, &c, 1 );
    if ( n == 1 ) {
      if ( c == '\n' || c == '\r' ) {
        os << "\n";
        return true;
      }
      if ( c == 'q' || c == 'Q' || c == 27 /* Esc */ ) {
        os << "\n";
        return false;
      }
    }
  }
}

} // namespace openseekthermal::tools
