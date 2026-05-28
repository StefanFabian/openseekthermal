// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

// Identify dead / bad pixels and write a binary mask the rest of the toolchain
// can apply at runtime.
//
// Why this tool exists: the camera library already replaces pixels whose raw
// sensor value is exactly 0x0000 or 0xFFFF with a 3x3 gaussian over the valid
// neighbours (see seek_thermal_camera.cpp::extractFrame). That covers
// fully-stuck pixels but misses three other failure modes:
//
//   1. Partially-stuck pixels that read a roughly constant intermediate value
//      and so look fine to the 0/0xffff check.
//   2. Pixels with a large fixed DC offset that survives the shutter FFC
//      (e.g. response slope is wrong, not just the bias).
//   3. Pixels with abnormally high temporal noise.
//
// Capture procedure (caller responsibility):
//   * Let the camera run for ~60 s before starting so the body / lens / shutter
//     reach thermal equilibrium with ambient.
//   * **Pan / sweep the camera across a varied scene** for the entire capture.
//     A good pixel sees different scene content frame to frame and so builds
//     up substantial temporal stddev; a stuck pixel doesn't move with its
//     neighbours and so reads a near-zero stddev. If you instead point at a
//     uniform target, every pixel — good or stuck — has near-zero stddev and
//     stuck pixels become invisible to the temporal pass.
//     Pointing it at your face or hand and slowly turning it away to walls
//     should usually do the trick.
//   * Cover the whole field of view with the motion (corners and centre alike)
//     and include a mix of warm/cold structure in the scene. Slow, smooth
//     motion is fine; you do not need to be precise.
//
// Detection (after capture):
//   For each pixel we compute mean, stddev, min, max across the captured
//   thermal frames. The dead-pixel mask is the union of three flags:
//
//     STUCK:    stddev[i] < max(stuck_floor, k_stuck · median(stddev))
//               (the pixel didn't track scene motion that its neighbours saw)
//     NOISY:    stddev[i] > median(stddev) + k_noisy · 1.4826 · MAD(stddev)
//               (the pixel is much more variable than its neighbours)
//     OFFSET:   |mean[i] − median_filtered(mean, 5x5)[i]|
//                   > k_offset · 1.4826 · MAD(residuals)
//               (the pixel has a fixed DC offset that survives FFC)
//
// Output:
//   * dead_pixels.pgm — 8-bit P5 mask, 255 = dead, 0 = good
//     (matches OpenCV inpaint / numpy masked-array convention: non-zero means
//     "this pixel needs replacement")
//   * --write-diagnostics also writes mean / stddev / residual PGMs
//
// Notes:
//   * If you have a vignette polynomial fit (calibrate_vignette), pass it via
//     --vignette to subtract the radial component before the OFFSET pass. This
//     makes the spatial check more sensitive to small per-pixel offsets in the
//     corners, which would otherwise be partially hidden under residual
//     vignette in the time-averaged mean image.
//   * If the median stddev is very small the tool warns: that means the scene
//     wasn't varied enough during capture and the STUCK pass is unreliable.

#include "openseekthermal/camera_calibration.hpp"
#include "openseekthermal/openseekthermal.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <endian.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using namespace openseekthermal;

namespace
{

void printUsage( const char *argv0 )
{
  std::cout
      << "Usage: " << argv0 << " [options]\n"
      << "  --calibration PATH       unified calibration file to update (default "
         "calibration.ini).\n"
      << "                             Reads [vignette] (if present) for de-vignetting, writes "
         "the\n"
      << "                             [dead_pixels] section back.\n"
      << "  --frames N               number of thermal frames to capture (default 200)\n"
      << "  --warmup K               frames to discard before capture (default 30)\n"
      << "  --offset-k K             offset MAD multiplier for DC-offset pixels (default 8.0)\n"
      << "  --noisy-k K              MAD multiplier for noisy pixels (default 6.0)\n"
      << "  --stuck-k K              stddev fraction below local median to flag as stuck\n"
      << "                             (default 0.3 — i.e. < 30% of median stddev)\n"
      << "  --stuck-floor F          absolute stddev floor below which a pixel is stuck\n"
      << "                             regardless of --stuck-k (default 2.0 counts)\n"
      << "  --write-diagnostics      also write mean/stddev/residual PGMs next to --calibration\n"
      << "  --serial S | --port P    device selector\n"
      << "\n"
      << "Capture: PAN / SWEEP THE CAMERA across a varied scene during capture.\n"
      << "Holding still or pointing at a uniform field will hide stuck pixels.\n";
}

bool writePgm8( const fs::path &path, const std::vector<uint8_t> &data, int width, int height )
{
  std::ofstream out( path, std::ios::binary );
  if ( !out )
    return false;
  out << "P5\n" << width << " " << height << "\n255\n";
  out.write( reinterpret_cast<const char *>( data.data() ),
             static_cast<std::streamsize>( data.size() ) );
  return out.good();
}

bool writePgm16BE( const fs::path &path, const std::vector<uint16_t> &data, int width, int height )
{
  std::ofstream out( path, std::ios::binary );
  if ( !out )
    return false;
  out << "P5\n" << width << " " << height << "\n65535\n";
  std::vector<uint8_t> bytes( data.size() * 2 );
  for ( size_t i = 0; i < data.size(); ++i ) {
    bytes[2 * i] = static_cast<uint8_t>( ( data[i] >> 8 ) & 0xFF );
    bytes[2 * i + 1] = static_cast<uint8_t>( data[i] & 0xFF );
  }
  out.write( reinterpret_cast<const char *>( bytes.data() ),
             static_cast<std::streamsize>( bytes.size() ) );
  return out.good();
}

// 5x5 median over a separated source. Pixels near the border use a smaller
// window (everything that fits inside the image).
std::vector<double> medianFilter5x5( const std::vector<double> &src, int width, int height )
{
  std::vector<double> dst( src.size(), 0.0 );
  std::vector<double> window;
  window.reserve( 25 );
  for ( int y = 0; y < height; ++y ) {
    const int y0 = std::max( 0, y - 2 );
    const int y1 = std::min( height - 1, y + 2 );
    for ( int x = 0; x < width; ++x ) {
      const int x0 = std::max( 0, x - 2 );
      const int x1 = std::min( width - 1, x + 2 );
      window.clear();
      for ( int yy = y0; yy <= y1; ++yy ) {
        for ( int xx = x0; xx <= x1; ++xx ) { window.push_back( src[yy * width + xx] ); }
      }
      const size_t mid = window.size() / 2;
      std::nth_element( window.begin(), window.begin() + mid, window.end() );
      dst[y * width + x] = window[mid];
    }
  }
  return dst;
}

double mad( std::vector<double> values )
{
  if ( values.empty() )
    return 0.0;
  const size_t mid = values.size() / 2;
  std::nth_element( values.begin(), values.begin() + mid, values.end() );
  const double median = values[mid];
  for ( double &v : values ) v = std::abs( v - median );
  std::nth_element( values.begin(), values.begin() + mid, values.end() );
  return values[mid];
}

double median( std::vector<double> values )
{
  if ( values.empty() )
    return 0.0;
  const size_t mid = values.size() / 2;
  std::nth_element( values.begin(), values.begin() + mid, values.end() );
  return values[mid];
}

} // namespace

int main( int argc, char **argv )
{
  fs::path calibration_path = "calibration.ini";
  int frames = 200;
  int warmup = 30;
  double k_offset = 8.0;
  double k_noisy = 6.0;
  double k_stuck = 0.3;
  double stuck_floor = 2.0;
  bool write_diag = false;
  std::string serial;
  std::string port;

  for ( int i = 1; i < argc; ++i ) {
    std::string arg = argv[i];
    if ( ( arg == "--calibration" || arg == "-c" ) && i + 1 < argc ) {
      calibration_path = argv[++i];
    } else if ( arg == "--frames" && i + 1 < argc ) {
      frames = std::max( 4, std::atoi( argv[++i] ) );
    } else if ( arg == "--warmup" && i + 1 < argc ) {
      warmup = std::max( 0, std::atoi( argv[++i] ) );
    } else if ( arg == "--offset-k" && i + 1 < argc ) {
      k_offset = std::atof( argv[++i] );
    } else if ( arg == "--noisy-k" && i + 1 < argc ) {
      k_noisy = std::atof( argv[++i] );
    } else if ( arg == "--stuck-k" && i + 1 < argc ) {
      k_stuck = std::atof( argv[++i] );
    } else if ( arg == "--stuck-floor" && i + 1 < argc ) {
      stuck_floor = std::atof( argv[++i] );
    } else if ( arg == "--write-diagnostics" ) {
      write_diag = true;
    } else if ( arg == "--serial" && i + 1 < argc ) {
      serial = argv[++i];
    } else if ( arg == "--port" && i + 1 < argc ) {
      port = argv[++i];
    } else if ( arg == "-h" || arg == "--help" ) {
      printUsage( argv[0] );
      return 0;
    } else {
      std::cerr << "Unknown argument: " << arg << "\n\n";
      printUsage( argv[0] );
      return 2;
    }
  }

  auto devices = listDevices();
  if ( devices.empty() ) {
    std::cerr << "No SeekThermal devices found.\n";
    return 1;
  }

  SeekDevice device = devices[0];
  if ( !serial.empty() ) {
    auto it = std::find_if( devices.begin(), devices.end(),
                            [&]( const SeekDevice &d ) { return d.serial == serial; } );
    if ( it == devices.end() ) {
      std::cerr << "Device with serial '" << serial << "' not found.\n";
      return 1;
    }
    device = *it;
  } else if ( !port.empty() ) {
    auto it = std::find_if( devices.begin(), devices.end(),
                            [&]( const SeekDevice &d ) { return d.usb_port == port; } );
    if ( it == devices.end() ) {
      std::cerr << "Device with port '" << port << "' not found.\n";
      return 1;
    }
    device = *it;
  }

  auto camera = createCamera( device );
  if ( !camera ) {
    std::cerr << "Failed to create camera for " << device << "\n";
    return 1;
  }

  try {
    camera->open();
  } catch ( const std::exception &e ) {
    std::cerr << "Failed to open camera: " << e.what() << "\n";
    return 1;
  }

  const int width = camera->getFrameWidth();
  const int height = camera->getFrameHeight();
  const size_t pixel_count = static_cast<size_t>( width ) * height;

  CameraCalibration cal;
  if ( fs::exists( calibration_path ) ) {
    try {
      cal = loadCameraCalibration( calibration_path, width, height );
    } catch ( const std::exception &e ) {
      std::cerr << "Failed to read existing calibration '" << calibration_path.string()
                << "': " << e.what() << "\n";
      camera->close();
      return 1;
    }
  }

  std::cout << "Capturing dead-pixel statistics from " << device << " (" << width << "x" << height
            << ")\n";
  std::cout << "  warmup: " << warmup << " frames, capturing: " << frames << " thermal frames\n";
  std::cout << "  >>> PAN / SWEEP THE CAMERA across a varied scene for the entire capture <<<\n";

  // Warmup.
  for ( int i = 0; i < warmup; ++i ) {
    unsigned char *buf = nullptr;
    size_t buf_size = 0;
    if ( camera->grabFrame( &buf, buf_size, nullptr ) != GrabFrameResult::SUCCESS ) {
      delete[] buf;
      std::cerr << "Warmup grab failed.\n";
      camera->close();
      return 1;
    }
    delete[] buf;
  }

  // Welford's algorithm for per-pixel mean and stddev — keeps memory at O(N)
  // pixels regardless of frame count.
  std::vector<double> mean_buf( pixel_count, 0.0 );
  std::vector<double> m2_buf( pixel_count, 0.0 );
  std::vector<uint16_t> min_buf( pixel_count, 0xFFFF );
  std::vector<uint16_t> max_buf( pixel_count, 0 );
  int accumulated = 0;
  int skipped_cal = 0;
  while ( accumulated < frames ) {
    FrameHeader header;
    unsigned char *buf = nullptr;
    size_t buf_size = 0;
    GrabFrameResult res = camera->grabFrame( &buf, buf_size, &header );
    if ( res != GrabFrameResult::SUCCESS ) {
      delete[] buf;
      std::cerr << "Capture grab failed: " << to_string( res ) << "\n";
      camera->close();
      return 1;
    }
    if ( header.getFrameType() == FrameType::CALIBRATION_FRAME ) {
      ++skipped_cal;
      delete[] buf;
      continue;
    }
    const auto *frame = reinterpret_cast<const uint16_t *>( buf );
    ++accumulated;
    const double n_inv = 1.0 / static_cast<double>( accumulated );
    for ( size_t i = 0; i < pixel_count; ++i ) {
      const uint16_t v = le16toh( frame[i] );
      if ( v < min_buf[i] )
        min_buf[i] = v;
      if ( v > max_buf[i] )
        max_buf[i] = v;
      const double delta = static_cast<double>( v ) - mean_buf[i];
      mean_buf[i] += delta * n_inv;
      const double delta2 = static_cast<double>( v ) - mean_buf[i];
      m2_buf[i] += delta * delta2;
    }
    delete[] buf;
    if ( accumulated % 10 == 0 || accumulated == frames )
      std::cout << "  captured " << accumulated << " / " << frames << "\n";
  }

  std::vector<double> stddev_buf( pixel_count, 0.0 );
  if ( accumulated > 1 ) {
    const double denom = static_cast<double>( accumulated - 1 );
    for ( size_t i = 0; i < pixel_count; ++i ) stddev_buf[i] = std::sqrt( m2_buf[i] / denom );
  }

  // Optional vignette removal (improves spatial sensitivity in the corners).
  std::vector<double> mean_devignette = mean_buf;
  if ( cal.vignette ) {
    for ( int y = 0; y < height; ++y ) {
      for ( int x = 0; x < width; ++x ) {
        const double v = cal.vignette->evaluate( x, y );
        mean_devignette[y * width + x] -= ( v - cal.vignette->mean_model );
      }
    }
  }

  // OFFSET pass: residual = mean − local 5x5 median, then MAD across pixels.
  const std::vector<double> mean_smoothed = medianFilter5x5( mean_devignette, width, height );
  std::vector<double> residual( pixel_count );
  for ( size_t i = 0; i < pixel_count; ++i ) residual[i] = mean_devignette[i] - mean_smoothed[i];
  const double offset_mad = mad( residual );
  const double offset_cutoff = std::max( 1.0, k_offset * 1.4826 * offset_mad );

  // Temporal: stuck (low stddev — pixel didn't track scene motion) and noisy
  // (high stddev outlier).
  const double stddev_median = median( stddev_buf );
  const double stddev_mad = mad( stddev_buf );
  const double noisy_cutoff = stddev_median + k_noisy * 1.4826 * stddev_mad;
  const double stuck_cutoff = std::max( stuck_floor, k_stuck * stddev_median );

  std::vector<uint8_t> mask( pixel_count, 0 );
  size_t offset_dead = 0;
  size_t noisy_dead = 0;
  size_t stuck_dead = 0;
  for ( size_t i = 0; i < pixel_count; ++i ) {
    bool dead = false;
    if ( std::abs( residual[i] ) > offset_cutoff ) {
      ++offset_dead;
      dead = true;
    }
    if ( stddev_buf[i] > noisy_cutoff ) {
      ++noisy_dead;
      dead = true;
    } else if ( stddev_buf[i] < stuck_cutoff ) {
      ++stuck_dead;
      dead = true;
    }
    if ( dead )
      mask[i] = 255;
  }
  size_t total_dead = 0;
  for ( uint8_t m : mask ) {
    if ( m != 0 )
      ++total_dead;
  }

  std::cout << std::fixed << std::setprecision( 2 );
  std::cout << "Offset:  residual MAD=" << offset_mad << " cutoff=" << offset_cutoff
            << " counts -> " << offset_dead << " flagged\n";
  std::cout << "Stuck:   stddev median=" << stddev_median << " cutoff<" << stuck_cutoff << " -> "
            << stuck_dead << " flagged\n";
  std::cout << "Noisy:   stddev MAD=" << stddev_mad << " cutoff>" << noisy_cutoff << " -> "
            << noisy_dead << " flagged\n";
  std::cout << "Total dead pixels: " << total_dead << " / " << pixel_count << " ("
            << ( 100.0 * static_cast<double>( total_dead ) / static_cast<double>( pixel_count ) )
            << "%)\n";
  std::cout << "Skipped " << skipped_cal << " calibration frames\n";

  // The STUCK pass only works if good pixels saw real scene variation. If the
  // median stddev is tiny the camera was effectively static — warn the caller.
  if ( stddev_median < 5.0 ) {
    std::cout << "\nWarning: median temporal stddev is " << stddev_median
              << " counts — the\n"
                 "      scene barely changed during capture. Stuck-pixel detection is\n"
                 "      unreliable. Re-run while panning the camera across a varied scene.\n";
  }

  std::vector<std::pair<int, int>> dead_coords;
  dead_coords.reserve( total_dead );
  for ( int y = 0; y < height; ++y ) {
    for ( int x = 0; x < width; ++x ) {
      if ( mask[static_cast<size_t>( y ) * width + x] != 0 )
        dead_coords.emplace_back( x, y );
    }
  }
  cal.dead_pixels = DeadPixelMask( width, height, dead_coords );
  std::ostringstream hdr;
  hdr << "Dead-pixel mask (" << total_dead << " entries) generated by calibrate_dead_pixels.";
  try {
    saveCameraCalibration( calibration_path, cal, hdr.str() );
  } catch ( const std::exception &e ) {
    std::cerr << "Failed to write calibration '" << calibration_path.string() << "': " << e.what()
              << "\n";
    camera->close();
    return 1;
  }
  std::cout << "Wrote " << calibration_path.string() << " [dead_pixels]\n";

  if ( write_diag ) {
    fs::path stem = calibration_path;
    stem.replace_extension( "" );
    std::vector<uint16_t> mean_pgm( pixel_count );
    std::vector<uint16_t> stddev_pgm( pixel_count );
    std::vector<uint16_t> residual_pgm( pixel_count );
    // Stretch stddev to 0..65535 for visibility (it's typically tiny).
    double stddev_max = 0.0;
    for ( double v : stddev_buf ) {
      if ( v > stddev_max )
        stddev_max = v;
    }
    const double stddev_scale = stddev_max > 0.0 ? 65535.0 / stddev_max : 0.0;
    for ( size_t i = 0; i < pixel_count; ++i ) {
      mean_pgm[i] = static_cast<uint16_t>( std::clamp( mean_buf[i], 0.0, 65535.0 ) );
      stddev_pgm[i] =
          static_cast<uint16_t>( std::clamp( stddev_buf[i] * stddev_scale, 0.0, 65535.0 ) );
      // Residual is centred at 32768 so positive and negative are both visible.
      residual_pgm[i] = static_cast<uint16_t>( std::clamp( residual[i] + 32768.0, 0.0, 65535.0 ) );
    }
    writePgm16BE( stem.string() + "_mean.pgm", mean_pgm, width, height );
    writePgm16BE( stem.string() + "_stddev.pgm", stddev_pgm, width, height );
    writePgm16BE( stem.string() + "_residual.pgm", residual_pgm, width, height );
    std::cout << "Wrote " << stem.string() << "_{mean,stddev,residual}.pgm\n";
    std::cout << "  (stddev scaled by " << stddev_scale << " for display)\n";
  }

  if ( total_dead > pixel_count / 50 ) {
    std::cout << "\nNote: more than 2% of pixels were flagged. The scene may be too non-uniform\n"
                 "      for the offset check, or thresholds may be too tight. Loosen detection by\n"
                 "      raising --offset-k / --noisy-k (or lowering --stuck-k), or recapture\n"
                 "      against a more uniform target. See --help for the full list.\n";
  }

  try {
    camera->close();
  } catch ( ... ) {
  }
  return 0;
}
