// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

// Capture data and fit a radial polynomial that models lens-shading vignette.
//
// The Seek shutter calibration cancels per-pixel offsets that share temperature
// with the shutter (which sits next to the sensor). It does not fully cancel
// non-uniformities driven by the *lens*: the lens equilibrates to a different
// temperature than the shutter, so its non-uniform self-emission leaks
// through. On the Nano 300 this typically shows up as image edges reading
// warmer than the centre — a radially symmetric (or near-symmetric) artifact.
//
// We model that artifact as a polynomial in r² (with r the distance from the
// optical axis, defaulted to image centre):
//
//     vignette(x, y) = c[0] + c[1]·u + c[2]·u² + … + c[D]·u^D
//     where u = r²(x, y) / r²_max  (normalised to [0, 1])
//
// Why a radial fit instead of a flat-field PGM:
//   * Doesn't require a perfectly uniform target — non-radial scene variation
//     is in the orthogonal subspace of the radial basis and gets rejected
//     by the least-squares fit automatically.
//   * Tiny output (handful of floats) instead of a width × height × 2 PGM.
//   * Easy to apply at runtime, no per-pixel storage.
//
// Application at runtime (after the camera's normal FFC):
//
//     u = ((x - cx)² + (y - cy)²) / r²_max
//     model = sum(c[k] · u^k for k in 0..D)
//     corrected_pixel = ffc_pixel - model + mean_model
//
// (subtracting `mean_model` preserves overall scene intensity.)
//
// Tips for a clean capture:
//   * Let the camera run for ~60 s before capturing so the body / lens reach
//     thermal equilibrium with ambient.
//   * Point at any roughly-flat target — a wall, a piece of cardboard. The
//     fit is robust to non-radial gradients (e.g. one wall side warmer than
//     the other), so it doesn't have to be perfect.
//   * Avoid your own breath / hand IR signature in the field of view.
//   * Diagnostic outputs (--write-diagnostics) include the per-pixel
//     residual; a clean fit has residuals < a few hundred counts.

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
      << "  --out PATH         output file for fit coefficients (default vignette.ini)\n"
      << "  --frames N         number of thermal frames to average (default 50)\n"
      << "  --warmup K         frames to discard before capture (default 30)\n"
      << "  --degree D         polynomial degree in r² (default 3, range 1..6)\n"
      << "  --dead-pixels PATH  PGM mask from calibrate_dead_pixels; flagged pixels are\n"
      << "                       excluded from the fit (255 = dead, 0 = good)\n"
      << "  --write-diagnostics  also write average / model / residual PGMs next to --out\n"
      << "  --serial S | --port P  device selector\n";
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

// Loads an 8-bit P5 PGM dead-pixel mask (the format written by
// calibrate_dead_pixels). Non-zero byte = dead.
bool loadDeadMaskPgm( const fs::path &path, int expected_width, int expected_height,
                      std::vector<bool> &out_mask, std::string &error )
{
  std::ifstream in( path, std::ios::binary );
  if ( !in ) {
    error = "could not open file";
    return false;
  }
  std::string magic;
  in >> magic;
  if ( magic != "P5" ) {
    error = "not a P5 PGM";
    return false;
  }
  auto skipWhitespaceAndComments = [&]() {
    char c;
    while ( in.get( c ) ) {
      if ( c == '#' ) {
        while ( in.get( c ) && c != '\n' ) {
        }
      } else if ( !std::isspace( static_cast<unsigned char>( c ) ) ) {
        in.unget();
        break;
      }
    }
  };
  skipWhitespaceAndComments();
  int w = 0;
  in >> w;
  skipWhitespaceAndComments();
  int h = 0;
  in >> h;
  skipWhitespaceAndComments();
  int maxval = 0;
  in >> maxval;
  in.get(); // consume the single byte of whitespace after maxval
  if ( maxval != 255 ) {
    error = "expected 8-bit PGM (maxval 255), got " + std::to_string( maxval );
    return false;
  }
  if ( w != expected_width || h != expected_height ) {
    error = "size " + std::to_string( w ) + "x" + std::to_string( h ) + " does not match camera " +
            std::to_string( expected_width ) + "x" + std::to_string( expected_height );
    return false;
  }
  std::vector<uint8_t> bytes( static_cast<size_t>( w ) * h );
  in.read( reinterpret_cast<char *>( bytes.data() ), static_cast<std::streamsize>( bytes.size() ) );
  if ( !in ) {
    error = "short read";
    return false;
  }
  out_mask.assign( bytes.size(), false );
  for ( size_t i = 0; i < bytes.size(); ++i ) out_mask[i] = bytes[i] != 0;
  return true;
}

struct RadialFit {
  int width = 0;
  int height = 0;
  double cx = 0.0;
  double cy = 0.0;
  double r2_max = 0.0;
  int degree = 0;
  std::vector<double> coeffs;
  double mean_model = 0.0;
};

double evaluate( const RadialFit &f, double x, double y )
{
  const double dx = x - f.cx;
  const double dy = y - f.cy;
  const double u = ( dx * dx + dy * dy ) / f.r2_max;
  double v = 0.0;
  double up = 1.0;
  for ( int k = 0; k <= f.degree; ++k ) {
    v += f.coeffs[k] * up;
    up *= u;
  }
  return v;
}

// Solve A * x = b in-place by Gauss-Jordan. A is N×N row-major; b is length N.
// Returns true on success. N is small (≤ 7 in this tool), so this is fine.
bool solveLinear( std::vector<double> &A, std::vector<double> &b, int N )
{
  for ( int i = 0; i < N; ++i ) {
    int pivot = i;
    double pv = std::abs( A[i * N + i] );
    for ( int r = i + 1; r < N; ++r ) {
      const double v = std::abs( A[r * N + i] );
      if ( v > pv ) {
        pv = v;
        pivot = r;
      }
    }
    if ( pv < 1e-18 )
      return false;
    if ( pivot != i ) {
      for ( int c = 0; c < N; ++c ) std::swap( A[i * N + c], A[pivot * N + c] );
      std::swap( b[i], b[pivot] );
    }
    const double inv = 1.0 / A[i * N + i];
    for ( int c = 0; c < N; ++c ) A[i * N + c] *= inv;
    b[i] *= inv;
    for ( int r = 0; r < N; ++r ) {
      if ( r == i )
        continue;
      const double f = A[r * N + i];
      if ( f == 0.0 )
        continue;
      for ( int c = 0; c < N; ++c ) A[r * N + c] -= f * A[i * N + c];
      b[r] -= f * b[i];
    }
  }
  return true;
}

// Fit a polynomial of `degree` in u = r² / r²_max to the averaged image.
// Two passes: an initial least-squares fit (excluding any caller-supplied
// dead pixels), then a robust re-fit additionally excluding pixels whose
// residual exceeds 3·MAD (rejects non-radial scene structure that survived
// averaging, hot/cold spots that the dead-pixel mask missed, etc.).
//
// `dead_mask` may be null. When provided, it must have width*height entries;
// any `true` entry is excluded from both passes and from the MAD computation.
RadialFit fitRadialPolynomial( const std::vector<double> &avg, int width, int height, int degree,
                               const std::vector<bool> *dead_mask = nullptr )
{
  RadialFit fit;
  fit.width = width;
  fit.height = height;
  fit.cx = ( width - 1 ) * 0.5;
  fit.cy = ( height - 1 ) * 0.5;
  // Normalisation: r²_max is the corner distance² so u ∈ [0, 1].
  const double dx_corner = std::max( fit.cx, ( width - 1 ) - fit.cx );
  const double dy_corner = std::max( fit.cy, ( height - 1 ) - fit.cy );
  fit.r2_max = dx_corner * dx_corner + dy_corner * dy_corner;
  fit.degree = degree;
  fit.coeffs.assign( degree + 1, 0.0 );

  const int N = degree + 1;
  std::vector<double> A( N * N, 0.0 );
  std::vector<double> b( N, 0.0 );

  // Pre-cache normalised r² per pixel.
  std::vector<double> u( static_cast<size_t>( width ) * height, 0.0 );
  for ( int y = 0; y < height; ++y ) {
    for ( int x = 0; x < width; ++x ) {
      const double dx = x - fit.cx;
      const double dy = y - fit.cy;
      u[y * width + x] = ( dx * dx + dy * dy ) / fit.r2_max;
    }
  }

  auto accumulate = [&]( const std::vector<bool> *include ) {
    std::fill( A.begin(), A.end(), 0.0 );
    std::fill( b.begin(), b.end(), 0.0 );
    for ( size_t i = 0; i < u.size(); ++i ) {
      if ( include && !( *include )[i] )
        continue;
      double up_i = 1.0;
      std::vector<double> basis( N );
      for ( int k = 0; k < N; ++k ) {
        basis[k] = up_i;
        up_i *= u[i];
      }
      const double y_i = avg[i];
      for ( int r = 0; r < N; ++r ) {
        b[r] += basis[r] * y_i;
        for ( int c = 0; c < N; ++c ) A[r * N + c] += basis[r] * basis[c];
      }
    }
  };

  // Pass 1: all live pixels (dead-mask exclusions only).
  std::vector<bool> include1( u.size(), true );
  if ( dead_mask ) {
    for ( size_t i = 0; i < u.size(); ++i ) include1[i] = !( *dead_mask )[i];
  }
  accumulate( &include1 );
  std::vector<double> coeffs1 = b;
  std::vector<double> A1 = A;
  if ( !solveLinear( A1, coeffs1, N ) )
    return fit; // degenerate; coefficients stay zero
  fit.coeffs = coeffs1;

  // Compute residuals on live pixels for MAD-based robust rejection in pass 2.
  // Dead pixels are skipped entirely so they don't bias the MAD estimate.
  std::vector<double> residuals( u.size(), 0.0 );
  std::vector<double> abs_res_live;
  abs_res_live.reserve( u.size() );
  for ( size_t i = 0; i < u.size(); ++i ) {
    double model = 0.0;
    double up_i = 1.0;
    for ( int k = 0; k < N; ++k ) {
      model += fit.coeffs[k] * up_i;
      up_i *= u[i];
    }
    residuals[i] = avg[i] - model;
    if ( include1[i] )
      abs_res_live.push_back( std::abs( residuals[i] ) );
  }
  double mad = 0.0;
  if ( !abs_res_live.empty() ) {
    const size_t mid = abs_res_live.size() / 2;
    std::nth_element( abs_res_live.begin(), abs_res_live.begin() + mid, abs_res_live.end() );
    mad = abs_res_live[mid];
  }
  // 1.4826 converts MAD to a stddev-equivalent for normal residuals.
  const double cutoff = std::max( 1.0, 3.0 * 1.4826 * mad );

  std::vector<bool> include2( u.size(), false );
  for ( size_t i = 0; i < u.size(); ++i )
    include2[i] = include1[i] && std::abs( residuals[i] ) <= cutoff;

  // Pass 2: dead + MAD outliers both excluded.
  accumulate( &include2 );
  std::vector<double> coeffs2 = b;
  std::vector<double> A2 = A;
  if ( solveLinear( A2, coeffs2, N ) )
    fit.coeffs = coeffs2;

  // Compute the model's spatial mean — needed as the additive constant when
  // applying the correction so that overall image intensity is preserved.
  double sum = 0.0;
  for ( size_t i = 0; i < u.size(); ++i ) {
    double model = 0.0;
    double up_i = 1.0;
    for ( int k = 0; k < N; ++k ) {
      model += fit.coeffs[k] * up_i;
      up_i *= u[i];
    }
    sum += model;
  }
  fit.mean_model = sum / static_cast<double>( u.size() );
  return fit;
}

void writeFit( const RadialFit &f, const fs::path &p, const SeekDevice &device, int frames_used )
{
  std::ofstream out( p );
  out << std::setprecision( 9 );
  out << "; openseekthermal vignette correction (radial polynomial in r²)\n";
  out << "; device " << device << "\n";
  out << "; frames " << frames_used << "\n";
  out << "\n[vignette]\n";
  out << "width = " << f.width << "\n";
  out << "height = " << f.height << "\n";
  out << "center_x = " << f.cx << "\n";
  out << "center_y = " << f.cy << "\n";
  out << "r2_max = " << f.r2_max << "\n";
  out << "degree = " << f.degree << "\n";
  out << "mean_model = " << f.mean_model << "\n";
  for ( int k = 0; k <= f.degree; ++k ) out << "c" << k << " = " << f.coeffs[k] << "\n";
}

} // namespace

int main( int argc, char **argv )
{
  fs::path out_path = "vignette.ini";
  fs::path dead_pixels_path;
  int frames = 50;
  int warmup = 30;
  int degree = 3;
  bool write_diag = false;
  std::string serial;
  std::string port;

  for ( int i = 1; i < argc; ++i ) {
    std::string arg = argv[i];
    if ( ( arg == "--out" || arg == "-o" ) && i + 1 < argc ) {
      out_path = argv[++i];
    } else if ( arg == "--frames" && i + 1 < argc ) {
      frames = std::max( 1, std::atoi( argv[++i] ) );
    } else if ( arg == "--warmup" && i + 1 < argc ) {
      warmup = std::max( 0, std::atoi( argv[++i] ) );
    } else if ( arg == "--degree" && i + 1 < argc ) {
      degree = std::clamp( std::atoi( argv[++i] ), 1, 6 );
    } else if ( arg == "--dead-pixels" && i + 1 < argc ) {
      dead_pixels_path = argv[++i];
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

  std::vector<bool> dead_mask;
  size_t n_dead = 0;
  if ( !dead_pixels_path.empty() ) {
    std::string err;
    if ( !loadDeadMaskPgm( dead_pixels_path, width, height, dead_mask, err ) ) {
      std::cerr << "Failed to load dead pixel mask '" << dead_pixels_path.string() << "': " << err
                << "\n";
      camera->close();
      return 1;
    }
    n_dead = static_cast<size_t>( std::count( dead_mask.begin(), dead_mask.end(), true ) );
    if ( n_dead >= pixel_count ) {
      std::cerr << "All pixels in mask are flagged dead — refusing to fit.\n";
      camera->close();
      return 1;
    }
  }

  std::cout << "Capturing radial vignette fit from " << device << " (" << width << "x" << height
            << ", degree " << degree << ")\n";
  std::cout << "  warmup: " << warmup << " frames, averaging: " << frames << " thermal frames\n";
  if ( !dead_mask.empty() )
    std::cout << "  excluding " << n_dead << " dead pixels from fit (loaded from "
              << dead_pixels_path.string() << ")\n";

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

  // Average pixel-wise. Only THERMAL_FRAMEs (skip CALIBRATION_FRAME).
  std::vector<uint64_t> sum( pixel_count, 0 );
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
    for ( size_t i = 0; i < pixel_count; ++i ) sum[i] += le16toh( frame[i] );
    ++accumulated;
    delete[] buf;
    if ( accumulated % 10 == 0 || accumulated == frames )
      std::cout << "  captured " << accumulated << " / " << frames << "\n";
  }

  std::vector<double> avg( pixel_count );
  for ( size_t i = 0; i < pixel_count; ++i )
    avg[i] = static_cast<double>( sum[i] ) / static_cast<double>( frames );

  RadialFit fit = fitRadialPolynomial( avg, width, height, degree,
                                       dead_mask.empty() ? nullptr : &dead_mask );
  if ( fit.coeffs.empty() ) {
    std::cerr << "Fit failed (degenerate normal equations).\n";
    camera->close();
    return 1;
  }

  // Final residual stats vs. the fit. Skip dead pixels — they don't fit the
  // radial model and would dominate max|residual| / RMS otherwise.
  double res_sum_sq = 0.0;
  double max_abs = 0.0;
  double max_dev = 0.0;
  double min_dev = 0.0;
  size_t live_count = 0;
  for ( int y = 0; y < height; ++y ) {
    for ( int x = 0; x < width; ++x ) {
      const size_t i = y * width + x;
      const double model = evaluate( fit, x, y );
      const double dev = model - fit.mean_model;
      if ( dev > max_dev )
        max_dev = dev;
      if ( dev < min_dev )
        min_dev = dev;
      if ( !dead_mask.empty() && dead_mask[i] )
        continue;
      const double r = avg[i] - model;
      res_sum_sq += r * r;
      if ( std::abs( r ) > max_abs )
        max_abs = std::abs( r );
      ++live_count;
    }
  }
  const double rms = live_count > 0 ? std::sqrt( res_sum_sq / static_cast<double>( live_count ) ) : 0.0;

  std::cout << "Fit: c0=" << fit.coeffs[0];
  for ( int k = 1; k <= fit.degree; ++k ) std::cout << "  c" << k << "=" << fit.coeffs[k];
  std::cout << "\n";
  std::cout << "  mean_model=" << std::fixed << std::setprecision( 1 ) << fit.mean_model
            << "  edge-vs-centre swing=" << ( max_dev - min_dev ) << " counts\n";
  std::cout << "  residual RMS=" << rms << "  max|residual|=" << max_abs << " counts\n";
  std::cout << "  skipped " << skipped_cal << " calibration frames\n";

  if ( ( max_dev - min_dev ) < 200.0 ) {
    std::cout << "Note: radial swing is small — vignette may not be the dominant artifact for\n"
                 "      this unit, or the degree may be too low to capture it.\n";
  }
  if ( max_abs > 5.0 * rms && rms > 0.0 ) {
    std::cout << "Note: large per-pixel residuals remain — likely non-radial scene structure or\n"
                 "      hot/dead pixels in the field. Re-capture against a more uniform target\n"
                 "      or trust the radial fit only.\n";
  }

  writeFit( fit, out_path, device, frames );
  std::cout << "Wrote " << out_path.string() << "\n";

  if ( write_diag ) {
    fs::path stem = out_path;
    stem.replace_extension( "" );
    std::vector<uint16_t> avg_pgm( pixel_count );
    std::vector<uint16_t> model_pgm( pixel_count );
    std::vector<uint16_t> residual_pgm( pixel_count );
    for ( int y = 0; y < height; ++y ) {
      for ( int x = 0; x < width; ++x ) {
        const size_t i = y * width + x;
        const double m = evaluate( fit, x, y );
        const double r = avg[i] - m + fit.mean_model; // residual centred for visibility
        avg_pgm[i] = static_cast<uint16_t>( std::clamp( avg[i], 0.0, 65535.0 ) );
        model_pgm[i] = static_cast<uint16_t>( std::clamp( m, 0.0, 65535.0 ) );
        residual_pgm[i] = static_cast<uint16_t>( std::clamp( r, 0.0, 65535.0 ) );
      }
    }
    writePgm16BE( stem.string() + "_average.pgm", avg_pgm, width, height );
    writePgm16BE( stem.string() + "_model.pgm", model_pgm, width, height );
    writePgm16BE( stem.string() + "_residual.pgm", residual_pgm, width, height );
    std::cout << "Wrote " << stem.string() << "_{average,model,residual}.pgm\n";
  }

  try {
    camera->close();
  } catch ( ... ) {
  }
  return 0;
}
