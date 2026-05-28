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
// optical axis, which is jointly fit alongside the coefficients):
//
//     vignette(x, y) = c[0] + c[1]·u + c[2]·u² + … + c[D]·u^D
//     where u = ((x - cx)² + (y - cy)²) / r²_max  (≈ [0, 1])
//
// r²_max is fixed at the image-center corner distance² so the coefficient
// parametrization stays comparable across runs. The center (cx, cy) starts at
// the image center and is refined by Gauss-Newton.
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
      << "  --calibration PATH unified calibration file to update (default calibration.ini).\n"
      << "                       Reads [dead_pixels] (if present) for fit exclusion, writes\n"
      << "                       the [vignette] section back.\n"
      << "  --frames N         number of thermal frames to average (default 50)\n"
      << "  --warmup K         frames to discard before capture (default 30)\n"
      << "  --degree D         polynomial degree in r² (default 3, range 1..6)\n"
      << "  --fix-center       lock the optical center to the image center instead of\n"
      << "                       jointly fitting it (use if the joint fit drifts on a\n"
      << "                       poor target)\n"
      << "  --write-diagnostics  also write average / model / residual PGMs next to --calibration\n"
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

// Jointly fit (cx, cy, c[0..D]) to the averaged image.
//
// The model is nonlinear in (cx, cy) but linear in the coefficients, so we
// alternate: hold (cx, cy) fixed and solve coeffs in closed form, then take a
// Gauss-Newton step on (cx, cy) holding coeffs fixed. r²_max is kept fixed at
// the image-center corner distance² — moving the center slightly shifts u
// out of [0, 1] in places, which the coefficients absorb.
//
// After convergence we do one robust MAD-based refit of the coefficients,
// rejecting pixels whose residual exceeds 3·MAD (non-radial scene structure
// that survived averaging, hot/cold spots the dead-pixel mask missed, etc.).
//
// `dead_mask` may be null. When provided, any `true` entry is excluded from
// every fit pass and from the MAD computation.
RadialFit fitRadialPolynomial( const std::vector<double> &avg, int width, int height, int degree,
                               bool fit_center, const std::vector<bool> *dead_mask = nullptr )
{
  RadialFit fit;
  fit.width = width;
  fit.height = height;
  fit.cx = ( width - 1 ) * 0.5;
  fit.cy = ( height - 1 ) * 0.5;
  // Fixed normalisation based on image-center corner distance².
  const double dx_corner = std::max( fit.cx, ( width - 1 ) - fit.cx );
  const double dy_corner = std::max( fit.cy, ( height - 1 ) - fit.cy );
  fit.r2_max = dx_corner * dx_corner + dy_corner * dy_corner;
  fit.degree = degree;
  fit.coeffs.assign( degree + 1, 0.0 );

  const int N = degree + 1;
  const size_t P = static_cast<size_t>( width ) * height;

  std::vector<bool> live( P, true );
  if ( dead_mask ) {
    for ( size_t i = 0; i < P; ++i ) live[i] = !( *dead_mask )[i];
  }

  std::vector<double> u( P, 0.0 );
  auto computeU = [&]() {
    for ( int y = 0; y < height; ++y ) {
      for ( int x = 0; x < width; ++x ) {
        const double dx = x - fit.cx;
        const double dy = y - fit.cy;
        u[y * width + x] = ( dx * dx + dy * dy ) / fit.r2_max;
      }
    }
  };

  auto solveCoeffs = [&]( const std::vector<bool> &include ) -> bool {
    std::vector<double> A( N * N, 0.0 );
    std::vector<double> b( N, 0.0 );
    std::vector<double> basis( N );
    for ( size_t i = 0; i < P; ++i ) {
      if ( !include[i] )
        continue;
      double up_i = 1.0;
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
    if ( !solveLinear( A, b, N ) )
      return false;
    fit.coeffs = b;
    return true;
  };

  computeU();
  if ( !solveCoeffs( live ) )
    return fit; // degenerate

  // Joint refinement of (cx, cy) by Gauss-Newton, alternating with a coeff
  // resolve at each step. Converges in well under 30 iterations for any
  // physically realistic vignette. Skipped when the caller locks the center.
  constexpr int kMaxCenterIters = 30;
  constexpr double kCenterTolPx = 0.01;
  for ( int iter = 0; fit_center && iter < kMaxCenterIters; ++iter ) {
    // Build 2×2 normal equations for the (cx, cy) update.
    // ∂model/∂cx = (-2 (x - cx) / r²_max) · D(u)
    // where D(u) = Σ_{k≥1} c_k · k · u^(k-1).
    double JJxx = 0.0, JJxy = 0.0, JJyy = 0.0;
    double Jrx = 0.0, Jry = 0.0;
    for ( int y = 0; y < height; ++y ) {
      for ( int x = 0; x < width; ++x ) {
        const size_t i = static_cast<size_t>( y ) * width + x;
        if ( !live[i] )
          continue;
        const double ui = u[i];
        double Du = 0.0;
        double upow = 1.0; // u^(k-1) starting at k=1
        double model = fit.coeffs[0];
        for ( int k = 1; k <= degree; ++k ) {
          Du += fit.coeffs[k] * k * upow;
          upow *= ui;
          model += fit.coeffs[k] * upow;
        }
        const double resi = avg[i] - model;
        const double factor = -2.0 / fit.r2_max * Du;
        const double Jx = factor * ( x - fit.cx );
        const double Jy = factor * ( y - fit.cy );
        JJxx += Jx * Jx;
        JJxy += Jx * Jy;
        JJyy += Jy * Jy;
        Jrx += Jx * resi;
        Jry += Jy * resi;
      }
    }
    const double det = JJxx * JJyy - JJxy * JJxy;
    if ( std::abs( det ) < 1e-18 )
      break; // singular — leave center where it is
    double dcx = ( JJyy * Jrx - JJxy * Jry ) / det;
    double dcy = ( JJxx * Jry - JJxy * Jrx ) / det;
    // Cap the step at a quarter of the image size to keep early iterations
    // sane if the initial residual happens to be dominated by scene structure.
    const double max_step = 0.25 * std::min( width, height );
    const double step = std::hypot( dcx, dcy );
    if ( step > max_step ) {
      dcx *= max_step / step;
      dcy *= max_step / step;
    }
    fit.cx = std::clamp( fit.cx + dcx, 0.0, static_cast<double>( width - 1 ) );
    fit.cy = std::clamp( fit.cy + dcy, 0.0, static_cast<double>( height - 1 ) );
    computeU();
    if ( !solveCoeffs( live ) )
      return fit;
    if ( std::hypot( dcx, dcy ) < kCenterTolPx )
      break;
  }

  // Robust pass: compute residuals at the current (cx, cy, coeffs), reject
  // pixels above 3·MAD, and refit coefficients. The center is not re-updated
  // here — by this point it's already converged to sub-pixel tolerance.
  std::vector<double> residuals( P, 0.0 );
  std::vector<double> abs_res_live;
  abs_res_live.reserve( P );
  for ( size_t i = 0; i < P; ++i ) {
    double model = 0.0;
    double up_i = 1.0;
    for ( int k = 0; k < N; ++k ) {
      model += fit.coeffs[k] * up_i;
      up_i *= u[i];
    }
    residuals[i] = avg[i] - model;
    if ( live[i] )
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
  std::vector<bool> include_robust( P, false );
  for ( size_t i = 0; i < P; ++i )
    include_robust[i] = live[i] && std::abs( residuals[i] ) <= cutoff;
  solveCoeffs( include_robust );

  // Compute the model's spatial mean — needed as the additive constant when
  // applying the correction so that overall image intensity is preserved.
  double sum = 0.0;
  for ( size_t i = 0; i < P; ++i ) {
    double model = 0.0;
    double up_i = 1.0;
    for ( int k = 0; k < N; ++k ) {
      model += fit.coeffs[k] * up_i;
      up_i *= u[i];
    }
    sum += model;
  }
  fit.mean_model = sum / static_cast<double>( P );
  return fit;
}

VignetteCorrection toVignetteCorrection( const RadialFit &f )
{
  VignetteCorrection v;
  v.width = f.width;
  v.height = f.height;
  v.cx = f.cx;
  v.cy = f.cy;
  v.r2_max = f.r2_max;
  v.degree = f.degree;
  v.mean_model = f.mean_model;
  v.coeffs.assign( f.coeffs.begin(), f.coeffs.end() );
  return v;
}

} // namespace

int main( int argc, char **argv )
{
  fs::path calibration_path = "calibration.ini";
  int frames = 50;
  int warmup = 30;
  int degree = 3;
  bool fit_center = true;
  bool write_diag = false;
  std::string serial;
  std::string port;

  for ( int i = 1; i < argc; ++i ) {
    std::string arg = argv[i];
    if ( ( arg == "--calibration" || arg == "-c" ) && i + 1 < argc ) {
      calibration_path = argv[++i];
    } else if ( arg == "--frames" && i + 1 < argc ) {
      frames = std::max( 1, std::atoi( argv[++i] ) );
    } else if ( arg == "--warmup" && i + 1 < argc ) {
      warmup = std::max( 0, std::atoi( argv[++i] ) );
    } else if ( arg == "--degree" && i + 1 < argc ) {
      degree = std::clamp( std::atoi( argv[++i] ), 1, 6 );
    } else if ( arg == "--fix-center" ) {
      fit_center = false;
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
  std::vector<bool> dead_mask;
  size_t n_dead = 0;
  if ( fs::exists( calibration_path ) ) {
    try {
      cal = loadCameraCalibration( calibration_path, width, height );
    } catch ( const std::exception &e ) {
      std::cerr << "Failed to read existing calibration '" << calibration_path.string()
                << "': " << e.what() << "\n";
      camera->close();
      return 1;
    }
    if ( cal.dead_pixels ) {
      dead_mask.assign( pixel_count, false );
      for ( const auto &entry : cal.dead_pixels->entries() ) { dead_mask[entry.index] = true; }
      n_dead = cal.dead_pixels->deadPixelCount();
    }
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
    std::cout << "  excluding " << n_dead << " dead pixels from fit (from "
              << calibration_path.string() << " [dead_pixels])\n";

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

  RadialFit fit = fitRadialPolynomial( avg, width, height, degree, fit_center,
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
  const double rms =
      live_count > 0 ? std::sqrt( res_sum_sq / static_cast<double>( live_count ) ) : 0.0;

  std::cout << "Fit: c0=" << fit.coeffs[0];
  for ( int k = 1; k <= fit.degree; ++k ) std::cout << "  c" << k << "=" << fit.coeffs[k];
  std::cout << "\n";
  const double img_cx = ( width - 1 ) * 0.5;
  const double img_cy = ( height - 1 ) * 0.5;
  std::cout << "  center=(" << std::fixed << std::setprecision( 2 ) << fit.cx << ", " << fit.cy
            << ")  offset-from-image-center=(" << ( fit.cx - img_cx ) << ", " << ( fit.cy - img_cy )
            << ") px\n";
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

  cal.vignette = toVignetteCorrection( fit );
  std::ostringstream hdr;
  hdr << "openseekthermal vignette correction (radial polynomial in r²) for " << device << " from "
      << frames << " frames.";
  try {
    saveCameraCalibration( calibration_path, cal, hdr.str() );
  } catch ( const std::exception &e ) {
    std::cerr << "Failed to write calibration '" << calibration_path.string() << "': " << e.what()
              << "\n";
    camera->close();
    return 1;
  }
  std::cout << "Wrote " << calibration_path.string() << " [vignette]\n";

  if ( write_diag ) {
    fs::path stem = calibration_path;
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
