// Copyright (c) 2026 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

// Interactive, menu-driven CLI that walks through the full per-unit calibration
// (vignette, dead pixels, temperature) and writes a unified calibration .ini via
// saveCameraCalibration(). An existing .ini can be loaded to update; every
// section is independently skippable.
//
// Temperature offers two methods:
//   (a) known-temperature offset: point the camera at a surface of known
//       temperature (ice water, boiling water, a taped printer bed, ...) and
//       nudge only c0 so it reads correctly.
//   (b) a manual c0 offset: enter a fixed °C correction directly, no reference.
//
// The c0-only method exploits that c0 is a pure additive °C offset on the cK
// output (cK = round(100*(c0 + c1*raw) + 27315)). We grab live cK frames through
// the full pipeline with a provisional {c1, c0_start}, average a centered ROI to
// get T_measured, and set new_c0 = c0_start + (T_known - T_measured). Because the
// measurement runs the same drift/vignette pipeline used at runtime, the result
// is runtime-consistent.

#include "common/capture_helpers.hpp"
#include "common/detect_dead_pixels.hpp"
#include "common/fit_vignette.hpp"
#include "common/terminal_preview.hpp"
#include "openseekthermal/camera_calibration.hpp"
#include "openseekthermal/openseekthermal.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using namespace openseekthermal;
using namespace openseekthermal::tools;

namespace
{

constexpr double kCkToK = 27315.0; // cK at 0 °C
constexpr int kXRef = 0x4000;      // 16384, centering constant for the c1 domain

void printUsage( const char *argv0 )
{
  std::cout << "Usage: " << argv0 << " [options]\n"
            << "  Interactive calibration wizard for Seek Thermal cameras.\n"
            << "  --calibration PATH   existing unified .ini to load/update (also the\n"
            << "                         default save target). Optional.\n"
            << "  --serial S | --port P  device selector (otherwise prompts)\n"
            << "  -h | --help          show this message\n";
}

std::string isoNow()
{
  using namespace std::chrono;
  const std::time_t t = system_clock::to_time_t( system_clock::now() );
  std::tm tm{};
  gmtime_r( &t, &tm );
  std::ostringstream ss;
  ss << std::put_time( &tm, "%Y-%m-%dT%H:%M:%SZ" );
  return ss.str();
}

// Fixed centered measurement ROI (kept small so it stays near the optical
// center where vignette is weakest).
void centeredRoi( int width, int height, int &rx, int &ry, int &rw, int &rh )
{
  rw = std::min( 120, width );
  rh = std::min( 120, height );
  rx = ( width - rw ) / 2;
  ry = ( height - rh ) / 2;
}

// ---------------------------------------------------------------------------
// [1] Vignette

void doVignette( SeekThermalCamera &cam, CameraCalibration &cal, int width, int height, bool &dirty )
{
  // Fixed radial-model knobs (degree of the r² polynomial; the optical center is
  // always jointly fit). Not user-facing — the model is fixed, only the data
  // changes per capture.
  constexpr int kFrames = 50;
  constexpr int kWarmup = 30;
  constexpr int kDegree = 3;
  constexpr bool kFitCenter = true;

  const size_t pixel_count = static_cast<size_t>( width ) * height;
  std::cout << "\n--- Vignette calibration ---\n"
            << "Point the camera at a roughly-flat, uniform surface (a wall or a\n"
            << "sheet of cardboard). Avoid your own breath / hand in the view.\n";

  // Exclude any known dead pixels from the fit.
  std::vector<bool> dead_mask;
  size_t n_dead = 0;
  if ( cal.dead_pixels ) {
    dead_mask.assign( pixel_count, false );
    for ( const auto &e : cal.dead_pixels->entries() ) dead_mask[e.index] = true;
    n_dead = cal.dead_pixels->deadPixelCount();
    if ( n_dead >= pixel_count ) {
      std::cout << "All pixels are flagged dead — refusing to fit.\n";
      return;
    }
    std::cout << "Excluding " << n_dead << " dead pixels from the fit.\n";
  }

  if ( !livePreviewUntilStart( cam, std::cin, std::cout,
                               "Aim at a flat, uniform surface (wall / cardboard)." ) )
    return;

  // Capture in the raw-count domain the vignette correction operates in: disable
  // any previously-installed vignette (so the fit sees the true lens shading, not
  // the residual after an existing correction) and the substrate-drift
  // compensation (which runs after the vignette filter). Both, plus the original
  // `cal`, are restored on every exit path below.
  CameraCalibration capture_cal = cal;
  capture_cal.vignette.reset();
  const bool drift_was_enabled = cam.isDriftCompensationEnabled();
  try {
    cam.setCalibration( capture_cal );
  } catch ( const std::exception &e ) {
    std::cout << "Failed to prepare capture calibration: " << e.what() << "\n";
    return;
  }
  cam.setDriftCompensationEnabled( false );
  const auto restore_calibration = [&]() {
    cam.setDriftCompensationEnabled( drift_was_enabled );
    try {
      cam.setCalibration( cal );
    } catch ( const std::exception & ) {
    }
  };

  const bool preview = isInteractiveTerminal();
  std::string err;
  if ( !warmup( cam, kWarmup, err ) ) {
    std::cout << err << "\n";
    restore_calibration();
    return;
  }
  std::vector<uint64_t> sum;
  int skipped_cal = 0;
  if ( !accumulateAverage( cam, width, height, kFrames, sum, skipped_cal, "Capturing vignette",
                           &std::cout, preview, err ) ) {
    std::cout << err << "\n";
    restore_calibration();
    return;
  }
  std::vector<double> avg( pixel_count );
  for ( size_t i = 0; i < pixel_count; ++i )
    avg[i] = static_cast<double>( sum[i] ) / static_cast<double>( kFrames );

  RadialFit fit = fitRadialPolynomial( avg, width, height, kDegree, kFitCenter,
                                       dead_mask.empty() ? nullptr : &dead_mask );
  if ( fit.coeffs.empty() ) {
    std::cout << "Fit failed (degenerate normal equations) — vignette left unchanged.\n";
    restore_calibration();
    return;
  }

  // Summary stats vs. the fit (skip dead pixels).
  double res_sum_sq = 0.0, max_abs = 0.0, max_dev = 0.0, min_dev = 0.0;
  size_t live_count = 0;
  for ( int y = 0; y < height; ++y ) {
    for ( int x = 0; x < width; ++x ) {
      const size_t i = static_cast<size_t>( y ) * width + x;
      const double model = evaluate( fit, x, y );
      const double dev = model - fit.mean_model;
      max_dev = std::max( max_dev, dev );
      min_dev = std::min( min_dev, dev );
      if ( !dead_mask.empty() && dead_mask[i] )
        continue;
      const double r = avg[i] - model;
      res_sum_sq += r * r;
      max_abs = std::max( max_abs, std::abs( r ) );
      ++live_count;
    }
  }
  const double rms =
      live_count > 0 ? std::sqrt( res_sum_sq / static_cast<double>( live_count ) ) : 0.0;

  std::cout << "Fit: c0=" << fit.coeffs[0];
  for ( int k = 1; k <= fit.degree; ++k ) std::cout << "  c" << k << "=" << fit.coeffs[k];
  std::cout << "\n";
  std::cout << std::fixed << std::setprecision( 2 ) << "  center=(" << fit.cx << ", " << fit.cy
            << ")  edge-vs-centre swing=" << ( max_dev - min_dev ) << " counts\n"
            << "  residual RMS=" << rms << "  max|residual|=" << max_abs << " counts\n"
            << "  skipped " << skipped_cal << " calibration frames\n"
            << std::defaultfloat;
  if ( ( max_dev - min_dev ) < 200.0 )
    std::cout << "Note: radial swing is small — vignette may not be the dominant artifact, or\n"
                 "      the degree may be too low.\n";
  if ( max_abs > 5.0 * rms && rms > 0.0 )
    std::cout << "Note: large per-pixel residuals remain — likely non-radial scene structure or\n"
                 "      hot/dead pixels in the field.\n";

  cal.vignette = toVignetteCorrection( fit );
  dirty = true;
  // Install the finalized calibration (incl. the new vignette) so subsequent
  // previews and measurements reflect it.
  restore_calibration();
  std::cout << "Vignette updated.\n";
}

// ---------------------------------------------------------------------------
// [2] Dead pixels

void doDeadPixels( SeekThermalCamera &cam, const SeekDevice &device, CameraCalibration &cal,
                   int width, int height, bool &dirty )
{
  constexpr int kFrames = 200;
  constexpr int kWarmup = 30;
  std::cout << "\n--- Dead-pixel calibration ---\n"
            << ">>> PAN / SWEEP the camera across a varied scene for the entire capture. <<<\n"
            << "    Holding still or pointing at a uniform field hides stuck pixels.\n"
            << "    Flags only pixels that stay static or read 0 / 0xFFFF.\n";

  if ( !livePreviewUntilStart( cam, std::cin, std::cout,
                               "Aim, then PAN slowly across a varied scene during capture." ) )
    return;

  const bool preview = isInteractiveTerminal();
  std::string err;
  if ( !warmup( cam, kWarmup, err ) ) {
    std::cout << err << "\n";
    return;
  }
  RawPixelStats stats;
  if ( !captureWelfordRaw( cam, device, width, height, kFrames, stats, &std::cout, preview, err ) ) {
    std::cout << err << "\n";
    return;
  }

  const DeadPixelResult r = detectDeadPixels( stats, width, height );
  const size_t pixel_count = static_cast<size_t>( width ) * height;
  std::cout << std::fixed << std::setprecision( 2 );
  std::cout << "Dead pixels (stuck vs neighbours): " << r.total_dead << " / " << pixel_count << " ("
            << ( 100.0 * static_cast<double>( r.total_dead ) / static_cast<double>( pixel_count ) )
            << "%)\n"
            << "Panning activity (median local stddev): " << r.panning_activity << " counts\n"
            << std::defaultfloat;
  if ( r.sentinel_info > 0 )
    std::cout << "(" << r.sentinel_info
              << " pixels read 0/0xFFFF — inpainted by the driver at runtime, not masked)\n";

  if ( r.panning_activity < r.min_activity )
    std::cout << "Warning: low panning activity (" << r.panning_activity
              << " counts) — the scene barely changed. Stuck-pixel detection is\n"
                 "      unreliable; re-run while panning the camera across a varied scene.\n";

  // Coverage: pixels whose neighbourhood saw too little motion were NOT checked.
  // Reporting this prevents the user assuming the whole sensor was calibrated when
  // entire regions were never exercised.
  if ( r.unjudged > 0 ) {
    const double unjudged_pct =
        100.0 * static_cast<double>( r.unjudged ) / static_cast<double>( pixel_count );
    std::cout << "Warning: " << r.unjudged << " pixels (" << static_cast<int>( unjudged_pct )
              << "%) were in regions with too little motion to judge — these were NOT\n"
                 "      checked for stuck behaviour and are not covered by this calibration.\n"
                 "      Pan across those areas and re-run to cover the full sensor.\n";
  }

  std::vector<std::pair<int, int>> coords;
  coords.reserve( r.total_dead );
  for ( int y = 0; y < height; ++y ) {
    for ( int x = 0; x < width; ++x ) {
      if ( r.mask[static_cast<size_t>( y ) * width + x] != 0 )
        coords.emplace_back( x, y );
    }
  }
  cal.dead_pixels = DeadPixelMask( width, height, coords );
  dirty = true;
  std::cout << "Dead-pixel mask updated (" << coords.size() << " pixels).\n";
}

// ---------------------------------------------------------------------------
// Pin the drift anchor the fitted c0 was measured against onto the calibration,
// so the frozen absolute offset stays valid across reboots (setCalibration re-
// pins it instead of taking the per-boot first-thermal anchor). Only meaningful
// when drift compensation contributed to the live measurement; otherwise c0 is
// anchor-independent and pad_ref is left unset (NaN).
void pinDriftAnchor( const SeekThermalCamera &cam, TemperatureCalibration &t )
{
  if ( cam.isDriftCompensationEnabled() && cam.getDriftCompensationCoefficient() > 0.0 )
    t.pad_ref = cam.getDriftReferenceAnchor();
}

// [3a] Temperature: c0-only offset against a surface of known temperature

void doKnownTemperature( SeekThermalCamera &cam, CameraCalibration &cal, int width, int height,
                         bool &dirty )
{
  constexpr int kFrames = 60;
  constexpr int kWarmup = 30;
  int rx, ry, rw, rh;
  centeredRoi( width, height, rx, ry, rw, rh );

  std::cout << "\n--- Known-temperature offset (c0) ---\n"
            << "Adjusts only the offset so a surface of known temperature reads\n"
            << "correctly; keeps the existing/firmware gain (c1). Fill the ROI with a\n"
            << "stable, high known-emissivity (~0.95) target — avoid bare/shiny metal.\n"
            << "Examples: ice-water slush (0 °C), boiling water (~100 °C, lower at\n"
            << "altitude), a 3D-printer bed covered with matte black tape, or a stirred\n"
            << "water bath read with a thermometer.\n";
  const double t_known = promptDouble( std::cin, std::cout, "Known temperature (°C)", 20.0 );

  std::string err;
  // Seed {c1, c0_start} from the calibration the camera currently maps with, so
  // the provisional reading matches what the camera reports live.
  const TemperatureCalibration seed = *cam.calibration().temperature;
  const double c1 = seed.c1;
  const double c0_start = seed.c0;

  if ( !livePreviewUntilStart( cam, std::cin, std::cout,
                               "Fill the ROI with the reference (" +
                                   std::to_string( static_cast<int>( t_known ) ) + " °C)." ) )
    return;

  const bool preview = isInteractiveTerminal();

  // Install the provisional calibration so grabFrame applies {c1, c0_start}
  // (keeping any vignette / dead-pixel sections active).
  CameraCalibration provisional = cal;
  TemperatureCalibration t_prov;
  t_prov.c0 = c0_start;
  t_prov.c1 = c1;
  provisional.temperature = t_prov;
  try {
    cam.setCalibration( provisional );
  } catch ( const std::exception &e ) {
    std::cout << "Failed to install provisional calibration: " << e.what() << "\n";
    return;
  }

  if ( !warmup( cam, kWarmup, err ) ) {
    std::cout << err << "\n";
    return;
  }

  // Collect centered-ROI cK samples over THERMAL frames; the median is robust to
  // a few stray hot/cold pixels in the ROI.
  std::vector<float> samples;
  samples.reserve( static_cast<size_t>( rw ) * rh * kFrames );
  bool clamped = false;
  int accumulated = 0;
  if ( preview )
    std::cout << "\033[s"; // anchor the in-place preview below the printed instructions
  while ( accumulated < kFrames ) {
    FrameHeader header;
    unsigned char *buf = nullptr;
    size_t buf_size = 0;
    const GrabFrameResult res = cam.grabFrame( &buf, buf_size, &header );
    if ( res != GrabFrameResult::SUCCESS ) {
      delete[] buf;
      std::cout << "Capture grab failed: " << to_string( res ) << "\n";
      return;
    }
    // Skip non-thermal frames
    if ( header.getFrameType() != FrameType::THERMAL_FRAME ) {
      delete[] buf;
      continue;
    }
    const auto *px = reinterpret_cast<const uint16_t *>( buf );
    for ( int y = ry; y < ry + rh; ++y ) {
      for ( int x = rx; x < rx + rw; ++x ) {
        const uint16_t v = px[static_cast<size_t>( y ) * width + x];
        if ( v == 0 || v == 0xffff )
          clamped = true;
        samples.push_back( static_cast<float>( v ) );
      }
    }
    ++accumulated;
    if ( preview ) {
      renderFrameAnsi( std::cout, px, width, height, width, 48,
                       "Hold steady — capturing " + std::to_string( accumulated ) + " / " +
                           std::to_string( kFrames ),
                       true );
    }
    delete[] buf;
    if ( !preview && ( accumulated % 10 == 0 || accumulated == kFrames ) )
      std::cout << "  captured " << accumulated << " / " << kFrames << "\n";
  }
  if ( samples.empty() ) {
    std::cout << "No ROI pixels captured.\n";
    return;
  }

  const size_t mid = samples.size() / 2;
  std::nth_element( samples.begin(), samples.begin() + mid, samples.end() );
  const double median_ck = samples[mid];
  const double t_meas = ( median_ck - kCkToK ) / 100.0;
  if ( clamped )
    std::cout << "Warning: some ROI pixels saturated (0 / 65535). The additive c0 identity is\n"
                 "      invalid when clamping occurs; re-seed from a fresh calibration or move\n"
                 "      closer to the expected temperature.\n";

  const double new_c0 = c0_start + ( t_known - t_meas );
  TemperatureCalibration t;
  t.c0 = new_c0;
  t.c1 = c1;
  pinDriftAnchor( cam, t );
  cal.temperature = t;
  dirty = true;

  std::cout << std::fixed << std::setprecision( 3 );
  std::cout << "Measured " << t_meas << " °C, target " << t_known << " °C.\n"
            << "c0: " << c0_start << " -> " << new_c0 << "  (c1=" << c1 << ")\n"
            << std::defaultfloat;

  // Re-install the finalized calibration so subsequent actions reflect it.
  CameraCalibration finalized = cal;
  try {
    cam.setCalibration( finalized );
  } catch ( const std::exception & ) {
  }
  std::cout << "Temperature updated (c0-only).\n";
}

// ---------------------------------------------------------------------------
// [3c] Temperature: manual additive c0 offset
//
// Lets the user nudge the output by a fixed °C amount without a reference
// measurement — useful when readings are off by a known constant (e.g. against
// a trusted thermometer). The offset is added to the current c0 (or to a
// firmware-seeded c0 if no temperature cal is installed yet); c1 is untouched.

void doManualOffset( SeekThermalCamera &cam, CameraCalibration &cal, bool &dirty )
{
  std::cout << "\n--- Manual offset (c0) ---\n"
            << "Adds a fixed °C offset to the temperature output (adjusts c0 only,\n"
            << "keeps the existing/firmware gain c1). Positive raises the reading.\n"
            << "The offset is applied on top of the current calibration.\n";

  // Seed from the calibration the camera currently maps with (the installed cal
  // if any, otherwise the factory default derived at open()).
  const TemperatureCalibration seed = *cam.calibration().temperature;
  const double c1 = seed.c1;
  const double c0_start = seed.c0;

  const double offset = promptDouble( std::cin, std::cout, "Offset to add (°C)", 0.0 );
  if ( offset == 0.0 ) {
    std::cout << "Zero offset — temperature left unchanged.\n";
    return;
  }

  TemperatureCalibration t;
  t.c0 = c0_start + offset;
  t.c1 = c1;
  pinDriftAnchor( cam, t );
  cal.temperature = t;
  dirty = true;

  std::cout << std::fixed << std::setprecision( 3 ) << "c0: " << c0_start << " -> " << t.c0 << "  ("
            << ( offset >= 0 ? "+" : "" ) << offset << " °C, c1=" << c1 << ")\n"
            << std::defaultfloat;

  // Re-install so subsequent actions reflect the new offset.
  try {
    cam.setCalibration( cal );
  } catch ( const std::exception & ) {
  }
  std::cout << "Temperature updated (manual offset).\n";
}

void doTemperature( SeekThermalCamera &cam, CameraCalibration &cal, int width, int height,
                    bool &dirty )
{
  while ( true ) {
    std::cout << "\nTemperature method:\n"
              << "  [a] Known temperature (measure a known reference; adjusts offset)\n"
              << "  [b] Manual offset (enter a fixed °C offset; no reference)\n"
              << "  [c] back\n";
    const std::string choice = promptLine( std::cin, std::cout, "Select", "c" );
    if ( choice.empty() )
      return;
    switch ( choice[0] ) {
    case 'a':
      doKnownTemperature( cam, cal, width, height, dirty );
      return;
    case 'b':
      doManualOffset( cam, cal, dirty );
      return;
    case 'c':
      return;
    default:
      std::cout << "Unknown choice.\n";
    }
  }
}

// ---------------------------------------------------------------------------
// [4] Show current state

void showState( const CameraCalibration &cal, const SeekDevice &device, int width, int height )
{
  std::cout << "\n=== Current calibration state ===\n";
  std::cout << "Device: " << device << "  (" << width << "x" << height << ")\n";

  std::cout << "[temperature] ";
  if ( cal.temperature ) {
    const double example = cal.temperature->c0 + cal.temperature->c1 * kXRef;
    std::cout << "c0=" << cal.temperature->c0 << "  c1=" << cal.temperature->c1 << "  (raw "
              << kXRef << " -> " << example << " °C)\n";
  } else {
    std::cout << "not set — omitted on save\n";
  }

  std::cout << "[vignette] ";
  if ( cal.vignette ) {
    std::cout << cal.vignette->width << "x" << cal.vignette->height << "  center=(" << std::fixed
              << std::setprecision( 2 ) << cal.vignette->cx << ", " << cal.vignette->cy
              << ")  degree=" << cal.vignette->degree << "  mean_model=" << std::setprecision( 1 )
              << cal.vignette->mean_model << "\n"
              << std::defaultfloat;
  } else {
    std::cout << "not set — omitted on save\n";
  }

  std::cout << "[dead_pixels] ";
  if ( cal.dead_pixels ) {
    const size_t n = cal.dead_pixels->deadPixelCount();
    const size_t total = static_cast<size_t>( width ) * height;
    std::cout << n << " pixels (" << std::fixed << std::setprecision( 3 )
              << ( 100.0 * static_cast<double>( n ) / static_cast<double>( total ) ) << "%)\n"
              << std::defaultfloat;
  } else {
    std::cout << "not set — omitted on save\n";
  }
}

// ---------------------------------------------------------------------------
// [5] Save

void doSave( const CameraCalibration &cal, const SeekDevice &device, fs::path &save_path,
             const std::string &default_name, int width, int height, bool &dirty )
{
  const std::string def = save_path.empty() ? default_name : save_path.string();
  const std::string path_str = promptLine( std::cin, std::cout, "Save path", def );
  const fs::path path = path_str;

  std::ostringstream hdr;
  hdr << "openseekthermal interactive calibration\n"
      << "Generated at " << isoNow() << "\n"
      << "Device: " << device << "\n"
      << "Sections:";
  if ( cal.temperature )
    hdr << " temperature";
  if ( cal.vignette )
    hdr << " vignette";
  if ( cal.dead_pixels )
    hdr << " dead_pixels";

  try {
    saveCameraCalibration( path, cal, hdr.str() );
  } catch ( const std::exception &e ) {
    std::cout << "Failed to write " << path << ": " << e.what() << "\n";
    return;
  }
  std::cout << "Wrote " << path << "\n";
  save_path = path;
  dirty = false;

  // Round-trip self-check.
  try {
    loadCameraCalibration( path, width, height );
    std::cout << "Round-trip check: reloaded OK.\n";
  } catch ( const std::exception &e ) {
    std::cout << "Round-trip check FAILED: " << e.what() << "\n";
  }
}

} // namespace

int main( int argc, char **argv )
{
  std::string serial;
  std::string port;
  fs::path calibration_path;

  for ( int i = 1; i < argc; ++i ) {
    const std::string arg = argv[i];
    if ( ( arg == "--calibration" || arg == "-c" ) && i + 1 < argc ) {
      calibration_path = argv[++i];
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

  SeekDevice device;
  std::string err;
  if ( !serial.empty() || !port.empty() ) {
    if ( !pickDevice( serial, port, device, err ) ) {
      std::cerr << err << "\n";
      return 1;
    }
  } else if ( !pickDeviceInteractive( device, err, std::cin, std::cout ) ) {
    std::cerr << err << "\n";
    return 1;
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

  // Suggested calibration filename embeds the unit serial so per-unit files
  // don't collide. The Nano advertises its serial over USB; the Compact Pro's
  // is read from factory data during open().
  const std::string cam_serial = camera->getSerialNumber();
  const std::string default_calibration_name =
      cam_serial.empty() ? std::string( "calibration.ini" ) : "calibration_" + cam_serial + ".ini";

  CameraCalibration cal;
  fs::path save_path = calibration_path;
  bool dirty = false;

  // Load an existing calibration to update.
  if ( calibration_path.empty() ) {
    // Let the user point at a differently-named config; our serial-based name is
    // the default both for loading and as the save target.
    const std::string name =
        promptLine( std::cin, std::cout, "Calibration file", default_calibration_name );
    if ( fs::exists( name ) &&
         promptYesNo( std::cin, std::cout, "Found existing calibration '" + name + "'. Load it?",
                      true ) ) {
      calibration_path = name;
    } else {
      // Not loading anything — keep the chosen name as the save target.
      save_path = name;
    }
  }
  while ( !calibration_path.empty() ) {
    if ( !fs::exists( calibration_path ) ) {
      std::cout << "File does not exist: " << calibration_path << " — starting fresh.\n";
      save_path = calibration_path; // still use it as the default save target
      break;
    }
    try {
      cal = loadCameraCalibration( calibration_path, width, height );
      save_path = calibration_path;
      std::cout << "Loaded " << calibration_path << "\n";
      break;
    } catch ( const std::exception &e ) {
      std::cout << "Failed to load: " << e.what() << "\n";
      const std::string p =
          promptLine( std::cin, std::cout, "Try another path (blank to start fresh)", "" );
      calibration_path = p;
    }
  }

  // Apply whatever was loaded so live measurements reflect it.
  try {
    camera->setCalibration( cal );
  } catch ( const std::exception &e ) {
    std::cout << "Warning: could not apply loaded calibration: " << e.what() << "\n";
  }

  bool running = true;
  while ( running ) {
    std::cout << "\n=== openseekthermal interactive calibration ===  " << device << "  " << width
              << "x" << height << "\n"
              << "  [1] Calibrate vignette      (" << ( cal.vignette ? "set" : "not set" ) << ")\n"
              << "  [2] Calibrate dead pixels   ("
              << ( cal.dead_pixels ? std::to_string( cal.dead_pixels->deadPixelCount() ) + " px"
                                   : "not set" )
              << ")\n"
              << "  [3] Calibrate temperature   (" << ( cal.temperature ? "set" : "not set" ) << ")\n"
              << "  [4] Show current state\n"
              << "  [5] Save calibration" << ( dirty ? "  (unsaved changes)" : "" ) << "\n"
              << "  [q] Quit\n";
    const std::string choice = promptLine( std::cin, std::cout, "Select", "" );
    if ( !std::cin ) { // EOF / closed stdin — stop rather than spin
      std::cout << "\n";
      break;
    }
    if ( choice.empty() )
      continue;
    switch ( choice[0] ) {
    case '1':
      doVignette( *camera, cal, width, height, dirty );
      break;
    case '2':
      doDeadPixels( *camera, device, cal, width, height, dirty );
      break;
    case '3':
      doTemperature( *camera, cal, width, height, dirty );
      break;
    case '4':
      showState( cal, device, width, height );
      break;
    case '5':
      doSave( cal, device, save_path, default_calibration_name, width, height, dirty );
      break;
    case 'q':
    case 'Q':
      if ( dirty && !promptYesNo( std::cin, std::cout, "Quit without saving?", false ) )
        break;
      running = false;
      break;
    default:
      std::cout << "Unknown choice.\n";
    }
  }

  try {
    camera->close();
  } catch ( ... ) {
  }
  return 0;
}
