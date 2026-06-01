// Copyright (c) 2026 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "fit_vignette.hpp"

#include <algorithm>
#include <cmath>

namespace openseekthermal::tools
{

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

RadialFit fitRadialPolynomial( const std::vector<double> &avg, int width, int height, int degree,
                               bool fit_center, const std::vector<bool> *dead_mask )
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

} // namespace openseekthermal::tools
