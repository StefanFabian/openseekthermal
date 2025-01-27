// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef OPENSEEKTHERMAL_FRAMERATE_HPP
#define OPENSEEKTHERMAL_FRAMERATE_HPP

namespace openseekthermal
{
struct Framerate {
  Framerate( int numerator, int denominator = 1 )
      : numerator( numerator ), denominator( denominator )
  {
  }

  float operator()() const
  {
    return static_cast<float>( numerator ) / static_cast<float>( denominator );
  }

  bool isInteger() const { return ( numerator / denominator ) * denominator == numerator; }

  int numerator;
  int denominator;
};
} // namespace openseekthermal

#endif // OPENSEEKTHERMAL_FRAMERATE_HPP
