// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef OPENSEEKTHERMAL_EXCEPTIONS_HPP
#define OPENSEEKTHERMAL_EXCEPTIONS_HPP

#include <stdexcept>

namespace openseekthermal
{

class InvalidDeviceError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

class SeekRuntimeError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

class SeekSetupError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

class USBError : public std::runtime_error
{
public:
  USBError( const std::string &msg, int error_code );

  int errorCode() const { return error_code_; }

private:
  int error_code_;
};
} // namespace openseekthermal

#endif // OPENSEEKTHERMAL_EXCEPTIONS_HPP
