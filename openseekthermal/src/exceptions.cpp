// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "openseekthermal/detail/exceptions.hpp"
#include <libusb-1.0/libusb.h>

namespace openseekthermal
{
USBError::USBError( const std::string &msg, int error_code )
    : std::runtime_error( msg + ( error_code < 0
                                      ? std::string( " (" ) + libusb_error_name( error_code ) + ")"
                                      : "" ) ),
      error_code_( error_code )
{
}
} // namespace openseekthermal
