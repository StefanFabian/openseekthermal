// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <gst/gst.h>
#include <openseekthermal_gstreamer/gstopenseekthermalsrc.h>

static gboolean plugin_init( GstPlugin *plugin )
{
  return gst_element_register( plugin, "openseekthermalsrc", GST_RANK_NONE,
                               GST_TYPE_OPENSEEKTHERMALSRC );
}

#ifndef VERSION
#define VERSION "1.0.0"
#endif
#ifndef PACKAGE
#define PACKAGE "openseekthermal_gstreamer"
#endif
#ifndef PACKAGE_NAME
#define PACKAGE_NAME "openseekthermal_gstreamer"
#endif
#ifndef GST_PACKAGE_ORIGIN
#define GST_PACKAGE_ORIGIN "https://github.com/StefanFabian/openseekthermal"
#endif

GST_PLUGIN_DEFINE( GST_VERSION_MAJOR, GST_VERSION_MINOR, openseekthermal_gstreamer,
                   "OpenSeekThermal camera driver", plugin_init, VERSION, "LGPL", PACKAGE_NAME,
                   GST_PACKAGE_ORIGIN )
