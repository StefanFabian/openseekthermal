// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef OPENSEEKTHERMAL_GSTREAMER_OPENSEEKTHERMALSRC_HPP
#define OPENSEEKTHERMAL_GSTREAMER_OPENSEEKTHERMALSRC_HPP

#include <gst/base/gstpushsrc.h>
#include <gst/video/video-format.h>
#include <openseekthermal/openseekthermal.hpp>

G_BEGIN_DECLS

#define GST_TYPE_OPENSEEKTHERMALSRC ( gst_openseekthermalsrc_get_type() )
#define GST_OPENSEEKTHERMALSRC( obj )                                                              \
  ( G_TYPE_CHECK_INSTANCE_CAST( ( obj ), GST_TYPE_OPENSEEKTHERMALSRC, GstOpenSeekThermalSrc ) )
#define GST_TYPE_OPENSEEKTHERMALSRC_CLASS( klass )                                                 \
  ( G_TYPE_CHECK_CLASS_CAST( ( klass ), GST_TYPE_OPENSEEKTHERMALSRC, GstOpenSeekThermalSrc ) )
#define GST_IS_OPENSEEKTHERMALSRC( obj )                                                           \
  ( G_TYPE_CHECK_INSTANCE_TYPE( ( obj ), GST_TYPE_OPENSEEKTHERMALSRC ) )
#define GST_IS_OPENSEEKTHERMALSRC_CLASS( obj )                                                     \
  ( G_TYPE_CHECK_CLASS_TYPE( ( klass ), GST_TYPE_OPENSEEKTHERMALSRC ) )

typedef struct _GstOpenSeekThermalSrc GstOpenSeekThermalSrc;
typedef struct _GstOpenSeekThermalSrcClass GstOpenSeekThermalSrcClass;

struct _GstOpenSeekThermalSrc {
  GstPushSrc parent;
  gchar *serial;
  gchar *port;
  gboolean skip_invalid_frames;
  gboolean normalize;
  guint normalize_frame_count;
  openseekthermal::SeekThermalCamera::SharedPtr camera;

  gboolean first_frame;
  guint index;
  guint count_values;
  guint *min_values;
  guint *max_values;
  guint *sort_value_buffer;
};

struct _GstOpenSeekThermalSrcClass {
  GstPushSrcClass parent_class;

  // stick member function pointers here
  // along with member function pointers for signal handlers
};

GType gst_openseekthermalsrc_get_type( void );

G_END_DECLS

#endif // OPENSEEKTHERMAL_GSTREAMER_OPENSEEKTHERMALSRC_HPP
