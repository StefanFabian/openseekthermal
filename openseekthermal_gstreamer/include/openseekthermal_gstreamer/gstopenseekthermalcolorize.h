// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef OPENSEEKTHERMAL_GSTREAMER_OPENSEEKTHERMALCOLORIZE_HPP
#define OPENSEEKTHERMAL_GSTREAMER_OPENSEEKTHERMALCOLORIZE_HPP

#include <cairo/cairo.h>
#include <gst/video/gstvideofilter.h>

G_BEGIN_DECLS

#define GST_TYPE_OPENSEEKTHERMALCOLORIZE ( gst_openseekthermalcolorize_get_type() )
#define GST_OPENSEEKTHERMALCOLORIZE( obj )                                                         \
  ( G_TYPE_CHECK_INSTANCE_CAST( ( obj ), GST_TYPE_OPENSEEKTHERMALCOLORIZE,                         \
                                GstOpenSeekThermalColorize ) )
#define GST_TYPE_OPENSEEKTHERMALCOLORIZE_CLASS( klass )                                            \
  ( G_TYPE_CHECK_CLASS_CAST( ( klass ), GST_TYPE_OPENSEEKTHERMALCOLORIZE,                          \
                             GstOpenSeekThermalColorize ) )
#define GST_IS_OPENSEEKTHERMALCOLORIZE( obj )                                                      \
  ( G_TYPE_CHECK_INSTANCE_TYPE( ( obj ), GST_TYPE_OPENSEEKTHERMALCOLORIZE ) )
#define GST_IS_OPENSEEKTHERMALCOLORIZE_CLASS( obj )                                                \
  ( G_TYPE_CHECK_CLASS_TYPE( ( klass ), GST_TYPE_OPENSEEKTHERMALCOLORIZE ) )

#define GST_TYPE_OPENSEEKTHERMAL_COLORMAP ( gst_openseekthermal_colormap_get_type() )

typedef enum {
  GST_OPENSEEKTHERMAL_COLORMAP_IRON = 0,
  GST_OPENSEEKTHERMAL_COLORMAP_GRAYSCALE,
  GST_OPENSEEKTHERMAL_COLORMAP_JET,
  GST_OPENSEEKTHERMAL_COLORMAP_INFERNO
} GstOpenSeekThermalColormap;

GType gst_openseekthermal_colormap_get_type( void );

typedef struct _GstOpenSeekThermalColorize GstOpenSeekThermalColorize;
typedef struct _GstOpenSeekThermalColorizeClass GstOpenSeekThermalColorizeClass;

struct _GstOpenSeekThermalColorize {
  GstVideoFilter parent;

  /* properties */
  gint min_centikelvin; /* -1 = auto */
  gint max_centikelvin; /* -1 = auto */
  GstOpenSeekThermalColormap colormap;
  guint roi_width;
  guint roi_height;
  guint auto_range_frames;

  /* derived per-format state, refreshed in set_info */
  gint out_format; /* GstVideoFormat as int to avoid header include */
  gint bytes_per_pixel;
  gint r_offset;
  gint g_offset;
  gint b_offset;
  gint a_offset; /* -1 if no alpha */

  /* colormap LUT, built when colormap changes */
  guint8 lut[256 * 3];
  GstOpenSeekThermalColormap lut_for;

  /* rolling auto-range state */
  guint *min_values;
  guint *max_values;
  guint *sort_value_buffer;
  guint count_values;
  guint index;
  gboolean first_frame;

  /* full-frame ARGB32 scratch + cairo surface, reused across frames */
  guint8 *scratch;
  gint scratch_w;
  gint scratch_h;
  gint scratch_stride;
  cairo_surface_t *scratch_surface;
};

struct _GstOpenSeekThermalColorizeClass {
  GstVideoFilterClass parent_class;
};

GType gst_openseekthermalcolorize_get_type( void );

G_END_DECLS

#endif // OPENSEEKTHERMAL_GSTREAMER_OPENSEEKTHERMALCOLORIZE_HPP
