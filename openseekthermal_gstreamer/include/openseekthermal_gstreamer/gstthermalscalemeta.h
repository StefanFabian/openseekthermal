// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef OPENSEEKTHERMAL_GSTREAMER_THERMALSCALEMETA_HPP
#define OPENSEEKTHERMAL_GSTREAMER_THERMALSCALEMETA_HPP

#include <gst/gst.h>

G_BEGIN_DECLS

/**
 * GstThermalScaleMeta:
 * @meta:   parent #GstMeta
 * @scale:  factor applied during normalization
 * @offset: offset applied during normalization
 *
 * Attached by openseekthermalsrc to buffers whose 16-bit pixels were rescaled
 * away from the camera's native units (e.g. centi-Kelvin) via
 *   value = original * scale + offset
 * Downstream elements can recover the original value with
 *   original = ( value - offset ) / scale
 * If the meta is absent the buffer contents are assumed to be in the native
 * units already.
 */
typedef struct {
  GstMeta meta;
  gfloat scale;
  gfloat offset;
} GstThermalScaleMeta;

GType gst_thermal_scale_meta_api_get_type( void );
#define GST_THERMAL_SCALE_META_API_TYPE ( gst_thermal_scale_meta_api_get_type() )

const GstMetaInfo *gst_thermal_scale_meta_get_info( void );
#define GST_THERMAL_SCALE_META_INFO ( gst_thermal_scale_meta_get_info() )

#define gst_buffer_get_thermal_scale_meta( b )                                                     \
  ( (GstThermalScaleMeta *)gst_buffer_get_meta( ( b ), GST_THERMAL_SCALE_META_API_TYPE ) )

GstThermalScaleMeta *gst_buffer_add_thermal_scale_meta( GstBuffer *buffer, gfloat scale,
                                                        gfloat offset );

G_END_DECLS

#endif // OPENSEEKTHERMAL_GSTREAMER_THERMALSCALEMETA_HPP
