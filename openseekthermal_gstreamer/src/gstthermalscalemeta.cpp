// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "openseekthermal_gstreamer/gstthermalscalemeta.h"

GType gst_thermal_scale_meta_api_get_type( void )
{
  static GType type = 0;
  static const gchar *tags[] = { NULL };
  if ( g_once_init_enter( &type ) ) {
    GType _type = gst_meta_api_type_register( "GstThermalScaleMetaAPI", tags );
    g_once_init_leave( &type, _type );
  }
  return type;
}

static gboolean gst_thermal_scale_meta_init( GstMeta *meta, gpointer, GstBuffer * )
{
  auto *m = reinterpret_cast<GstThermalScaleMeta *>( meta );
  m->scale = 1.0f;
  m->offset = 0.0f;
  return TRUE;
}

static gboolean gst_thermal_scale_meta_transform( GstBuffer *dest, GstMeta *meta, GstBuffer *,
                                                  GQuark type, gpointer )
{
  auto *src_meta = reinterpret_cast<GstThermalScaleMeta *>( meta );
  if ( GST_META_TRANSFORM_IS_COPY( type ) ) {
    gst_buffer_add_thermal_scale_meta( dest, src_meta->scale, src_meta->offset );
  }
  return TRUE;
}

const GstMetaInfo *gst_thermal_scale_meta_get_info( void )
{
  static const GstMetaInfo *info = NULL;
  if ( g_once_init_enter( &info ) ) {
    const GstMetaInfo *m = gst_meta_register(
        gst_thermal_scale_meta_api_get_type(), "GstThermalScaleMeta", sizeof( GstThermalScaleMeta ),
        gst_thermal_scale_meta_init, NULL, gst_thermal_scale_meta_transform );
    g_once_init_leave( &info, m );
  }
  return info;
}

GstThermalScaleMeta *gst_buffer_add_thermal_scale_meta( GstBuffer *buffer, gfloat scale,
                                                        gfloat offset )
{
  auto *m = reinterpret_cast<GstThermalScaleMeta *>(
      gst_buffer_add_meta( buffer, GST_THERMAL_SCALE_META_INFO, NULL ) );
  m->scale = scale;
  m->offset = offset;
  return m;
}
