/**
 * SECTION:element-openseekthermalsrc
 * @title: openseekthermalsrc
 *
 * openseekthermalsrc can be used to capture video from SeekThermal devices.
 * Currently supported are:
 * - SeekThermal Compact Pro FF
 * - SeekThermal Nano300
 *
 * ## Example launch lines
 * |[
 * gst-launch-1.0 openseekthermalsrc ! videoconvert ! autovideosink
 * ]| This pipeline shows the thermal live stream from the first connected SeekThermal camera.
 * |[
 * gst-launch-1.0 openseekthermalsrc port=1-2.1 ! videoconvert ! autovideosink
 * ]| This pipeline shows the thermal live stream captured from the camera connected to USB bus 1,
 *    and port 1 on a hub connected to port 2.
 *
 */
// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include "openseekthermal_gstreamer/gstopenseekthermalsrc.h"

#include <openseekthermal/detail/exceptions.hpp>

#include <algorithm>
#include <gst/video/gstvideometa.h>
#include <gst/video/gstvideopool.h>
#include <numeric>
#include <string.h>

GST_DEBUG_CATEGORY_STATIC( openseekthermalsrc_debug );
#define GST_CAT_DEFAULT openseekthermalsrc_debug

enum {
  PROP_0,
  PROP_SERIAL,
  PROP_PORT,
  PROP_SKIP_INVALID_FRAMES,
  PROP_NORMALIZE,
  PROP_NORMALIZE_FRAME_COUNT,
  PROP_LAST
};

/* pad templates */
#define OPENSEEKTHERMALSRC_CAPS                                                                    \
  "video/x-raw, "                                                                                  \
  "format = { Y16, GRAY16_LE }, "                                                                  \
  "width = " GST_VIDEO_SIZE_RANGE ", "                                                             \
  "height = " GST_VIDEO_SIZE_RANGE ", "                                                            \
  "framerate = 0/1 "
static GstStaticPadTemplate gst_openseekthermalsrc_src_template = GST_STATIC_PAD_TEMPLATE(
    "src", GST_PAD_SRC, GST_PAD_ALWAYS, GST_STATIC_CAPS( OPENSEEKTHERMALSRC_CAPS ) );

G_DEFINE_TYPE_WITH_CODE( GstOpenSeekThermalSrc, gst_openseekthermalsrc, GST_TYPE_PUSH_SRC,
                         GST_DEBUG_CATEGORY_INIT( openseekthermalsrc_debug, "openseekthermalsrc",
                                                  0, "Debug category for openseekthermalsrc element" ) )

static void gst_openseekthermalsrc_finalize( GstOpenSeekThermalSrc *ostsrc );

/* element methods */
static GstStateChangeReturn gst_openseekthermalsrc_change_state( GstElement *element,
                                                                 GstStateChange transition );

/* basesrc methods */
static gboolean gst_openseekthermalsrc_open( GstOpenSeekThermalSrc *src );

static gboolean gst_openseekthermalsrc_close( GstOpenSeekThermalSrc *src );

static GstCaps *gst_openseekthermalsrc_get_caps( GstBaseSrc *src, GstCaps *filter );

static gboolean gst_openseekthermalsrc_query( GstBaseSrc *src, GstQuery *query );

static GstFlowReturn gst_openseekthermalsrc_create( GstPushSrc *src, GstBuffer **out );

static void gst_openseekthermalsrc_set_property( GObject *object, guint prop_id,
                                                 const GValue *value, GParamSpec *pspec );

static void gst_openseekthermalsrc_get_property( GObject *object, guint prop_id, GValue *value,
                                                 GParamSpec *pspec );

static void gst_openseekthermalsrc_class_init( GstOpenSeekThermalSrcClass *klass )
{
  GObjectClass *gobject_class = G_OBJECT_CLASS( klass );
  GstElementClass *element_class = GST_ELEMENT_CLASS( klass );
  GstBaseSrcClass *basesrc_class = GST_BASE_SRC_CLASS( klass );
  GstPushSrcClass *pushsrc_class = GST_PUSH_SRC_CLASS( klass );

  gobject_class->finalize = (GObjectFinalizeFunc)gst_openseekthermalsrc_finalize;
  gobject_class->set_property = gst_openseekthermalsrc_set_property;
  gobject_class->get_property = gst_openseekthermalsrc_get_property;

  g_object_class_install_property(
      gobject_class, PROP_SERIAL,
      g_param_spec_string(
          "serial", "Serial Number",
          "Serial of the SeekThermal serial to use. Only SeekThermal Nanos seem to have one.", "",
          (GParamFlags)( G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS ) ) );
  g_object_class_install_property(
      gobject_class, PROP_PORT,
      g_param_spec_string(
          "port", "Port",
          "The USB port of the SeekThermal serial to use. If the serial is set, this is ignored. "
          "E.g. 1-3.2 for Bus 1, Hub on port 3 and port 2 on the hub. When opening, the port is "
          "printed as info.",
          "", (GParamFlags)( G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS ) ) );
  g_object_class_install_property(
      gobject_class, PROP_SKIP_INVALID_FRAMES,
      g_param_spec_boolean(
          "skip-invalid-frames", "Skip Invalid Frames",
          "If enabled, invalid frames (e.g. during shutter calibration) will be skipped.", TRUE,
          (GParamFlags)( G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS ) ) );

  g_object_class_install_property(
      gobject_class, PROP_NORMALIZE,
      g_param_spec_boolean(
          "normalize",
          "Normalize", "Normalize the thermal data. If enabled will scale the pixel values to use the full range of the datatype.",
          TRUE, (GParamFlags)( G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS ) ) );
  g_object_class_install_property(
      gobject_class, PROP_NORMALIZE_FRAME_COUNT,
      g_param_spec_uint( "normalize-frame-count", "Normalize Frame Count",
                         "Number of frames to use for normalization.", 1, 25 * 60 * 60, 8,
                         (GParamFlags)( G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS ) ) );

  element_class->change_state = gst_openseekthermalsrc_change_state;

  gst_element_class_set_static_metadata( element_class, "OpenSeekThermal Video Source",
                                         "Source/Video",
                                         "Reads thermal image frames from a SeekThermal camera",
                                         "Stefan Fabian <gstreamer@stefanfabian.com>" );

  gst_element_class_add_static_pad_template( element_class, &gst_openseekthermalsrc_src_template );

  basesrc_class->get_caps = GST_DEBUG_FUNCPTR( gst_openseekthermalsrc_get_caps );
  basesrc_class->query = GST_DEBUG_FUNCPTR( gst_openseekthermalsrc_query );

  pushsrc_class->create = GST_DEBUG_FUNCPTR( gst_openseekthermalsrc_create );

  GST_DEBUG_CATEGORY_INIT( openseekthermalsrc_debug, "openseekthermalsrc", 0,
                           "OpenSeekThermal source element" );
}

void resize_value_buffers( GstOpenSeekThermalSrc *ostsrc, guint new_size )
{
  if ( ostsrc->count_values == new_size )
    return;
  ostsrc->min_values = (guint *)g_realloc( ostsrc->min_values, new_size * sizeof( guint ) );
  ostsrc->max_values = (guint *)g_realloc( ostsrc->max_values, new_size * sizeof( guint ) );
  ostsrc->sort_value_buffer =
      (guint *)g_realloc( ostsrc->sort_value_buffer, new_size * sizeof( guint ) );
  guint source_index = 0;
  for ( guint i = ostsrc->count_values; i < new_size; i++ ) {
    ostsrc->min_values[i] = ostsrc->min_values[source_index];
    ostsrc->max_values[i] = ostsrc->max_values[source_index];
    if ( ++source_index == ostsrc->count_values )
      source_index = 0;
  }
  ostsrc->count_values = new_size;
}

static void gst_openseekthermalsrc_init( GstOpenSeekThermalSrc *ostsrc )
{
  ostsrc->skip_invalid_frames = TRUE;
  ostsrc->normalize = TRUE;
  ostsrc->normalize_frame_count = 8;

  ostsrc->serial = g_strdup( "" );
  ostsrc->port = g_strdup( "" );
  ostsrc->camera = nullptr;

  ostsrc->min_values = nullptr;
  ostsrc->max_values = nullptr;
  ostsrc->sort_value_buffer = nullptr;
  ostsrc->count_values = 0;
  resize_value_buffers( ostsrc, ostsrc->normalize_frame_count );
  ostsrc->index = 0;
  ostsrc->first_frame = TRUE;
  gst_base_src_set_format( GST_BASE_SRC( ostsrc ), GST_FORMAT_TIME );
  gst_base_src_set_live( GST_BASE_SRC( ostsrc ), TRUE );
  gst_base_src_set_do_timestamp( GST_BASE_SRC( ostsrc ), TRUE );
  GST_DEBUG_OBJECT( ostsrc, "OpenSeekThermalSrc initialized" );
}

static void gst_openseekthermalsrc_finalize( GstOpenSeekThermalSrc *ostsrc )
{
  g_free( ostsrc->serial );
  ostsrc->serial = nullptr;
  g_free( ostsrc->port );
  ostsrc->port = nullptr;
  g_free_sized( ostsrc->min_values, ostsrc->count_values * sizeof( guint ) );
  ostsrc->min_values = nullptr;
  g_free_sized( ostsrc->max_values, ostsrc->count_values * sizeof( guint ) );
  ostsrc->max_values = nullptr;
  g_free_sized( ostsrc->sort_value_buffer, ostsrc->count_values * sizeof( guint ) );
  ostsrc->sort_value_buffer = nullptr;

  G_OBJECT_CLASS( gst_openseekthermalsrc_parent_class )->finalize( (GObject *)( ostsrc ) );
}

static void gst_openseekthermalsrc_set_property( GObject *object, guint prop_id,
                                                 const GValue *value, GParamSpec *pspec )
{
  GstOpenSeekThermalSrc *ostsrc = GST_OPENSEEKTHERMALSRC( object );

  switch ( prop_id ) {
  case PROP_SERIAL:
    g_free( ostsrc->serial );
    ostsrc->serial = g_value_dup_string( value );
    GST_DEBUG_OBJECT( ostsrc, "Device set to %s", ostsrc->serial );
    break;
  case PROP_PORT:
    g_free( ostsrc->port );
    ostsrc->port = g_value_dup_string( value );
    GST_DEBUG_OBJECT( ostsrc, "Port set to %s", ostsrc->port );
    break;
  case PROP_SKIP_INVALID_FRAMES:
    ostsrc->skip_invalid_frames = g_value_get_boolean( value );
    GST_DEBUG_OBJECT( ostsrc, "Skip invalid frames set to %d", ostsrc->skip_invalid_frames );
    break;
  case PROP_NORMALIZE:
    ostsrc->normalize = g_value_get_boolean( value );
    GST_DEBUG_OBJECT( ostsrc, "Normalize set to %d", ostsrc->normalize );
    break;
  case PROP_NORMALIZE_FRAME_COUNT:
    ostsrc->normalize_frame_count = g_value_get_uint( value );
    GST_DEBUG_OBJECT( ostsrc, "Normalize frame count set to %d", ostsrc->normalize );
    if ( ostsrc->normalize_frame_count == 0 ) {
      GST_WARNING_OBJECT( ostsrc, "Normalize frame count can't be 0. Setting to 1." );
      ostsrc->normalize_frame_count = 1;
    } else if ( ostsrc->normalize_frame_count > 16383 ) {
      GST_WARNING_OBJECT( ostsrc,
                          "Normalize frame count can't be larger than 16383. Setting to 16383." );
      ostsrc->normalize_frame_count = 16383;
    }
    resize_value_buffers( ostsrc, ostsrc->normalize_frame_count );
    ostsrc->index = 0;
    break;
  default:
    G_OBJECT_WARN_INVALID_PROPERTY_ID( object, prop_id, pspec );
    break;
  }
}

static void gst_openseekthermalsrc_get_property( GObject *object, guint prop_id, GValue *value,
                                                 GParamSpec *pspec )
{
  GstOpenSeekThermalSrc *ostsrc = GST_OPENSEEKTHERMALSRC( object );

  switch ( prop_id ) {
  case PROP_SERIAL:
    g_value_set_string( value, ostsrc->serial );
    break;
  case PROP_PORT:
    g_value_set_string( value, ostsrc->port );
    break;
  case PROP_SKIP_INVALID_FRAMES:
    g_value_set_boolean( value, ostsrc->skip_invalid_frames );
    break;
  case PROP_NORMALIZE:
    g_value_set_boolean( value, ostsrc->normalize );
    break;
  case PROP_NORMALIZE_FRAME_COUNT:
    g_value_set_uint( value, ostsrc->normalize_frame_count );
    break;
  default:
    G_OBJECT_WARN_INVALID_PROPERTY_ID( object, prop_id, pspec );
    break;
  }
}

static GstCaps *gst_openseekthermalsrc_get_caps( GstBaseSrc *src, GstCaps * )
{
  GstOpenSeekThermalSrc *ostsrc = GST_OPENSEEKTHERMALSRC( src );

  if ( ostsrc->camera == nullptr ) {
    GST_INFO_OBJECT( src, "No camera available yet. Returning template caps." );
    return gst_pad_get_pad_template_caps( GST_BASE_SRC_PAD( ostsrc ) );
  }

  auto width = static_cast<gint>( ostsrc->camera->getFrameWidth() );
  auto height = static_cast<gint>( ostsrc->camera->getFrameHeight() );
  GstCaps *caps = gst_caps_new_simple( "video/x-raw", "format", G_TYPE_STRING, "GRAY16_LE", "width",
                                       G_TYPE_INT, width, "height", G_TYPE_INT, height, "framerate",
                                       GST_TYPE_FRACTION, 0, 1, NULL );
  gchar *caps_str = gst_caps_to_string( caps );
  GST_INFO_OBJECT( src, "Camera caps: %s", caps_str );
  g_free( caps_str );
  return caps;
}

static gboolean gst_openseekthermalsrc_query( GstBaseSrc *src, GstQuery *query )
{
  GstOpenSeekThermalSrc *ostsrc = GST_OPENSEEKTHERMALSRC( src );

  switch ( GST_QUERY_TYPE( query ) ) {
  case GST_QUERY_LATENCY: {
    // Camera must be open to give latency
    if ( ostsrc->camera == nullptr ) {
      GST_WARNING_OBJECT( ostsrc, "Can't give latency since serial isn't open !" );
      return FALSE;
    }

    auto framerate = ostsrc->camera->getMaxFramerate();
    // Since this is live min latency is always time to capture one frame which is the inverse of the framerate
    GstClockTime min_latency =
        gst_util_uint64_scale_int( GST_SECOND, framerate.denominator, framerate.numerator );

    // max latency is the time per frame times the maximum number of frames in the buffer
    GstClockTime max_latency;
    guint num_buffers = 0;
    if ( num_buffers == 0 )
      max_latency = -1;
    else
      max_latency = num_buffers * min_latency;

    GST_DEBUG_OBJECT( src, "report latency min %" GST_TIME_FORMAT " max %" GST_TIME_FORMAT,
                      GST_TIME_ARGS( min_latency ), GST_TIME_ARGS( max_latency ) );

    gst_query_set_latency( query, TRUE, min_latency, max_latency );

    return TRUE;
  }
  default:
    break;
  }
  return GST_BASE_SRC_CLASS( gst_openseekthermalsrc_parent_class )->query( src, query );
}

static GstStateChangeReturn gst_openseekthermalsrc_change_state( GstElement *element,
                                                                 GstStateChange transition )
{
  GstStateChangeReturn ret = GST_STATE_CHANGE_SUCCESS;
  GstOpenSeekThermalSrc *ostsrc = GST_OPENSEEKTHERMALSRC( element );

  switch ( transition ) {
  case GST_STATE_CHANGE_NULL_TO_READY:
    if ( !gst_openseekthermalsrc_open( ostsrc ) ) {
      return GST_STATE_CHANGE_FAILURE;
    }
    break;
  default:
    break;
  }

  ret = GST_ELEMENT_CLASS( gst_openseekthermalsrc_parent_class )->change_state( element, transition );

  switch ( transition ) {
  case GST_STATE_CHANGE_READY_TO_NULL:
    if ( !gst_openseekthermalsrc_close( ostsrc ) )
      return GST_STATE_CHANGE_FAILURE;
    break;
  default:
    break;
  }

  return ret;
}

guint get_stable_mean( guint *values, guint count, guint *sort_container )
{
  std::partial_sort_copy( values, values + count, sort_container, sort_container + count );
  return sort_container[count / 2];
}

void update_normalization_factor( GstOpenSeekThermalSrc *ostsrc, guint min_value, guint max_value,
                                  gfloat &scale, gfloat &offset )
{
  if ( ostsrc->count_values == 0 )
    return;
  if ( ostsrc->first_frame ) {
    std::fill( ostsrc->min_values, ostsrc->min_values + ostsrc->count_values, min_value );
    std::fill( ostsrc->max_values, ostsrc->max_values + ostsrc->count_values, max_value );
    ostsrc->first_frame = FALSE;
  }

  if ( ++ostsrc->index >= ostsrc->count_values )
    ostsrc->index = 0;
  ostsrc->min_values[ostsrc->index] = min_value;
  ostsrc->max_values[ostsrc->index] = max_value;

  guint median_max =
      get_stable_mean( ostsrc->max_values, ostsrc->count_values, ostsrc->sort_value_buffer );
  guint median_min =
      get_stable_mean( ostsrc->min_values, ostsrc->count_values, ostsrc->sort_value_buffer );
  if ( median_min == median_max ) {
    scale = 1.0f;
    offset = 0;
    return;
  }
  scale = 65535.0f / static_cast<gfloat>( median_max - median_min );
  offset = -static_cast<gfloat>( median_min ) * scale;
}

static GstFlowReturn gst_openseekthermalsrc_create( GstPushSrc *src, GstBuffer **buf )
{
  GstOpenSeekThermalSrc *ostsrc = GST_OPENSEEKTHERMALSRC( src );
  if ( ostsrc->camera == nullptr ) {
    GST_ERROR_OBJECT( ostsrc, "Camera not available yet!" );
    return GST_FLOW_ERROR;
  }
  GstFlowReturn ret = GST_BASE_SRC_CLASS( gst_openseekthermalsrc_parent_class )
                          ->alloc( GST_BASE_SRC( src ), 0, ostsrc->camera->getFrameSize(), buf );
  if ( G_UNLIKELY( ret != GST_FLOW_OK ) ) {
    GST_ERROR_OBJECT( ostsrc, "Failed to allocate buffer of %lu bytes",
                      ostsrc->camera->getFrameSize() );
    return ret;
  }

  GstMapInfo map;
  gst_buffer_map( *buf, &map, GST_MAP_WRITE );
  openseekthermal::FrameHeader header;
  int tries = 1;
  const int MAX_TRIES = 10;
  while ( tries < MAX_TRIES ) {
    try {
      auto result = ostsrc->camera->grabFrame( &map.data, map.size, &header );
      if ( result != openseekthermal::GrabFrameResult::SUCCESS ) {
        GST_ERROR_OBJECT( ostsrc, "Failed to grab frame: %s",
                          openseekthermal::to_string( result ).c_str() );
        ++tries;
        continue;
      }
    } catch ( const openseekthermal::USBError &e ) {
      GST_ERROR_OBJECT( ostsrc, "Failed to grab frame: %s. Retrying.", e.what() );
      ++tries;
      continue;
    }
    if ( !ostsrc->skip_invalid_frames ||
         header.getFrameType() == openseekthermal::FrameType::THERMAL_FRAME ) {
      if ( !ostsrc->first_frame )
        ++tries;
      break;
    }
    GST_DEBUG_OBJECT( ostsrc, "Received non-thermal frame. Skipping. Type: %s",
                      openseekthermal::to_string( header.getFrameType() ).c_str() );
  }
  if ( tries >= MAX_TRIES ) {
    GST_ERROR_OBJECT( ostsrc, "Failed to get a valid frame in %d tries.", tries );
    gst_buffer_unmap( *buf, &map );
    return GST_FLOW_ERROR;
  }
  ostsrc->first_frame = FALSE;

  // Got a valid frame
  if ( ostsrc->normalize ) {
    gfloat scale = 1;
    gfloat offset = 0;
    guint min_value = 65535;
    guint max_value = 0;
    for ( guint i = 0; i < map.size / 2; ++i ) {
      guint value = le16toh( reinterpret_cast<uint16_t *>( map.data )[i] );
      min_value = std::min( min_value, value );
      max_value = std::max( max_value, value );
    }
    update_normalization_factor( ostsrc, min_value, max_value, scale, offset );
    GST_DEBUG_OBJECT( ostsrc, "Normalization: scale=%f, offset=%f", scale, offset );

    auto *data = reinterpret_cast<uint16_t *>( map.data );
    gsize size = map.size / 2;
    for ( gsize i = 0; i < size; ++i ) {
      gfloat value = std::max( 0.f, std::min( 65535.f, le16toh( data[i] ) * scale + offset ) );
      data[i] = htole16( static_cast<uint16_t>( value ) );
    }
  }
  gst_buffer_unmap( *buf, &map );
  return GST_FLOW_OK;
}

static gboolean gst_openseekthermalsrc_open( GstOpenSeekThermalSrc *src )
{
  auto cameras = openseekthermal::listDevices();
  if ( cameras.empty() ) {
    GST_ERROR_OBJECT( src, "No SeekThermal cameras found!" );
    return FALSE;
  }
  openseekthermal::SeekDevice device = cameras[0];
  if ( src->serial != nullptr && strlen( src->serial ) > 0 ) {
    auto it =
        std::find_if( cameras.begin(), cameras.end(), [src]( const openseekthermal::SeekDevice &d ) {
          return d.serial == src->serial;
        } );
    if ( it == cameras.end() ) {
      GST_ERROR_OBJECT( src, "Device with serial '%s' not found!", src->serial );
      return FALSE;
    }
    device = *it;
  } else if ( src->port != nullptr && strlen( src->port ) > 0 ) {
    auto it =
        std::find_if( cameras.begin(), cameras.end(), [src]( const openseekthermal::SeekDevice &d ) {
          return d.usb_port == src->port;
        } );
    if ( it == cameras.end() ) {
      GST_ERROR_OBJECT( src, "Device with port '%s' not found!", src->port );
      return FALSE;
    }
    device = *it;
  }

  try {
    src->camera = openseekthermal::createCamera( device );
    src->camera->open();
    src->first_frame = TRUE;
    src->serial = g_strdup( device.serial.c_str() );
    src->port = g_strdup( device.usb_port.c_str() );
    GST_INFO_OBJECT( src, "Opened camera with serial '%s' and port '%s'.", src->serial, src->port );
  } catch ( const std::exception &e ) {
    GST_ERROR_OBJECT( src, "Failed to open camera: %s", e.what() );
    return FALSE;
  }
  return TRUE;
}

static gboolean gst_openseekthermalsrc_close( GstOpenSeekThermalSrc *src )
{
  try {
    src->camera->close();
    GST_INFO_OBJECT( src, "Camera closed." );
  } catch ( const std::exception &e ) {
    GST_ERROR_OBJECT( src, "Failed to close camera: %s", e.what() );
    return FALSE;
  }
  return TRUE;
}
