/**
 * SECTION:element-openseekthermalcolorize
 * @title: openseekthermalcolorize
 *
 * openseekthermalcolorize converts a GRAY16_LE thermal stream (centi-Kelvin
 * pixel values) into an 8-bit color image using a selectable colormap.
 * A centered ROI rectangle is drawn, and the mean temperature inside that
 * rectangle is shown as a rounded-rectangle pill in the bottom-right corner.
 *
 * ## Example launch lines
 * |[
 * gst-launch-1.0 openseekthermalsrc ! openseekthermalcolorize ! videoconvert ! autovideosink
 * ]| Live colorized thermal stream with center ROI temperature readout.
 * |[
 * gst-launch-1.0 openseekthermalsrc ! openseekthermalcolorize \
 *   min-centikelvin=29000 max-centikelvin=31000 colormap=inferno \
 *   ! videoconvert ! autovideosink
 * ]| Fixed color range 290..310 K with the inferno colormap.
 */
// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include "openseekthermal_gstreamer/gstopenseekthermalcolorize.h"

#include "openseekthermal_gstreamer/gstthermalscalemeta.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>

GST_DEBUG_CATEGORY_STATIC( openseekthermalcolorize_debug );
#define GST_CAT_DEFAULT openseekthermalcolorize_debug

enum {
  PROP_0,
  PROP_MIN_CENTIKELVIN,
  PROP_MAX_CENTIKELVIN,
  PROP_COLORMAP,
  PROP_ROI_WIDTH,
  PROP_ROI_HEIGHT,
  PROP_AUTO_RANGE_FRAMES,
  PROP_LAST
};

#define DEFAULT_MIN_CENTIKELVIN -1
#define DEFAULT_MAX_CENTIKELVIN -1
#define DEFAULT_COLORMAP GST_OPENSEEKTHERMAL_COLORMAP_IRON
#define DEFAULT_ROI_WIDTH 32u
#define DEFAULT_ROI_HEIGHT 32u
#define DEFAULT_AUTO_RANGE_FRAMES 8u
#define MAX_AUTO_RANGE_FRAMES 256u

#define OPENSEEKTHERMALCOLORIZE_SINK_CAPS                                                          \
  "video/x-raw, format = (string)GRAY16_LE, "                                                      \
  "width = " GST_VIDEO_SIZE_RANGE ", height = " GST_VIDEO_SIZE_RANGE ", "                          \
  "framerate = " GST_VIDEO_FPS_RANGE

#define OPENSEEKTHERMALCOLORIZE_SRC_CAPS                                                           \
  "video/x-raw, format = (string){ RGBA, BGRA, RGB, BGRx }, "                                      \
  "width = " GST_VIDEO_SIZE_RANGE ", height = " GST_VIDEO_SIZE_RANGE ", "                          \
  "framerate = " GST_VIDEO_FPS_RANGE

static GstStaticPadTemplate sink_template = GST_STATIC_PAD_TEMPLATE(
    "sink", GST_PAD_SINK, GST_PAD_ALWAYS, GST_STATIC_CAPS( OPENSEEKTHERMALCOLORIZE_SINK_CAPS ) );

static GstStaticPadTemplate src_template = GST_STATIC_PAD_TEMPLATE(
    "src", GST_PAD_SRC, GST_PAD_ALWAYS, GST_STATIC_CAPS( OPENSEEKTHERMALCOLORIZE_SRC_CAPS ) );

GType gst_openseekthermal_colormap_get_type( void )
{
  static GType type = 0;
  if ( type == 0 ) {
    static const GEnumValue values[] = {
        { GST_OPENSEEKTHERMAL_COLORMAP_IRON, "Iron (black-purple-red-yellow-white)", "iron" },
        { GST_OPENSEEKTHERMAL_COLORMAP_GRAYSCALE, "Grayscale", "grayscale" },
        { GST_OPENSEEKTHERMAL_COLORMAP_JET, "Jet (blue-cyan-green-yellow-red)", "jet" },
        { GST_OPENSEEKTHERMAL_COLORMAP_INFERNO, "Inferno (matplotlib-style)", "inferno" },
        { 0, NULL, NULL } };
    type = g_enum_register_static( "GstOpenSeekThermalColormap", values );
  }
  return type;
}

// cppcheck-suppress unknownMacro
G_DEFINE_TYPE_WITH_CODE(
    GstOpenSeekThermalColorize, gst_openseekthermalcolorize, GST_TYPE_VIDEO_FILTER,
    GST_DEBUG_CATEGORY_INIT( openseekthermalcolorize_debug, "openseekthermalcolorize", 0,
                             "Debug category for openseekthermalcolorize element" ) )

/* forward decls */
static void gst_openseekthermalcolorize_finalize( GObject *object );
static void gst_openseekthermalcolorize_set_property( GObject *object, guint prop_id,
                                                      const GValue *value, GParamSpec *pspec );
static void gst_openseekthermalcolorize_get_property( GObject *object, guint prop_id, GValue *value,
                                                      GParamSpec *pspec );
static gboolean gst_openseekthermalcolorize_set_info( GstVideoFilter *filter, GstCaps *incaps,
                                                      GstVideoInfo *in_info, GstCaps *outcaps,
                                                      GstVideoInfo *out_info );
static GstCaps *gst_openseekthermalcolorize_transform_caps( GstBaseTransform *trans,
                                                            GstPadDirection direction,
                                                            GstCaps *caps, GstCaps *filter );
static GstFlowReturn gst_openseekthermalcolorize_transform_frame( GstVideoFilter *filter,
                                                                  GstVideoFrame *in_frame,
                                                                  GstVideoFrame *out_frame );

/* ----- colormap LUT construction --------------------------------------- */

struct ColorStop {
  float pos;
  guint8 r, g, b;
};

static const ColorStop kIron[] = { { 0.00f, 0, 0, 0 },      { 0.15f, 30, 0, 90 },
                                   { 0.35f, 130, 0, 150 },  { 0.55f, 220, 30, 60 },
                                   { 0.75f, 255, 130, 0 },  { 0.90f, 255, 220, 50 },
                                   { 1.00f, 255, 255, 255 } };

static const ColorStop kJet[] = { { 0.000f, 0, 0, 143 },   { 0.125f, 0, 0, 255 },
                                  { 0.375f, 0, 255, 255 }, { 0.625f, 255, 255, 0 },
                                  { 0.875f, 255, 0, 0 },   { 1.000f, 128, 0, 0 } };

static const ColorStop kInferno[] = { { 0.00f, 0, 0, 4 },      { 0.14f, 40, 11, 84 },
                                      { 0.29f, 101, 21, 110 }, { 0.43f, 159, 42, 99 },
                                      { 0.57f, 212, 72, 66 },  { 0.71f, 245, 125, 21 },
                                      { 0.86f, 250, 193, 39 }, { 1.00f, 252, 255, 164 } };

static void build_lut_from_stops( const ColorStop *stops, gsize n, guint8 *out_rgb )
{
  for ( int i = 0; i < 256; ++i ) {
    float t = static_cast<float>( i ) / 255.0f;
    gsize k = 0;
    while ( k + 1 < n - 1 && stops[k + 1].pos < t ) ++k;
    const ColorStop &a = stops[k];
    const ColorStop &b = stops[k + 1];
    float span = std::max( 1e-6f, b.pos - a.pos );
    float alpha = std::clamp( ( t - a.pos ) / span, 0.0f, 1.0f );
    out_rgb[i * 3 + 0] = static_cast<guint8>( std::lround( a.r + ( b.r - a.r ) * alpha ) );
    out_rgb[i * 3 + 1] = static_cast<guint8>( std::lround( a.g + ( b.g - a.g ) * alpha ) );
    out_rgb[i * 3 + 2] = static_cast<guint8>( std::lround( a.b + ( b.b - a.b ) * alpha ) );
  }
}

static void build_lut( GstOpenSeekThermalColormap cmap, guint8 *out_rgb )
{
  switch ( cmap ) {
  case GST_OPENSEEKTHERMAL_COLORMAP_GRAYSCALE:
    for ( int i = 0; i < 256; ++i ) {
      out_rgb[i * 3 + 0] = out_rgb[i * 3 + 1] = out_rgb[i * 3 + 2] = static_cast<guint8>( i );
    }
    break;
  case GST_OPENSEEKTHERMAL_COLORMAP_JET:
    build_lut_from_stops( kJet, G_N_ELEMENTS( kJet ), out_rgb );
    break;
  case GST_OPENSEEKTHERMAL_COLORMAP_INFERNO:
    build_lut_from_stops( kInferno, G_N_ELEMENTS( kInferno ), out_rgb );
    break;
  case GST_OPENSEEKTHERMAL_COLORMAP_IRON:
  default:
    build_lut_from_stops( kIron, G_N_ELEMENTS( kIron ), out_rgb );
    break;
  }
}

static void ensure_lut( GstOpenSeekThermalColorize *self )
{
  if ( self->lut_for == self->colormap )
    return;
  build_lut( self->colormap, self->lut );
  self->lut_for = self->colormap;
}

/* ----- output format layout -------------------------------------------- */

static gboolean get_format_layout( GstVideoFormat fmt, gint *bpp, gint *r_off, gint *g_off,
                                   gint *b_off, gint *a_off )
{
  switch ( fmt ) {
  case GST_VIDEO_FORMAT_RGBA:
    *bpp = 4;
    *r_off = 0;
    *g_off = 1;
    *b_off = 2;
    *a_off = 3;
    return TRUE;
  case GST_VIDEO_FORMAT_BGRA:
    *bpp = 4;
    *r_off = 2;
    *g_off = 1;
    *b_off = 0;
    *a_off = 3;
    return TRUE;
  case GST_VIDEO_FORMAT_RGB:
    *bpp = 3;
    *r_off = 0;
    *g_off = 1;
    *b_off = 2;
    *a_off = -1;
    return TRUE;
  case GST_VIDEO_FORMAT_BGRx:
    *bpp = 4;
    *r_off = 2;
    *g_off = 1;
    *b_off = 0;
    *a_off = -1;
    return TRUE;
  default:
    return FALSE;
  }
}

/* ----- rolling min/max (median over recent frames) --------------------- */

static void resize_value_buffers( GstOpenSeekThermalColorize *self, guint new_size )
{
  if ( self->count_values == new_size )
    return;
  self->min_values = (guint *)g_realloc( self->min_values, new_size * sizeof( guint ) );
  self->max_values = (guint *)g_realloc( self->max_values, new_size * sizeof( guint ) );
  self->sort_value_buffer = (guint *)g_realloc( self->sort_value_buffer, new_size * sizeof( guint ) );
  for ( guint i = self->count_values; i < new_size; ++i ) {
    self->min_values[i] = self->count_values > 0 ? self->min_values[0] : 0;
    self->max_values[i] = self->count_values > 0 ? self->max_values[0] : 65535;
  }
  self->count_values = new_size;
}

static guint median_of( guint *values, guint count, guint *scratch )
{
  std::partial_sort_copy( values, values + count, scratch, scratch + count );
  return scratch[count / 2];
}

static void push_and_get_stable_range( GstOpenSeekThermalColorize *self, guint frame_min,
                                       guint frame_max, guint *out_min, guint *out_max )
{
  if ( self->count_values == 0 ) {
    *out_min = frame_min;
    *out_max = frame_max;
    return;
  }
  if ( self->first_frame ) {
    std::fill( self->min_values, self->min_values + self->count_values, frame_min );
    std::fill( self->max_values, self->max_values + self->count_values, frame_max );
    self->first_frame = FALSE;
  }
  if ( ++self->index >= self->count_values )
    self->index = 0;
  self->min_values[self->index] = frame_min;
  self->max_values[self->index] = frame_max;
  *out_min = median_of( self->min_values, self->count_values, self->sort_value_buffer );
  *out_max = median_of( self->max_values, self->count_values, self->sort_value_buffer );
}

/* ----- scratch surface management -------------------------------------- */

static void destroy_scratch( GstOpenSeekThermalColorize *self )
{
  if ( self->scratch_surface ) {
    cairo_surface_destroy( self->scratch_surface );
    self->scratch_surface = nullptr;
  }
  g_free( self->scratch );
  self->scratch = nullptr;
  self->scratch_w = 0;
  self->scratch_h = 0;
  self->scratch_stride = 0;
}

static gboolean ensure_scratch( GstOpenSeekThermalColorize *self, gint w, gint h )
{
  if ( self->scratch && self->scratch_w == w && self->scratch_h == h )
    return TRUE;
  destroy_scratch( self );
  gint stride = cairo_format_stride_for_width( CAIRO_FORMAT_ARGB32, w );
  if ( stride <= 0 )
    return FALSE;
  self->scratch = (guint8 *)g_malloc0( static_cast<gsize>( stride ) * static_cast<gsize>( h ) );
  self->scratch_surface =
      cairo_image_surface_create_for_data( self->scratch, CAIRO_FORMAT_ARGB32, w, h, stride );
  if ( cairo_surface_status( self->scratch_surface ) != CAIRO_STATUS_SUCCESS ) {
    destroy_scratch( self );
    return FALSE;
  }
  self->scratch_w = w;
  self->scratch_h = h;
  self->scratch_stride = stride;
  return TRUE;
}

/* ----- cairo overlay drawing ------------------------------------------- */

static void rounded_rect_path( cairo_t *cr, double x, double y, double w, double h, double r )
{
  if ( r > w / 2.0 )
    r = w / 2.0;
  if ( r > h / 2.0 )
    r = h / 2.0;
  cairo_new_sub_path( cr );
  cairo_arc( cr, x + w - r, y + r, r, -M_PI / 2.0, 0.0 );
  cairo_arc( cr, x + w - r, y + h - r, r, 0.0, M_PI / 2.0 );
  cairo_arc( cr, x + r, y + h - r, r, M_PI / 2.0, M_PI );
  cairo_arc( cr, x + r, y + r, r, M_PI, 3.0 * M_PI / 2.0 );
  cairo_close_path( cr );
}

static void draw_overlay( GstOpenSeekThermalColorize *self, gint w, gint h, gint roi_x, gint roi_y,
                          gint roi_w, gint roi_h, double mean_celsius )
{
  cairo_surface_mark_dirty( self->scratch_surface );
  cairo_t *cr = cairo_create( self->scratch_surface );

  /* ROI rectangle */
  cairo_set_line_width( cr, 2.0 );
  cairo_set_source_rgba( cr, 1.0, 1.0, 1.0, 1.0 );
  cairo_rectangle( cr, roi_x + 0.5, roi_y + 0.5, roi_w - 1, roi_h - 1 );
  cairo_stroke( cr );

  /* Pill: bottom-right with temperature in Celsius */
  gchar label[32];
  g_snprintf( label, sizeof( label ), "%.1f °C", mean_celsius );

  double font_size = std::max( 12.0, h / 22.0 );
  cairo_select_font_face( cr, "sans", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_BOLD );
  cairo_set_font_size( cr, font_size );

  cairo_text_extents_t te;
  cairo_text_extents( cr, label, &te );

  double pad_x = font_size * 0.55;
  double pad_y = font_size * 0.35;
  double pill_w = te.width + 2.0 * pad_x;
  double pill_h = te.height + 2.0 * pad_y;
  double margin = std::max( 6.0, font_size * 0.4 );
  double pill_x = w - margin - pill_w;
  double pill_y = h - margin - pill_h;
  if ( pill_x < 0 )
    pill_x = 0;
  if ( pill_y < 0 )
    pill_y = 0;

  rounded_rect_path( cr, pill_x, pill_y, pill_w, pill_h, pill_h / 2.0 );
  cairo_set_source_rgba( cr, 0.0, 0.0, 0.0, 0.65 );
  cairo_fill( cr );

  double text_x = pill_x + pad_x - te.x_bearing;
  double text_y = pill_y + pad_y - te.y_bearing;
  cairo_set_source_rgba( cr, 1.0, 1.0, 1.0, 1.0 );
  cairo_move_to( cr, text_x, text_y );
  cairo_show_text( cr, label );

  cairo_destroy( cr );
  cairo_surface_flush( self->scratch_surface );
}

/* ----- core transform -------------------------------------------------- */

static inline guint cK_to_raw( double cK, double scale, double offset )
{
  double v = cK * scale + offset;
  return static_cast<guint>( std::clamp( v, 0.0, 65535.0 ) );
}

static void colorize_into_scratch( GstOpenSeekThermalColorize *self, const guint16 *src, gint w,
                                   gint h, gint src_stride_bytes, guint raw_lo, guint raw_hi )
{
  ensure_lut( self );
  const guint8 *lut = self->lut;
  guint span = ( raw_hi > raw_lo ) ? ( raw_hi - raw_lo ) : 1;
  for ( gint y = 0; y < h; ++y ) {
    const guint16 *row_in = reinterpret_cast<const guint16 *>(
        reinterpret_cast<const guint8 *>( src ) + y * src_stride_bytes );
    guint8 *row_out = self->scratch + y * self->scratch_stride;
    for ( gint x = 0; x < w; ++x ) {
      guint v = row_in[x];
      gint64 norm = ( static_cast<gint64>( v ) - static_cast<gint64>( raw_lo ) ) * 255 / span;
      guint8 idx = static_cast<guint8>( std::clamp<gint64>( norm, 0, 255 ) );
      const guint8 *c = lut + idx * 3;
      /* ARGB32 little-endian: bytes B,G,R,A */
      row_out[x * 4 + 0] = c[2];
      row_out[x * 4 + 1] = c[1];
      row_out[x * 4 + 2] = c[0];
      row_out[x * 4 + 3] = 255;
    }
  }
}

static void convert_scratch_to_output( GstOpenSeekThermalColorize *self, GstVideoFrame *out_frame )
{
  guint8 *out_data = reinterpret_cast<guint8 *>( GST_VIDEO_FRAME_PLANE_DATA( out_frame, 0 ) );
  gint out_stride = GST_VIDEO_FRAME_PLANE_STRIDE( out_frame, 0 );
  gint w = GST_VIDEO_FRAME_WIDTH( out_frame );
  gint h = GST_VIDEO_FRAME_HEIGHT( out_frame );
  gint bpp = self->bytes_per_pixel;
  gint r_off = self->r_offset;
  gint g_off = self->g_offset;
  gint b_off = self->b_offset;
  gint a_off = self->a_offset;
  for ( gint y = 0; y < h; ++y ) {
    const guint8 *row_in = self->scratch + y * self->scratch_stride;
    guint8 *row_out = out_data + y * out_stride;
    for ( gint x = 0; x < w; ++x ) {
      guint8 b = row_in[x * 4 + 0];
      guint8 g = row_in[x * 4 + 1];
      guint8 r = row_in[x * 4 + 2];
      guint8 *p = row_out + x * bpp;
      p[r_off] = r;
      p[g_off] = g;
      p[b_off] = b;
      if ( a_off >= 0 )
        p[a_off] = 255;
    }
  }
}

static GstFlowReturn gst_openseekthermalcolorize_transform_frame( GstVideoFilter *filter,
                                                                  GstVideoFrame *in_frame,
                                                                  GstVideoFrame *out_frame )
{
  GstOpenSeekThermalColorize *self = GST_OPENSEEKTHERMALCOLORIZE( filter );
  gint w = GST_VIDEO_FRAME_WIDTH( in_frame );
  gint h = GST_VIDEO_FRAME_HEIGHT( in_frame );
  const guint16 *src = reinterpret_cast<const guint16 *>( GST_VIDEO_FRAME_PLANE_DATA( in_frame, 0 ) );
  gint src_stride = GST_VIDEO_FRAME_PLANE_STRIDE( in_frame, 0 );

  if ( !ensure_scratch( self, w, h ) ) {
    GST_ERROR_OBJECT( self, "Failed to allocate %dx%d scratch surface", w, h );
    return GST_FLOW_ERROR;
  }

  /* The colormap is invariant under any linear transform of the pixel values,
   * so frame min/max and the auto-range median can stay in raw pixel space.
   * The thermal-scale meta only matters for two things: translating the
   * user-facing min/max-centikelvin bounds into raw units, and translating
   * the ROI mean back into Celsius. */
  double scale = 1.0;
  double offset = 0.0;
  GstThermalScaleMeta *scale_meta = gst_buffer_get_thermal_scale_meta( in_frame->buffer );
  if ( scale_meta != nullptr && scale_meta->scale != 0.0f ) {
    scale = scale_meta->scale;
    offset = scale_meta->offset;
  }

  guint roi_w = std::min<guint>( self->roi_width, static_cast<guint>( w ) );
  guint roi_h = std::min<guint>( self->roi_height, static_cast<guint>( h ) );
  if ( roi_w == 0 )
    roi_w = 1;
  if ( roi_h == 0 )
    roi_h = 1;
  gint roi_x = ( w - static_cast<gint>( roi_w ) ) / 2;
  gint roi_y = ( h - static_cast<gint>( roi_h ) ) / 2;

  guint raw_min = 65535;
  guint raw_max = 0;
  guint64 roi_sum_raw = 0;
  guint roi_count = 0;
  for ( gint y = 0; y < h; ++y ) {
    const guint16 *row =
        reinterpret_cast<const guint16 *>( reinterpret_cast<const guint8 *>( src ) + y * src_stride );
    for ( gint x = 0; x < w; ++x ) {
      guint v = row[x];
      if ( v < raw_min )
        raw_min = v;
      if ( v > raw_max )
        raw_max = v;
      if ( x >= roi_x && x < roi_x + static_cast<gint>( roi_w ) && y >= roi_y &&
           y < roi_y + static_cast<gint>( roi_h ) ) {
        roi_sum_raw += v;
        ++roi_count;
      }
    }
  }

  /* Auto range stays in raw units. */
  guint auto_min_raw = raw_min;
  guint auto_max_raw = raw_max;
  if ( self->min_centikelvin < 0 || self->max_centikelvin < 0 ) {
    // Lock against a concurrent set_property(auto-range-frames) realloc of the
    // rolling-window buffers.
    GST_OBJECT_LOCK( self );
    push_and_get_stable_range( self, raw_min, raw_max, &auto_min_raw, &auto_max_raw );
    GST_OBJECT_UNLOCK( self );
  }
  /* User-set bounds are in cK; map them into raw units once. */
  guint raw_lo = ( self->min_centikelvin >= 0 ) ? cK_to_raw( self->min_centikelvin, scale, offset )
                                                : auto_min_raw;
  guint raw_hi = ( self->max_centikelvin >= 0 ) ? cK_to_raw( self->max_centikelvin, scale, offset )
                                                : auto_max_raw;
  if ( raw_lo > raw_hi )
    std::swap( raw_lo, raw_hi );
  if ( raw_hi == raw_lo )
    raw_hi = raw_lo + 1;

  /* ROI mean: invert the linear transform on the mean only. */
  double mean_c = 0.0;
  if ( roi_count > 0 ) {
    double mean_raw = static_cast<double>( roi_sum_raw ) / static_cast<double>( roi_count );
    double mean_ck = ( mean_raw - offset ) / scale;
    mean_c = ( mean_ck - 27315.0 ) / 100.0;
  }

  colorize_into_scratch( self, src, w, h, src_stride, raw_lo, raw_hi );
  draw_overlay( self, w, h, roi_x, roi_y, static_cast<gint>( roi_w ), static_cast<gint>( roi_h ),
                mean_c );
  convert_scratch_to_output( self, out_frame );

  return GST_FLOW_OK;
}

static GstCaps *gst_openseekthermalcolorize_transform_caps( GstBaseTransform *trans,
                                                            GstPadDirection direction,
                                                            GstCaps *caps, GstCaps *filter )
{
  /* Strip format from the incoming caps, then intersect with the template of
   * the opposite pad so we always offer the full set of supported formats on
   * that side while preserving width/height/framerate. */
  GstCaps *other_template;
  if ( direction == GST_PAD_SINK ) {
    other_template = gst_pad_get_pad_template_caps( GST_BASE_TRANSFORM_SRC_PAD( trans ) );
  } else {
    other_template = gst_pad_get_pad_template_caps( GST_BASE_TRANSFORM_SINK_PAD( trans ) );
  }

  GstCaps *result = gst_caps_new_empty();
  for ( guint i = 0; i < gst_caps_get_size( caps ); ++i ) {
    GstStructure *s = gst_structure_copy( gst_caps_get_structure( caps, i ) );
    gst_structure_remove_field( s, "format" );
    gst_structure_set_name( s, "video/x-raw" );
    gst_caps_append_structure( result, s );
  }

  GstCaps *intersect = gst_caps_intersect_full( result, other_template, GST_CAPS_INTERSECT_FIRST );
  gst_caps_unref( result );
  gst_caps_unref( other_template );

  if ( filter ) {
    GstCaps *tmp = gst_caps_intersect_full( filter, intersect, GST_CAPS_INTERSECT_FIRST );
    gst_caps_unref( intersect );
    intersect = tmp;
  }
  return intersect;
}

static gboolean gst_openseekthermalcolorize_set_info( GstVideoFilter *filter, GstCaps *,
                                                      GstVideoInfo *in_info, GstCaps *,
                                                      GstVideoInfo *out_info )
{
  GstOpenSeekThermalColorize *self = GST_OPENSEEKTHERMALCOLORIZE( filter );
  if ( GST_VIDEO_INFO_FORMAT( in_info ) != GST_VIDEO_FORMAT_GRAY16_LE ) {
    GST_ERROR_OBJECT( self, "Unsupported input format" );
    return FALSE;
  }
  GstVideoFormat fmt = GST_VIDEO_INFO_FORMAT( out_info );
  gint bpp, r_off, g_off, b_off, a_off;
  if ( !get_format_layout( fmt, &bpp, &r_off, &g_off, &b_off, &a_off ) ) {
    GST_ERROR_OBJECT( self, "Unsupported output format: %s", gst_video_format_to_string( fmt ) );
    return FALSE;
  }
  self->out_format = fmt;
  self->bytes_per_pixel = bpp;
  self->r_offset = r_off;
  self->g_offset = g_off;
  self->b_offset = b_off;
  self->a_offset = a_off;
  ensure_lut( self );
  GST_INFO_OBJECT( self, "Negotiated output %s (%dx%d)", gst_video_format_to_string( fmt ),
                   GST_VIDEO_INFO_WIDTH( out_info ), GST_VIDEO_INFO_HEIGHT( out_info ) );
  return TRUE;
}

/* ----- gobject boilerplate --------------------------------------------- */

static void gst_openseekthermalcolorize_init( GstOpenSeekThermalColorize *self )
{
  self->min_centikelvin = DEFAULT_MIN_CENTIKELVIN;
  self->max_centikelvin = DEFAULT_MAX_CENTIKELVIN;
  self->colormap = DEFAULT_COLORMAP;
  self->lut_for = static_cast<GstOpenSeekThermalColormap>( -1 );
  self->roi_width = DEFAULT_ROI_WIDTH;
  self->roi_height = DEFAULT_ROI_HEIGHT;
  self->auto_range_frames = DEFAULT_AUTO_RANGE_FRAMES;

  self->out_format = GST_VIDEO_FORMAT_UNKNOWN;
  self->bytes_per_pixel = 0;
  self->r_offset = self->g_offset = self->b_offset = 0;
  self->a_offset = -1;

  self->min_values = nullptr;
  self->max_values = nullptr;
  self->sort_value_buffer = nullptr;
  self->count_values = 0;
  self->index = 0;
  self->first_frame = TRUE;
  resize_value_buffers( self, self->auto_range_frames );

  self->scratch = nullptr;
  self->scratch_w = self->scratch_h = self->scratch_stride = 0;
  self->scratch_surface = nullptr;
}

static void gst_openseekthermalcolorize_finalize( GObject *object )
{
  GstOpenSeekThermalColorize *self = GST_OPENSEEKTHERMALCOLORIZE( object );
  g_free( self->min_values );
  g_free( self->max_values );
  g_free( self->sort_value_buffer );
  destroy_scratch( self );
  G_OBJECT_CLASS( gst_openseekthermalcolorize_parent_class )->finalize( object );
}

static void gst_openseekthermalcolorize_set_property( GObject *object, guint prop_id,
                                                      const GValue *value, GParamSpec *pspec )
{
  GstOpenSeekThermalColorize *self = GST_OPENSEEKTHERMALCOLORIZE( object );
  switch ( prop_id ) {
  case PROP_MIN_CENTIKELVIN:
    self->min_centikelvin = g_value_get_int( value );
    break;
  case PROP_MAX_CENTIKELVIN:
    self->max_centikelvin = g_value_get_int( value );
    break;
  case PROP_COLORMAP:
    self->colormap = static_cast<GstOpenSeekThermalColormap>( g_value_get_enum( value ) );
    break;
  case PROP_ROI_WIDTH:
    self->roi_width = g_value_get_uint( value );
    break;
  case PROP_ROI_HEIGHT:
    self->roi_height = g_value_get_uint( value );
    break;
  case PROP_AUTO_RANGE_FRAMES:
    // Reallocs the rolling-window buffers that transform_frame reads on the
    // streaming thread; hold the object lock so a live property change can't
    // free them mid-read.
    GST_OBJECT_LOCK( self );
    self->auto_range_frames =
        std::clamp<guint>( g_value_get_uint( value ), 1u, MAX_AUTO_RANGE_FRAMES );
    resize_value_buffers( self, self->auto_range_frames );
    self->index = 0;
    self->first_frame = TRUE;
    GST_OBJECT_UNLOCK( self );
    break;
  default:
    G_OBJECT_WARN_INVALID_PROPERTY_ID( object, prop_id, pspec );
    break;
  }
}

static void gst_openseekthermalcolorize_get_property( GObject *object, guint prop_id, GValue *value,
                                                      GParamSpec *pspec )
{
  GstOpenSeekThermalColorize *self = GST_OPENSEEKTHERMALCOLORIZE( object );
  switch ( prop_id ) {
  case PROP_MIN_CENTIKELVIN:
    g_value_set_int( value, self->min_centikelvin );
    break;
  case PROP_MAX_CENTIKELVIN:
    g_value_set_int( value, self->max_centikelvin );
    break;
  case PROP_COLORMAP:
    g_value_set_enum( value, self->colormap );
    break;
  case PROP_ROI_WIDTH:
    g_value_set_uint( value, self->roi_width );
    break;
  case PROP_ROI_HEIGHT:
    g_value_set_uint( value, self->roi_height );
    break;
  case PROP_AUTO_RANGE_FRAMES:
    g_value_set_uint( value, self->auto_range_frames );
    break;
  default:
    G_OBJECT_WARN_INVALID_PROPERTY_ID( object, prop_id, pspec );
    break;
  }
}

static void gst_openseekthermalcolorize_class_init( GstOpenSeekThermalColorizeClass *klass )
{
  GObjectClass *gobject_class = G_OBJECT_CLASS( klass );
  GstElementClass *element_class = GST_ELEMENT_CLASS( klass );
  GstBaseTransformClass *trans_class = GST_BASE_TRANSFORM_CLASS( klass );
  GstVideoFilterClass *vfilter_class = GST_VIDEO_FILTER_CLASS( klass );

  gobject_class->finalize = gst_openseekthermalcolorize_finalize;
  gobject_class->set_property = gst_openseekthermalcolorize_set_property;
  gobject_class->get_property = gst_openseekthermalcolorize_get_property;

  g_object_class_install_property(
      gobject_class, PROP_MIN_CENTIKELVIN,
      g_param_spec_int( "min-centikelvin", "Min centi-Kelvin",
                        "Lower bound of the colormap range in centi-Kelvin. -1 = auto.", -1,
                        G_MAXINT, DEFAULT_MIN_CENTIKELVIN,
                        (GParamFlags)( G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS ) ) );
  g_object_class_install_property(
      gobject_class, PROP_MAX_CENTIKELVIN,
      g_param_spec_int( "max-centikelvin", "Max centi-Kelvin",
                        "Upper bound of the colormap range in centi-Kelvin. -1 = auto.", -1,
                        G_MAXINT, DEFAULT_MAX_CENTIKELVIN,
                        (GParamFlags)( G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS ) ) );
  g_object_class_install_property(
      gobject_class, PROP_COLORMAP,
      g_param_spec_enum( "colormap", "Colormap", "False-color palette applied to the thermal data.",
                         GST_TYPE_OPENSEEKTHERMAL_COLORMAP, DEFAULT_COLORMAP,
                         (GParamFlags)( G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS ) ) );
  g_object_class_install_property(
      gobject_class, PROP_ROI_WIDTH,
      g_param_spec_uint(
          "roi-width", "ROI width", "Width in pixels of the centered measurement rectangle.", 1,
          65535, DEFAULT_ROI_WIDTH, (GParamFlags)( G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS ) ) );
  g_object_class_install_property(
      gobject_class, PROP_ROI_HEIGHT,
      g_param_spec_uint(
          "roi-height", "ROI height", "Height in pixels of the centered measurement rectangle.", 1,
          65535, DEFAULT_ROI_HEIGHT, (GParamFlags)( G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS ) ) );
  g_object_class_install_property(
      gobject_class, PROP_AUTO_RANGE_FRAMES,
      g_param_spec_uint( "auto-range-frames", "Auto-range frames",
                         "Rolling-median window length used to stabilize auto min/max.", 1,
                         MAX_AUTO_RANGE_FRAMES, DEFAULT_AUTO_RANGE_FRAMES,
                         (GParamFlags)( G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS ) ) );

  gst_element_class_set_static_metadata(
      element_class, "OpenSeekThermal Colorize", "Filter/Effect/Video",
      "Maps centi-Kelvin thermal frames to a color image and overlays a ROI temperature readout",
      "Stefan Fabian <gstreamer@stefanfabian.com>" );

  gst_element_class_add_static_pad_template( element_class, &sink_template );
  gst_element_class_add_static_pad_template( element_class, &src_template );

  trans_class->transform_caps = GST_DEBUG_FUNCPTR( gst_openseekthermalcolorize_transform_caps );

  vfilter_class->set_info = GST_DEBUG_FUNCPTR( gst_openseekthermalcolorize_set_info );
  vfilter_class->transform_frame = GST_DEBUG_FUNCPTR( gst_openseekthermalcolorize_transform_frame );
}
