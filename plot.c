#include <math.h>
#include <string.h>
#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "nanovna.h"

static void cell_draw_marker_info(int m, int n, int w, int h);

int16_t grid_offset;
int16_t grid_width;

int16_t area_width  = AREA_WIDTH_NORMAL;
int16_t area_height = AREA_HEIGHT_NORMAL;

// Depends from spi_buffer size, CELLWIDTH*CELLHEIGHT <= sizeof(spi_buffer)
#define CELLWIDTH  (64)
#define CELLHEIGHT (32)

// indicate dirty cells (not redraw if cell data not changed)
#define MAX_MARKMAP_X    ((320+CELLWIDTH-1)/CELLWIDTH)
#define MAX_MARKMAP_Y    ((240+CELLHEIGHT-1)/CELLHEIGHT)
uint16_t markmap[2][MAX_MARKMAP_Y];
uint16_t current_mappage = 0;

// Trace data cache, for faster redraw cells
//   CELL_X[16:31] x position
//   CELL_Y[ 0:15] y position
static uint32_t trace_index[TRACES_MAX][POINTS_COUNT];

#define INDEX(x, y) ((((uint32_t)x)<<16)|(((uint32_t)y)))
#define CELL_X(i)  (int)(((i)>>16))
#define CELL_Y(i)  (int)(((i)&0xFFFF))
//#define CELL_P(i, x, y) (((((x)&0x03e0UL)<<22) | (((y)&0x03e0UL)<<17)) == ((i)&0xffc00000UL))

//#define floatToInt(v) ((int)(v))
static int
floatToInt(float v){
  if (v < 0) return v-0.5;
  if (v > 0) return v+0.5;
    return 0;
}

void update_grid(void)
{
  uint32_t gdigit = 100000000;
  uint32_t fstart = get_sweep_frequency(ST_START);
  uint32_t fspan  = get_sweep_frequency(ST_SPAN);
  uint32_t grid;
  
  while (gdigit > 100) {
    grid = 5 * gdigit;
    if (fspan / grid >= 4)
      break;
    grid = 2 * gdigit;
    if (fspan / grid >= 4)
      break;
    grid = gdigit;
    if (fspan / grid >= 4)
      break;
    gdigit /= 10;
  }

  grid_offset = (WIDTH) * ((fstart % grid) / 100) / (fspan / 100);
  grid_width = (WIDTH) * (grid / 100) / (fspan / 1000);

  force_set_markmap();
  redraw_request |= REDRAW_FREQUENCY;
}

static inline int
circle_inout(int x, int y, int r)
{
  int d = x*x + y*y - r*r;
  if (d < -r)
    return 1;
  if (d >  r)
    return -1;
  return 0;
}

static int
polar_grid(int x, int y)
{
  int d;

  // offset to center
  x -= P_CENTER_X;
  y -= P_CENTER_Y;

  // outer circle
  d = circle_inout(x, y, P_RADIUS);
  if (d < 0) return 0;
  if (d == 0) return 1;

  // vertical and horizontal axis
  if (x == 0 || y == 0)
    return 1;

  d = circle_inout(x, y, P_RADIUS / 5);
  if (d == 0) return 1;
  if (d > 0) return 0;

  d = circle_inout(x, y, P_RADIUS * 2 / 5);
  if (d == 0) return 1;
  if (d > 0) return 0;

  // cross sloping lines
  if (x == y || x == -y)
    return 1;

  d = circle_inout(x, y, P_RADIUS * 3 / 5);
  if (d == 0) return 1;
  if (d > 0) return 0;

  d = circle_inout(x, y, P_RADIUS * 4 / 5);
  if (d == 0) return 1;
  return 0;
}

/*
 * Constant Resistance circle: (u - r/(r+1))^2 + v^2 = 1/(r+1)^2
 * Constant Reactance circle:  (u - 1)^2 + (v-1/x)^2 = 1/x^2
 */
static int
smith_grid(int x, int y)
{
  int d;

  // offset to center
  x -= P_CENTER_X;
  y -= P_CENTER_Y;
  
  // outer circle
  d = circle_inout(x, y, P_RADIUS);
  if (d < 0)
    return 0;
  if (d == 0)
    return 1;
  
  // horizontal axis
  if (y == 0)
    return 1;

  // shift circle center to right origin
  x -= P_RADIUS;

  // Constant Reactance Circle: 2j : R/2 = P_RADIUS/2
  if (circle_inout(x, y+P_RADIUS/2, P_RADIUS/2) == 0)
    return 1;
  if (circle_inout(x, y-P_RADIUS/2, P_RADIUS/2) == 0)
    return 1;

  // Constant Resistance Circle: 3 : R/4 = P_RADIUS/4
  d = circle_inout(x+P_RADIUS/4, y, P_RADIUS/4);
  if (d > 0) return 0;
  if (d == 0) return 1;

  // Constant Reactance Circle: 1j : R = P_RADIUS
  if (circle_inout(x, y+P_RADIUS, P_RADIUS) == 0)
    return 1;
  if (circle_inout(x, y-P_RADIUS, P_RADIUS) == 0)
    return 1;

  // Constant Resistance Circle: 1 : R/2
  d = circle_inout(x+P_RADIUS/2, y, P_RADIUS/2);
  if (d > 0) return 0;
  if (d == 0) return 1;

  // Constant Reactance Circle: 1/2j : R*2
  if (circle_inout(x, y+P_RADIUS*2, P_RADIUS*2) == 0)
    return 1;
  if (circle_inout(x, y-P_RADIUS*2, P_RADIUS*2) == 0)
    return 1;

  // Constant Resistance Circle: 1/3 : R*3/4
  if (circle_inout(x+P_RADIUS*3/4, y, P_RADIUS*3/4) == 0)
    return 1;
  return 0;
}

#if 0
static int
smith_grid2(int x, int y, float scale)
{
  int d;

  // offset to center
  x -= P_CENTER_X;
  y -= P_CENTER_Y;
  
  // outer circle
  d = circle_inout(x, y, P_RADIUS);
  if (d < 0)
    return 0;
  if (d == 0)
    return 1;

  // shift circle center to right origin
  x -= P_RADIUS * scale;

  // Constant Reactance Circle: 2j : R/2 = 58
  if (circle_inout(x, y+58*scale, 58*scale) == 0)
    return 1;
  if (circle_inout(x, y-58*scale, 58*scale) == 0)
    return 1;
#if 0
  // Constant Resistance Circle: 3 : R/4 = 29
  d = circle_inout(x+29*scale, y, 29*scale);
  if (d > 0) return 0;
  if (d == 0) return 1;
  d = circle_inout(x-29*scale, y, 29*scale);
  if (d > 0) return 0;
  if (d == 0) return 1;
#endif

  // Constant Reactance Circle: 1j : R = 116
  if (circle_inout(x, y+116*scale, 116*scale) == 0)
    return 1;
  if (circle_inout(x, y-116*scale, 116*scale) == 0)
    return 1;

  // Constant Resistance Circle: 1 : R/2 = 58
  d = circle_inout(x+58*scale, y, 58*scale);
  if (d > 0) return 0;
  if (d == 0) return 1;
  d = circle_inout(x-58*scale, y, 58*scale);
  if (d > 0) return 0;
  if (d == 0) return 1;

  // Constant Reactance Circle: 1/2j : R*2 = 232
  if (circle_inout(x, y+232*scale, 232*scale) == 0)
    return 1;
  if (circle_inout(x, y-232*scale, 232*scale) == 0)
    return 1;

#if 0
  // Constant Resistance Circle: 1/3 : R*3/4 = 87
  d = circle_inout(x+87*scale, y, 87*scale);
  if (d > 0) return 0;
  if (d == 0) return 1;
  d = circle_inout(x+87*scale, y, 87*scale);
  if (d > 0) return 0;
  if (d == 0) return 1;
#endif

  // Constant Resistance Circle: 0 : R
  d = circle_inout(x+P_RADIUS*scale, y, P_RADIUS*scale);
  if (d > 0) return 0;
  if (d == 0) return 1;
  d = circle_inout(x-P_RADIUS*scale, y, P_RADIUS*scale);
  if (d > 0) return 0;
  if (d == 0) return 1;

  // Constant Resistance Circle: -1/3 : R*3/2 = 174
  d = circle_inout(x+174*scale, y, 174*scale);
  if (d > 0) return 0;
  if (d == 0) return 1;
  d = circle_inout(x-174*scale, y, 174*scale);
  //if (d > 0) return 0;
  if (d == 0) return 1;
  return 0;
}
#endif

#if 0
const int cirs[][4] = {
  { 0, 58/2, 58/2, 0 },    // Constant Reactance Circle: 2j : R/2 = 58
  { 29/2, 0, 29/2, 1 },    // Constant Resistance Circle: 3 : R/4 = 29
  { 0, 115/2, 115/2, 0 },  // Constant Reactance Circle: 1j : R = 115
  { 58/2, 0, 58/2, 1 },    // Constant Resistance Circle: 1 : R/2 = 58
  { 0, 230/2, 230/2, 0 },  // Constant Reactance Circle: 1/2j : R*2 = 230
  { 86/2, 0, 86/2, 1 },    // Constant Resistance Circle: 1/3 : R*3/4 = 86
  { 0, 460/2, 460/2, 0 },  // Constant Reactance Circle: 1/4j : R*4 = 460
  { 115/2, 0, 115/2, 1 },  // Constant Resistance Circle: 0 : R
  { 173/2, 0, 173/2, 1 },  // Constant Resistance Circle: -1/3 : R*3/2 = 173
  { 0, 0, 0, 0 } // sentinel
};  

static int
smith_grid3(int x, int y)
{
  int d;

  // offset to center
  x -= P_CENTER_X;
  y -= P_CENTER_Y;
  
  // outer circle
  d = circle_inout(x, y, P_RADIUS);
  if (d < 0)
    return 0;
  if (d == 0)
    return 1;

  // shift circle center to right origin
  x -= P_RADIUS /2;

  int i;
  for (i = 0; cirs[i][2]; i++) {
    d = circle_inout(x+cirs[i][0], y+cirs[i][1], cirs[i][2]);
    if (d == 0)
      return 1;
    if (d > 0 && cirs[i][3])
      return 0;
    d = circle_inout(x-cirs[i][0], y-cirs[i][1], cirs[i][2]);
    if (d == 0)
      return 1;
    if (d > 0 && cirs[i][3])
      return 0;
  }
  return 0;
}
#endif

#if 0
static int
rectangular_grid(int x, int y)
{
  //#define FREQ(x) (((x) * (fspan / 1000) / (WIDTH-1)) * 1000 + fstart)
  //int32_t n = FREQ(x-1) / fgrid;
  //int32_t m = FREQ(x) / fgrid;
  //if ((m - n) > 0)
  //if (((x * 6) % (WIDTH-1)) < 6)
  //if (((x - grid_offset) % grid_width) == 0)
  if (x == 0 || x == WIDTH-1)
    return 1;
  if ((y % GRIDY) == 0)
    return 1;
  if ((((x + grid_offset) * 10) % grid_width) < 10)
    return 1;
  return 0;
}
#endif

static int
rectangular_grid_x(int x)
{
  x-=CELLOFFSETX;
  if (x < 0)
    return 0;
  if (x == 0 || x == WIDTH)
    return 1;
  if ((((x + grid_offset) * 10) % grid_width) < 10)
    return 1;
  return 0;
}

static int
rectangular_grid_y(int y)
{
  if (y < 0)
    return 0;
  if ((y % GRIDY) == 0)
    return 1;
  return 0;
}

#if 0
int
set_strut_grid(int x)
{
  uint16_t *buf = spi_buffer;
  int y;

  for (y = 0; y < HEIGHT; y++) {
    int c = rectangular_grid(x, y);
    c |= smith_grid(x, y);
    *buf++ = c;
  }
  return y;
}

void
draw_on_strut(int v0, int d, int color)
{
  int v;
  int v1 = v0 + d;
  if (v0 < 0) v0 = 0;
  if (v1 < 0) v1 = 0;
  if (v0 >= HEIGHT) v0 = HEIGHT-1;
  if (v1 >= HEIGHT) v1 = HEIGHT-1;
  if (v0 == v1) {
    v = v0; d = 2;
  } else if (v0 < v1) {
    v = v0; d = v1 - v0 + 1;
  } else {
    v = v1; d = v0 - v1 + 1;
  }
  while (d-- > 0)
    spi_buffer[v++] |= color;
}
#endif

/*
 * calculate log10(abs(gamma))
 */ 
static float
logmag(const float *v)
{
  return log10f(v[0]*v[0] + v[1]*v[1]) * 10;
}

/*
 * calculate phase[-2:2] of coefficient
 */ 
static float
phase(const float *v)
{
  return 2 * atan2f(v[1], v[0]) / M_PI * 90;
}

/*
 * calculate groupdelay
 */
static float
groupdelay(const float *v, const float *w, float deltaf)
{
#if 1
  // atan(w)-atan(v) = atan((w-v)/(1+wv))
  float r = w[0]*v[1] - w[1]*v[0];
  float i = w[0]*v[0] + w[1]*v[1];
  return atan2f(r, i) / (2 * M_PI * deltaf);
#else
  return (atan2f(w[0], w[1]) - atan2f(v[0], v[1])) / (2 * M_PI * deltaf);
#endif
}

/*
 * calculate abs(gamma)
 */
static float
linear(const float *v)
{
  return - sqrtf(v[0]*v[0] + v[1]*v[1]);
}

/*
 * calculate vswr; (1+gamma)/(1-gamma)
 */ 
static float
swr(const float *v)
{
  float x = sqrtf(v[0]*v[0] + v[1]*v[1]);
  if (x >= 1)
    return INFINITY;
  return (1 + x)/(1 - x);
}

static float
resitance(const float *v) {
  float z0 = 50;
  float d = z0 / ((1-v[0])*(1-v[0])+v[1]*v[1]);
  float zr = ((1+v[0])*(1-v[0]) - v[1]*v[1]) * d;
  return zr;
}

static float
reactance(const float *v) {
  float z0 = 50;
  float d = z0 / ((1-v[0])*(1-v[0])+v[1]*v[1]);
  float zi = 2*v[1] * d;
  return zi;
}

static void
cartesian_scale(float re, float im, int *xp, int *yp, float scale)
{
  //float scale = 4e-3;
  int x = floatToInt(re * P_RADIUS * scale);
  int y = floatToInt(im * P_RADIUS * scale);
  if      (x < -P_RADIUS) x = -P_RADIUS;
  else if (x >  P_RADIUS) x =  P_RADIUS;
  if      (y < -P_RADIUS) y = -P_RADIUS;
  else if (y >  P_RADIUS) y =  P_RADIUS;
  *xp = P_CENTER_X + x;
  *yp = P_CENTER_Y - y;
}

float
groupdelay_from_array(int i, float array[POINTS_COUNT][2])
{
  int bottom = (i ==   0) ?   0 : i - 1;
  int top    = (i == POINTS_COUNT-1) ? POINTS_COUNT-1 : i + 1;
  float deltaf = frequencies[top] - frequencies[bottom];
  return groupdelay(array[bottom], array[top], deltaf);
}

static float
gamma2resistance(const float v[2])
{
  float z0 = 50;
  float d = z0 / ((1-v[0])*(1-v[0])+v[1]*v[1]);
  return ((1+v[0])*(1-v[0]) - v[1]*v[1]) * d;
}

static float
gamma2reactance(const float v[2])
{
  float z0 = 50;
  float d = z0 / ((1-v[0])*(1-v[0])+v[1]*v[1]);
  return 2*v[1] * d;
}

static uint32_t
trace_into_index(int t, int i, float array[POINTS_COUNT][2])
{
  int y, x;

  float *coeff = array[i];
  float refpos = 10 - get_trace_refpos(t);
  float v = refpos;
  float scale = 1 / get_trace_scale(t);
  switch (trace[t].type) {
  case TRC_LOGMAG:
    v-= logmag(coeff) * scale;
    break;
  case TRC_PHASE:
    v-= phase(coeff) * scale;
    break;
  case TRC_DELAY:
    v-= groupdelay_from_array(i, array) * scale;
    break;
  case TRC_LINEAR:
    v+= linear(coeff) * scale;
    break;
  case TRC_SWR:
    v+= (1 - swr(coeff)) * scale;
    break;
  case TRC_REAL:
    v-= coeff[0] * scale;
    break;
  case TRC_IMAG:
    v-= coeff[1] * scale;
    break;
  case TRC_R:
    v-= resitance(coeff) * scale;
    break;
  case TRC_X:
    v-= reactance(coeff) * scale;
    break;
  case TRC_SMITH:
  //case TRC_ADMIT:
  case TRC_POLAR:
    cartesian_scale(coeff[0], coeff[1], &x, &y, scale);
    goto set_index;
  }
  if (v <  0) v =  0;
  if (v > 10) v = 10;
  x = (i * (WIDTH) + (sweep_points-1)/2) / (sweep_points-1) + CELLOFFSETX;
  y = floatToInt(v * GRIDY);
set_index:
  return INDEX(x, y);
}

static void
format_smith_value(char *buf, int len, const float coeff[2], uint32_t frequency)
{
  // z = (gamma+1)/(gamma-1) * z0
  float z0 = 50;
  float d = z0 / ((1-coeff[0])*(1-coeff[0])+coeff[1]*coeff[1]);
  float zr = ((1+coeff[0])*(1-coeff[0]) - coeff[1]*coeff[1]) * d;
  float zi = 2*coeff[1] * d;
  char prefix;
  float value;
  switch (marker_smith_format) {
  case MS_LIN:
    chsnprintf(buf, len, "%.2f %.1f" S_DEGREE, linear(coeff), phase(coeff));
    break;

  case MS_LOG: {
      float v = logmag(coeff);
      if (v == -INFINITY)
        chsnprintf(buf, len, "-"S_INFINITY" dB");
      else
        chsnprintf(buf, len, "%.1fdB %.1f" S_DEGREE, v, phase(coeff));
    }
    break;
  case MS_REIM:
    chsnprintf(buf, len, "%F%+Fj", coeff[0], coeff[1]);
    break;  

  case MS_RX:
    chsnprintf(buf, len, "%F"S_OHM"%+Fj", zr, zi);
    break;

  case MS_RLC:
    if (zi < 0){// Capacity
      prefix = 'F';
      value = -1 / (2 * M_PI * frequency * zi);
    }
    else {
      prefix = 'H';
      value = zi / (2 * M_PI * frequency);
    }
    chsnprintf(buf, len, "%F"S_OHM" %F%c", zr, value, prefix);
    break;
  }
}

static void
trace_get_value_string(int t, char *buf, int len, float array[POINTS_COUNT][2], int i)
{
  float *coeff = array[i];
  float v;
  char *format;
  switch (trace[t].type) {
  case TRC_LOGMAG:
    format = "%.2fdB";
    v = logmag(coeff);
    break;
  case TRC_PHASE:
    format = "%.3f"S_DEGREE;
    v = phase(coeff);
    break;
  case TRC_DELAY:
    format = "%.2Fs";
    v = groupdelay_from_array(i, array);
    break;
  case TRC_LINEAR:
    format = "%.4f";
    v = linear(coeff);
    break;
  case TRC_SWR:
    format = "%.4f";
    v = swr(coeff);
    break;
  case TRC_REAL:
    format = "%.4f";
    v = coeff[0];
    break;
  case TRC_IMAG:
    format = "%.4fj";
    v = coeff[1];
    break;
  case TRC_R:
    format = "%.2F"S_OHM;
    v = gamma2resistance(coeff);
    break;
  case TRC_X:
    format = "%.2F"S_OHM;
    v = gamma2reactance(coeff);
    break;
  case TRC_SMITH:
    format_smith_value(buf, len, coeff, frequencies[i]);
    return;
    //case TRC_ADMIT:
  case TRC_POLAR:
    chsnprintf(buf, len, "%.2f%+.2fj", coeff[0], coeff[1]);
  default:
    return;
  }
  chsnprintf(buf, len, format, v);
}

static void
trace_get_value_string_delta(int t, char *buf, int len, float array[POINTS_COUNT][2], int index, int index_ref)
{
  float *coeff = array[index];
  float *coeff_ref = array[index_ref];
  float v;
  char *format;
  switch (trace[t].type) {
  case TRC_LOGMAG:
    format = S_DELTA"%.2fdB";
    v = logmag(coeff) - logmag(coeff_ref);
    break;
  case TRC_PHASE:
    format = S_DELTA"%.2f"S_DEGREE;
    v = phase(coeff) - phase(coeff_ref);
    break;
  case TRC_DELAY:
    format = "%.2Fs";
    v = groupdelay_from_array(index, array) - groupdelay_from_array(index_ref, array);
    break;
  case TRC_LINEAR:
    format = S_DELTA"%.3f";
    v = linear(coeff) - linear(coeff_ref);
    break;
  case TRC_SWR:
    format = S_DELTA"%.3f";
    v = swr(coeff);
    if (v!=INFINITY)
      v-=swr(coeff_ref);
    break;
  case TRC_SMITH:
    format_smith_value(buf, len, coeff, frequencies[index]);
    return;
  case TRC_REAL:
    format = S_DELTA"%.3f";
    v = coeff[0] - coeff_ref[0];
    break;
  case TRC_IMAG:
    format = S_DELTA"%.3fj";
    v = coeff[1] - coeff_ref[1];
    break;
  case TRC_R:
    format = "%.2F"S_OHM;
    v = gamma2resistance(coeff);
    break;
  case TRC_X:
    format = "%.2F"S_OHM;
    v = gamma2reactance(coeff);
    break;
  //case TRC_ADMIT:
  case TRC_POLAR:
    chsnprintf(buf, len, "%.2f%+.2fj", coeff[0], coeff[1]);
    return;
  default:
    return;
  }
  chsnprintf(buf, len, format, v);
}

static int
trace_get_info(int t, char *buf, int len)
{
  const char *name = get_trace_typename(t);
  float scale = get_trace_scale(t);
  switch (trace[t].type) {
  case TRC_LOGMAG:
      return chsnprintf(buf, len, "%s %ddB/", name, (int)scale);
  case TRC_PHASE:
      return chsnprintf(buf, len, "%s %d" S_DEGREE "/", name, (int)scale);
  case TRC_SMITH:
  //case TRC_ADMIT:
  case TRC_POLAR:
    if (scale != 1.0)
      return chsnprintf(buf, len, "%s %.1fFS", name, scale);
    else
      return chsnprintf(buf, len, "%s ", name);
  default:
      return chsnprintf(buf, len, "%s %F/", name, scale);
  }
  return 0;
}

static float time_of_index(int idx) {
   return 1.0 / (float)(frequencies[1] - frequencies[0]) / (float)FFT_SIZE * idx;
}

static float distance_of_index(int idx) {
#define SPEED_OF_LIGHT 299792458
   float distance = ((float)idx * (float)SPEED_OF_LIGHT) / ( (float)(frequencies[1] - frequencies[0]) * (float)FFT_SIZE * 2.0);
   return distance * velocity_factor;
}

static inline void
mark_map(int x, int y)
{
  if (y >= 0 && y < MAX_MARKMAP_Y && x >= 0 && x < MAX_MARKMAP_X)
    markmap[current_mappage][y] |= 1<<x;
}

static void
swap_markmap(void)
{
  current_mappage^= 1;
}

static void
clear_markmap(void)
{
  memset(markmap[current_mappage], 0, sizeof markmap[current_mappage]);
}

void
force_set_markmap(void)
{
  memset(markmap[current_mappage], 0xff, sizeof markmap[current_mappage]);
}

void
invalidateRect(int x0, int y0, int x1, int y1){
  x0/=CELLWIDTH;
  x1/=CELLWIDTH;
  y0/=CELLHEIGHT;
  y1/=CELLHEIGHT;
  int x, y;
  for (y=y0; y<=y1; y++)
    for (x=x0; x<=x1; x++)
      mark_map(x, y);
}

static void
mark_cells_from_index(void)
{
  int t;
  /* mark cells between each neighber points */
  for (t = 0; t < TRACES_MAX; t++) {
    if (!trace[t].enabled)
      continue;
    int x0 = CELL_X(trace_index[t][0]);
    int y0 = CELL_Y(trace_index[t][0]);
    int m0 = x0 / CELLWIDTH;
    int n0 = y0 / CELLHEIGHT;
    int i;
    mark_map(m0, n0);
    for (i = 1; i < sweep_points; i++) {
      int x1 = CELL_X(trace_index[t][i]);
      int y1 = CELL_Y(trace_index[t][i]);
      int m1 = x1 / CELLWIDTH;
      int n1 = y1 / CELLHEIGHT;
      while (m0 != m1 || n0 != n1) {
        if (m0 == m1) {
          if (n0 < n1) n0++; else n0--;
        } else if (n0 == n1) {
          if (m0 < m1) m0++; else m0--;
        } else {
          int x = ((m0 < m1) ? (m0 + 1) : m0) * CELLWIDTH;
          int y = ((n0 < n1) ? (n0 + 1) : n0) * CELLHEIGHT;
          int sgn = (n0 < n1) ? 1 : -1;
          if (sgn*(y-y0)*(x1-x0) < sgn*(x-x0)*(y1-y0)) {
            if (m0 < m1) m0++;
            else m0--;
          } else {
            if (n0 < n1) n0++;
            else n0--;
          }
        }
        mark_map(m0, n0);
      }
      x0 = x1;
      y0 = y1;
      m0 = m1;
      n0 = n1;
    }
  }
}

static inline void
markmap_upperarea(void)
{
  // Hardcoded, Text info from upper area
  invalidateRect(0, 0, AREA_WIDTH_NORMAL, 31);
}

#define INSIDE  0b0000;
#define LEFT    0b0001;
#define RIGHT   0b0010;
#define BOTTOM  0b0100;
#define TOP     0b1000;

static uint32_t
_compute_outcode(int w, int h, int x, int y)
{
    uint32_t code = INSIDE;
    if (x < 0) {
        code |= LEFT;
    } else
    if (x > w) {
        code |= RIGHT;
    }
    if (y < 0) {
        code |= BOTTOM;
    } else
    if (y > h) {
        code |= TOP;
    }
    return code;
}

static void
cell_drawline(int w, int h, int x0, int y0, int x1, int y1, int c)
{
  uint32_t outcode1 = _compute_outcode(w, h, x0, y0);
  uint32_t outcode2 = _compute_outcode(w, h, x1, y1);
  if (outcode1 & outcode2) {
      // this line is out of requested area. early return
      return;
  }
  // If outcode1 == 0  && outcode2 == 0, no clipping, not need check
  //outcode1+=outcode2;
  // modifed Bresenham's line algorithm, see https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
  int dx = x1 - x0, sx = 1; if (dx < 0) {dx = -dx; sx = -1;}
  int dy = y1 - y0, sy = 1; if (dy < 0) {dy = -dy; sy = -1;}
  int err = (dx > dy ? dx : -dy) / 2;

  while (1){
    if (//outcode1 == 0 ||
        (y0 >= 0 && y0 < h && x0 >= 0 && x0 < w))
      spi_buffer[y0*w+x0] |= c;
    if (x0 == x1 && y0 == y1)
      break;
    int e2 = err;
    if (e2 > -dx) { err -= dy; x0 += sx; }
    if (e2 <  dy) { err += dx; y0 += sy; }
  }
}

#if 0
int
search_index_range(int x, int y, uint32_t index[POINTS_COUNT], int *i0, int *i1)
{
  int i, j;
  int head = 0;
  int tail = sweep_points;
  i = 0;
  x &= 0x03e0;
  y &= 0x03e0;
  while (head < tail) {
    i = (head + tail) / 2;
    if (x < CELL_X0(index[i]))
      tail = i+1;
    else if (x > CELL_X0(index[i]))
      head = i;
    else if (y < CELL_Y0(index[i]))
      tail = i+1;
    else if (y > CELL_Y0(index[i]))
      head = i;
    else
      break;
  }

  if (x != CELL_X0(index[i]) || y != CELL_Y0(index[i]))
    return FALSE;
    
  j = i;
  while (j > 0 && x == CELL_X0(index[j-1]) && y == CELL_Y0(index[j-1]))
    j--;
  *i0 = j;
  j = i;
  while (j < POINTS_COUNT-1 && x == CELL_X0(index[j+1]) && y == CELL_Y0(index[j+1]))
    j++;
  *i1 = j;
  return TRUE;
}
#endif

static int
search_index_range_x(int x1, int x2, uint32_t index[POINTS_COUNT], int *i0, int *i1)
{
  int i, j;
  int head = 0;
  int tail = sweep_points;
  int idx_x;

  // Search index point in cell
  while (1) {
    i = (head + tail) / 2;
    idx_x = CELL_X(index[i]);
    if (idx_x >= x2){ // index after cell
      if (tail == i)
        return false;
      tail = i;
    }
    else if (idx_x < x1){    // index before cell
      if (head == i)
        return false;
      head = i;
    }
    else  // index in cell (x =< idx_x < cell_end)
      break;
  }
  j = i;
  // Search index left from point
  do{
    j--;
  }while (j > 0 && x1 <= CELL_X(index[j]));
  *i0 = j;
  // Search index right from point
  do{
    i++;
  }while (i < sweep_points-1 && CELL_X(index[i]) < x2);
  *i1 = i;

  return TRUE;
}

#define REFERENCE_WIDTH    5
#define REFERENCE_HEIGHT   5
#define REFERENCE_X_OFFSET 5
#define REFERENCE_Y_OFFSET 2

// Reference bitmap
static const uint8_t reference_bitmap[]={
  0b11000000,
  0b11110000,
  0b11111100,
  0b11110000,
  0b11000000,
};

static void
draw_refpos(int w, int h, int x, int y, int c)
{
  int y0=y, j;
  for (j=0; j<REFERENCE_HEIGHT; j++, y0++){
    if (y0 < 0 || y0 >= h)
      continue;
    int x0=x;
    uint8_t bits = reference_bitmap[j];
    while (bits){
      if (x0 >= 0 && x0 < w)
        spi_buffer[y0*w+x0] = (bits&0x80) ? c : DEFAULT_BG_COLOR;
      x0++;
      bits<<=1;
    }
  }
}

#define MARKER_WIDTH  7
#define MARKER_HEIGHT 10
#define X_MARKER_OFFSET 3
#define Y_MARKER_OFFSET 10
static const uint8_t marker_bitmap[]={
  // Marker 1
  0b11111110,
  0b11101110,
  0b11001110,
  0b11101110,
  0b11101110,
  0b11101110,
  0b11000110,
  0b01111100,
  0b00111000,
  0b00010000,
  // Marker 2
  0b11111110,
  0b11000110,
  0b10111010,
  0b11111010,
  0b11000110,
  0b10111110,
  0b10000010,
  0b01111100,
  0b00111000,
  0b00010000,
  // Marker 3
  0b11111110,
  0b11000110,
  0b10111010,
  0b11100110,
  0b11111010,
  0b10111010,
  0b11000110,
  0b01111100,
  0b00111000,
  0b00010000,
  // Marker 4
  0b11111110,
  0b11110110,
  0b11100110,
  0b11010110,
  0b10110110,
  0b10110110,
  0b10000010,
  0b01110100,
  0b00111000,
  0b00010000,
};

static void
draw_marker(int w, int h, int x, int y, int c, int ch)
{
  int y0=y, j;
  for (j=0;j<MARKER_HEIGHT;j++,y0++){
    int x0=x;
    uint8_t bits = marker_bitmap[ch*10+j];
    bool force_color = false;
    while (bits){
      if (bits&0x80)
        force_color = true;
      if (x0 >= 0 && x0 < w && y0 >= 0 && y0 < h){
        if (bits&0x80)
          spi_buffer[y0*w+x0] = c;
        else if (force_color)
          spi_buffer[y0*w+x0] = DEFAULT_BG_COLOR;
      }
      x0++;
      bits<<=1;
    }
  }
}

static void
markmap_marker(int marker)
{
  int t;
  if (!markers[marker].enabled)
    return;
  for (t = 0; t < TRACES_MAX; t++) {
    if (!trace[t].enabled)
      continue;
    uint32_t index = trace_index[t][markers[marker].index];
    int x = CELL_X(index) - X_MARKER_OFFSET;
    int y = CELL_Y(index) - Y_MARKER_OFFSET;
    invalidateRect(x, y, x+MARKER_WIDTH-1, y+MARKER_HEIGHT-1);
  }
}

static void
markmap_all_markers(void)
{
  int i;
  for (i = 0; i < MARKERS_MAX; i++) {
    if (!markers[i].enabled)
      continue;
    markmap_marker(i);
  }
  markmap_upperarea();
}

void
marker_position(int m, int t, int *x, int *y)
{
  uint32_t index = trace_index[t][markers[m].index];
  *x = CELL_X(index);
  *y = CELL_Y(index);
}

static int greater(int x, int y) { return x > y; }
static int lesser(int x, int y) { return x < y; }

static int (*compare)(int x, int y) = lesser;
int8_t marker_tracking = false;

int
marker_search(void)
{
  int i;
  int found = 0;

  if (uistat.current_trace == -1)
    return -1;

  int value = CELL_Y(trace_index[uistat.current_trace][0]);
  for (i = 0; i < POINTS_COUNT; i++) {
    uint32_t index = trace_index[uistat.current_trace][i];
    if ((*compare)(value, CELL_Y(index))) {
      value = CELL_Y(index);
      found = i;
    }
  }

  return found;
}

void
set_marker_search(int mode)
{
  compare = (mode == 0) ? greater : lesser;
}

int
marker_search_left(int from)
{
  int i;
  int found = -1;

  if (uistat.current_trace == -1)
    return -1;

  int value = CELL_Y(trace_index[uistat.current_trace][from]);
  for (i = from - 1; i >= 0; i--) {
    uint32_t index = trace_index[uistat.current_trace][i];
    if ((*compare)(value, CELL_Y(index)))
      break;
    value = CELL_Y(index);
  }

  for (; i >= 0; i--) {
    uint32_t index = trace_index[uistat.current_trace][i];
    if ((*compare)(CELL_Y(index), value)) {
      break;
    }
    found = i;
    value = CELL_Y(index);
  }
  return found;
}

int
marker_search_right(int from)
{
  int i;
  int found = -1;

  if (uistat.current_trace == -1)
    return -1;

  int value = CELL_Y(trace_index[uistat.current_trace][from]);
  for (i = from + 1; i < POINTS_COUNT; i++) {
    uint32_t index = trace_index[uistat.current_trace][i];
    if ((*compare)(value, CELL_Y(index)))
      break;
    value = CELL_Y(index);
  }

  for (; i < POINTS_COUNT; i++) {
    uint32_t index = trace_index[uistat.current_trace][i];
    if ((*compare)(CELL_Y(index), value)) {
      break;
    }
    found = i;
    value = CELL_Y(index);
  }
  return found;
}

int
search_nearest_index(int x, int y, int t)
{
  uint32_t *index = trace_index[t];
  int min_i = -1;
  int min_d = 1000;
  int i;
  for (i = 0; i < sweep_points; i++) {
    int16_t dx = x - CELL_X(index[i]);
    int16_t dy = y - CELL_Y(index[i]);
    if (dx < 0) dx = -dx;
    if (dy < 0) dy = -dy;
    if (dx > 20 || dy > 20)
      continue;
    int d = dx*dx + dy*dy;
    if (d < min_d) {
      min_d = d;
      min_i = i;
    }
  }

  return min_i;
}

void
plot_into_index(float measured[2][POINTS_COUNT][2])
{
  int t, i;
  for (t = 0; t < TRACES_MAX; t++) {
    if (!trace[t].enabled)
      continue;
    int ch = trace[t].channel;
    for (i = 0; i < sweep_points; i++)
      trace_index[t][i] = trace_into_index(t, i, measured[ch]);
  }
#if 0
  for (t = 0; t < TRACES_MAX; t++)
    if (trace[t].enabled && trace[t].polar)
      quicksort(trace_index[t], 0, sweep_points);
#endif

  mark_cells_from_index();
  markmap_all_markers();
}

static void
draw_cell(int m, int n)
{
  int x0 = m * CELLWIDTH;
  int y0 = n * CELLHEIGHT;
  int w = CELLWIDTH;
  int h = CELLHEIGHT;
  int x, y;
  int i0, i1, i;
  int t;
  // Clip cell by area
  if (x0 + w > area_width)
    w = area_width - x0;
  if (y0 + h > area_height)
    h = area_height - y0;
  if (w <= 0 || h <= 0)
    return;

  PULSE;
  // Clear buffer
  memset(spi_buffer, DEFAULT_BG_COLOR, sizeof spi_buffer);
  uint16_t c = config.grid_color;
  // Draw grid
  for (t = 0; t < TRACES_MAX; t++) {
    if (!trace[t].enabled)
      continue;
    uint32_t trace_type = (1<<trace[t].type);
    // Draw rectangular plot
    if (trace_type&RECTANGULAR_GRID_MASK){
      for (x = 0; x < w; x++) {
        if (rectangular_grid_x(x+x0)){
          for (y = 0; y < h; y++)
            spi_buffer[y * w + x] = c;
        }
      }
      for (y = 0; y < h; y++) {
        if (rectangular_grid_y(y+y0)){
          for (x = 0; x < w; x++)
            if (x+x0 >= CELLOFFSETX && x+x0 <= WIDTH+CELLOFFSETX)
              spi_buffer[y * w + x] = c;
        }
      }
      continue;
    }
    if(trace_type&(1<<TRC_SMITH)){
      for (y = 0; y < h; y++)
        for (x = 0; x < w; x++)
          if (smith_grid(x+x0, y+y0))
            spi_buffer[y * w + x] = c;
      continue;
    }
    if(trace_type&(1<<TRC_POLAR)){
      for (y = 0; y < h; y++)
        for (x = 0; x < w; x++)
          if (polar_grid(x+x0, y+y0))
            spi_buffer[y * w + x] = c;
      continue;
    }
#if 0
    if(trace_type&(1<<TRC_ADMIT)){
      for (y = 0; y < h; y++)
        for (x = 0; x < w; x++)
          if (smith_grid3(x+x0, y+y0)
           //   smith_grid2(x+x0, y+y0, 0.5))
            spi_buffer[y * w + x] = c;
      continue;
    }
#endif
  }
  PULSE;

#if 1
  // Draw traces
  for (t = 0; t < TRACES_MAX; t++) {
    if (!trace[t].enabled)
      continue;
    c = config.trace_color[t];
    // draw polar plot (check all points)
    i0 = 0; i1=0;
    uint32_t trace_type = (1<<trace[t].type);
    if (trace_type & ((1<<TRC_SMITH)|(1<<TRC_POLAR)))
      i1 = sweep_points-1;
    else // draw rectangular plot (search index range in cell)
      search_index_range_x(x0, x0+w, trace_index[t], &i0, &i1);
    for (i=i0; i < i1; i++) {
      int x1 = CELL_X(trace_index[t][i  ]) - x0;
      int y1 = CELL_Y(trace_index[t][i  ]) - y0;
      int x2 = CELL_X(trace_index[t][i+1]) - x0;
      int y2 = CELL_Y(trace_index[t][i+1]) - y0;
      cell_drawline(w, h, x1, y1, x2, y2, c);
    }
  }
#endif
  PULSE;
  //draw marker symbols on each trace
  for (i = 0; i < MARKERS_MAX; i++) {
    if (!markers[i].enabled)
      continue;
    for (t = 0; t < TRACES_MAX; t++) {
      if (!trace[t].enabled)
        continue;
      uint32_t index = trace_index[t][markers[i].index];
      int x = CELL_X(index) - x0 - X_MARKER_OFFSET;
      int y = CELL_Y(index) - y0 - Y_MARKER_OFFSET;
      // Check marker icon on cell
      if (x+MARKER_WIDTH>=0 && x-MARKER_WIDTH<w && y+MARKER_HEIGHT>=0 && y-MARKER_HEIGHT<h)
        draw_marker(w, h, x, y, config.trace_color[t], i);
    }
  }
  // Draw trace and marker info on the top
  if (n == 0)
    cell_draw_marker_info(m, n, w, h);

  PULSE;

  // Draw refrence position
  for (t = 0; t < TRACES_MAX; t++) {
    if (!trace[t].enabled)
      continue;
    uint32_t trace_type = (1<<trace[t].type);
    if (trace_type&((1<<TRC_SMITH)|(1<<TRC_POLAR)))
      continue;
    int x = 0 - x0 + CELLOFFSETX - REFERENCE_X_OFFSET;
    if (x+REFERENCE_WIDTH>=0 && x-REFERENCE_WIDTH<w){
      int y = 10*GRIDY - floatToInt((get_trace_refpos(t) * GRIDY)) - y0 - REFERENCE_Y_OFFSET;
      if (y+REFERENCE_HEIGHT>=0 && y-REFERENCE_HEIGHT<h)
        draw_refpos(w, h, x, y, config.trace_color[t]);
    }
  }
  // Draw cell
  ili9341_bulk(OFFSETX + x0, OFFSETY + y0, w, h);
}

static void
draw_all_cells(bool flush_markmap)
{
  int m, n;
  for (m = 0; m < (area_width+CELLWIDTH-1) / CELLWIDTH; m++)
    for (n = 0; n < (area_height+CELLHEIGHT-1) / CELLHEIGHT; n++) {
      uint16_t bit = 1<<m;
      if ((markmap[0][n] & bit) || (markmap[1][n] & bit)){
        draw_cell(m, n);
//        ili9341_fill(m*CELLWIDTH+10, n*CELLHEIGHT, 2, 2, RGB565(255,0,0));
      }
//    else
//      ili9341_fill(m*CELLWIDTH+10, n*CELLHEIGHT, 2, 2, RGB565(0,255,0));
    }

  if (flush_markmap) {
    // keep current map for update
    swap_markmap();
    // clear map for next plotting
    clear_markmap();
  }
}

void
draw_all(bool flush)
{
  if (redraw_request & REDRAW_CELLS)
    draw_all_cells(flush);
  if (redraw_request & REDRAW_FREQUENCY)
    draw_frequencies();
  if (redraw_request & REDRAW_CAL_STATUS)
    draw_cal_status();
  redraw_request = 0;
}

void
redraw_marker(int marker, int update_info)
{
  if (marker < 0)
    return;
  // mark map on new position of marker
  markmap_marker(marker);

  // mark cells on marker info
  if (update_info)
    markmap_upperarea();

  draw_all_cells(TRUE);
}

void
request_to_draw_cells_behind_menu(void)
{
  // Values Hardcoded from ui.c
  invalidateRect(320-70, 0, 319, 239);
  redraw_request |= REDRAW_CELLS;
}

void
request_to_draw_cells_behind_numeric_input(void)
{
  // Values Hardcoded from ui.c
  invalidateRect(0, 240-32, 319, 239);
  redraw_request |= REDRAW_CELLS;
}

static int
cell_drawchar(int w, int h, uint8_t ch, int x, int y)
{
  uint8_t bits;
  int c, r, ch_size;
  const uint8_t *char_buf = FONT_GET_DATA(ch);
  ch_size=FONT_GET_WIDTH(ch);
  if (y <= -FONT_GET_HEIGHT || y >= h || x <= -ch_size || x >= w)
    return ch_size;
  for(c = 0; c < FONT_GET_HEIGHT; c++) {
    bits = *char_buf++;
    if ((y + c) < 0 || (y + c) >= h)
      continue;
    for (r = 0; r < ch_size; r++) {
      if ((x+r) >= 0 && (x+r) < w && (0x80 & bits))
        spi_buffer[(y+c)*w + (x+r)] = foreground_color;
      bits <<= 1;
    }
  }
  return ch_size;
}

static void
cell_drawstring(int w, int h, char *str, int x, int y)
{
  while (*str) {
    x += cell_drawchar(w, h, *str, x, y);
    str++;
  }
}

static void
cell_draw_marker_info(int m, int n, int w, int h)
{
  char buf[32];
  int t;
  if (n != 0)
    return;
  if (active_marker < 0)
    return;
  int idx = markers[active_marker].index;
  int j = 0;
  if (active_marker != -1 && previous_marker != -1 && uistat.current_trace != -1) {
    int t = uistat.current_trace;
    int mk;
    for (mk = 0; mk < MARKERS_MAX; mk++) {
      if (!markers[mk].enabled)
        continue;
      int xpos = 1 + (j%2)*146;
      int ypos = 1 + (j/2)*8;
      xpos -= m * CELLWIDTH - CELLOFFSETX;
      ypos -= n * CELLHEIGHT;

      setForegroundColor(config.trace_color[t]);
      if (mk == active_marker)
        cell_drawstring(w, h, S_SARROW, xpos, ypos);
      xpos += 5;
      chsnprintf(buf, sizeof buf, "M%d", mk+1);
      cell_drawstring(w, h, buf, xpos, ypos);
      xpos += 13;
      //trace_get_info(t, buf, sizeof buf);
      uint32_t freq = frequencies[markers[mk].index];
      if (uistat.marker_delta && mk != active_marker) {
        uint32_t freq1 = frequencies[markers[active_marker].index];
        uint32_t delta = freq > freq1 ? freq - freq1 : freq1 - freq;
        chsnprintf(buf, sizeof buf, S_DELTA"%.9qHz", delta);
      } else {
        chsnprintf(buf, sizeof buf, "%.10qHz", freq);
      }
      cell_drawstring(w, h, buf, xpos, ypos);
      xpos += 67;
      if (uistat.marker_delta && mk != active_marker)
        trace_get_value_string_delta(t, buf, sizeof buf, measured[trace[t].channel], markers[mk].index, markers[active_marker].index);
      else
        trace_get_value_string(t, buf, sizeof buf, measured[trace[t].channel], markers[mk].index);
      setForegroundColor(DEFAULT_FG_COLOR);
      cell_drawstring(w, h, buf, xpos, ypos);
      j++;
    }

    // draw marker delta
    if (!uistat.marker_delta && previous_marker >= 0 && active_marker != previous_marker && markers[previous_marker].enabled) {
      int idx0 = markers[previous_marker].index;
      int xpos = 180;
      int ypos = 1 + (j/2)*8;
      xpos -= m * CELLWIDTH -CELLOFFSETX;
      ypos -= n * CELLHEIGHT;
      chsnprintf(buf, sizeof buf, S_DELTA"%d-%d", active_marker+1, previous_marker+1);
      setForegroundColor(DEFAULT_FG_COLOR);
      cell_drawstring(w, h, buf, xpos, ypos);
      xpos += 24;
      if ((domain_mode & DOMAIN_MODE) == DOMAIN_FREQ) {
        uint32_t freq  = frequencies[idx];
        uint32_t freq1 = frequencies[idx0];
        uint32_t delta = freq > freq1 ? freq - freq1 : freq1 - freq;
        chsnprintf(buf, sizeof buf, "%c%.13qHz", freq >= freq1 ? '+' : '-', delta);
      } else {
        chsnprintf(buf, sizeof buf, "%Fs (%Fm)", time_of_index(idx) - time_of_index(idx0), distance_of_index(idx) - distance_of_index(idx0));
      }
      cell_drawstring(w, h, buf, xpos, ypos);
    }
  } else {
    for (t = 0; t < TRACES_MAX; t++) {
      if (!trace[t].enabled)
        continue;
      int xpos = 1 + (j%2)*146;
      int ypos = 1 + (j/2)*8;
      xpos -= m * CELLWIDTH -CELLOFFSETX;
      ypos -= n * CELLHEIGHT;
      setForegroundColor(config.trace_color[t]);
      if (t == uistat.current_trace)
        cell_drawstring(w, h, S_SARROW, xpos, ypos);
      xpos += 5;
      chsnprintf(buf, sizeof buf, "CH%d", trace[t].channel);
      cell_drawstring(w, h, buf, xpos, ypos);
      xpos += 19;

      int n = trace_get_info(t, buf, sizeof buf);
      cell_drawstring(w, h, buf, xpos, ypos);
      xpos += n * 5 + 2;
      //xpos += 60;
      trace_get_value_string(t, buf, sizeof buf, measured[trace[t].channel], idx);
      setForegroundColor(DEFAULT_FG_COLOR);
      cell_drawstring(w, h, buf, xpos, ypos);
      j++;
    }

    // draw marker frequency
    int xpos = 185;
    int ypos = 1 + (j/2)*8;
    xpos -= m * CELLWIDTH -CELLOFFSETX;
    ypos -= n * CELLHEIGHT;
    setForegroundColor(DEFAULT_FG_COLOR);
    if (uistat.lever_mode == LM_MARKER)
      cell_drawstring(w, h, S_SARROW, xpos, ypos);
    xpos += 5;
    chsnprintf(buf, sizeof buf, "M%d:", active_marker+1);
    cell_drawstring(w, h, buf, xpos, ypos);
    xpos += 19;

    if ((domain_mode & DOMAIN_MODE) == DOMAIN_FREQ) {
      chsnprintf(buf, sizeof buf, "%qHz", frequencies[idx]);
    } else {
      chsnprintf(buf, sizeof buf, "%Fs (%Fm)", time_of_index(idx), distance_of_index(idx));
    }
    cell_drawstring(w, h, buf, xpos, ypos);
  }
  setForegroundColor(DEFAULT_FG_COLOR);
  if (electrical_delay != 0) {
    // draw electrical delay
    int xpos = 21;
    int ypos = 1 + ((j+1)/2)*8;
    xpos -= m * CELLWIDTH -CELLOFFSETX;
    ypos -= n * CELLHEIGHT;

    float light_speed_ps = 299792458e-12; //(m/ps)
    chsnprintf(buf, sizeof buf, "Edelay %Fs %Fm", electrical_delay * 1e-12,
                                                  electrical_delay * light_speed_ps * velocity_factor);
    cell_drawstring(w, h, buf, xpos, ypos);
  }
}

void
draw_frequencies(void)
{
  char buf1[32];
  char buf2[32];buf2[0]=0;
  if ((domain_mode & DOMAIN_MODE) == DOMAIN_FREQ) {
      if (frequency0 < frequency1) {
        chsnprintf(buf1, sizeof(buf1), " START %qHz", frequency0);
        chsnprintf(buf2, sizeof(buf2), " STOP %qHz", frequency1);
      } else if (frequency0 > frequency1) {
        chsnprintf(buf1, sizeof(buf1), " CENTER %qHz", frequency0/2 + frequency1/2);
        chsnprintf(buf2, sizeof(buf2), " SPAN %qHz", frequency0 - frequency1);
      } else {
        chsnprintf(buf1, sizeof(buf1), " CW %qHz", frequency0);
      }
  } else {
      chsnprintf(buf1, sizeof(buf1), " START 0s");
      chsnprintf(buf2, sizeof(buf2), "STOP %Fs (%Fm)", time_of_index(POINTS_COUNT-1), distance_of_index(POINTS_COUNT-1));
  }
  setForegroundColor(DEFAULT_FG_COLOR);
  setBackgroundColor(DEFAULT_BG_COLOR);
  ili9341_fill(0, 232, 320, 8, DEFAULT_BG_COLOR);
  if (uistat.lever_mode == LM_CENTER)
	buf1[0] = S_SARROW[0];
  if (uistat.lever_mode == LM_SPAN)
	buf2[0] = S_SARROW[0];
  ili9341_drawstring(buf1, OFFSETX, 232);
  ili9341_drawstring(buf2, 200, 232);
}

void
draw_cal_status(void)
{
  int x = 0;
  int y = 100;
#define YSTEP 8
  setForegroundColor(DEFAULT_FG_COLOR);
  setBackgroundColor(DEFAULT_BG_COLOR);
  ili9341_fill(0, y, 10, 6*YSTEP, DEFAULT_BG_COLOR);
  if (cal_status & CALSTAT_APPLY) {
    char c[3] = "C0";
    c[1] += lastsaveid;
    if (cal_status & CALSTAT_INTERPOLATED)
      c[0] = 'c';
    else if (active_props == &current_props)
      c[1] = '*';
    ili9341_drawstring(c, x, y);
    y += YSTEP;
  }

  if (cal_status & CALSTAT_ED) {
    ili9341_drawstring("D", x, y);
    y += YSTEP;
  }
  if (cal_status & CALSTAT_ER) {
    ili9341_drawstring("R", x, y);
    y += YSTEP;
  }
  if (cal_status & CALSTAT_ES) {
    ili9341_drawstring("S", x, y);
    y += YSTEP;
  }
  if (cal_status & CALSTAT_ET) {
    ili9341_drawstring("T", x, y);
    y += YSTEP;
  }
  if (cal_status & CALSTAT_EX) {
    ili9341_drawstring("X", x, y);
    y += YSTEP;
  }
}

// Draw battery level
#define BATTERY_TOP_LEVEL       4100
#define BATTERY_BOTTOM_LEVEL    3100
#define BATTERY_WARNING_LEVEL   3300

void
draw_battery_status(void)
{
  if (vbat<=0)
    return;
  uint8_t string_buf[25];
  // Set battery color
  setForegroundColor(vbat < BATTERY_WARNING_LEVEL ? DEFAULT_LOW_BAT_COLOR : DEFAULT_NORMAL_BAT_COLOR);
  setBackgroundColor(DEFAULT_BG_COLOR);
//  chsnprintf(string_buf, sizeof string_buf, "V:%d", vbat);
//  ili9341_drawstringV(string_buf, 1, 60);
  // Prepare battery bitmap image
  // Battery top
  int x=0;
  string_buf[x++] = 0b00111100;
  string_buf[x++] = 0b00100100;
  string_buf[x++] = 0b11111111;
//  string_buf[x++] = 0b10000001;
  // Fill battery status
  for (int power=BATTERY_TOP_LEVEL; power > BATTERY_BOTTOM_LEVEL; power-=100)
    string_buf[x++] = (power > vbat) ? 0b10000001 : // Empty line
                                       0b11111111;  // Full line
  // Battery bottom
//  string_buf[x++] = 0b10000001;
  string_buf[x++] = 0b11111111;
  // Draw battery
  blit8BitWidthBitmap(0, 1, 8, x, string_buf);
}

void
request_to_redraw_grid(void)
{
  force_set_markmap();
  redraw_request |= REDRAW_CELLS;
}

void
redraw_frame(void)
{
  ili9341_fill(0, 0, 320, 240, DEFAULT_BG_COLOR);
  draw_frequencies();
  draw_cal_status();
}

void
plot_init(void)
{
  force_set_markmap();
}
