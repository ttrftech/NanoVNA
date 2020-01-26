#include <math.h>
#include <string.h>
#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "nanovna.h"

#define SWAP(x,y) do { int z=x; x = y; y = z; } while(0)

static void cell_draw_marker_info(int m, int n, int w, int h);
void frequency_string(char *buf, size_t len, uint32_t freq, char *prefix);
void frequency_string_short(char *buf, size_t len, int32_t freq, char prefix);
void markmap_all_markers(void);

//#define GRID_COLOR 0x0863
//uint16_t grid_color = 0x1084;

/* indicate dirty cells */
uint16_t markmap[2][8];
uint16_t current_mappage = 0;

int32_t fgrid = 50000000;
int16_t grid_offset;
int16_t grid_width;

int area_width = AREA_WIDTH_NORMAL;
int area_height = HEIGHT+1;

#define GRID_RECTANGULAR (1<<0)
#define GRID_SMITH       (1<<1)
#define GRID_ADMIT       (1<<2)
#define GRID_POLAR       (1<<3)


#define CELLWIDTH 32
#define CELLHEIGHT 32

/*
 * CELL_X0[27:31] cell position
 * CELL_Y0[22:26]
 * CELL_N[10:21] original order
 * CELL_X[5:9] position in the cell
 * CELL_Y[0:4]
 */
uint32_t trace_index[TRACES_MAX][POINTS_COUNT];

#define INDEX(x, y, n) \
  ((((x)&0x03e0UL)<<22) | (((y)&0x03e0UL)<<17) | (((n)&0x0fffUL)<<10)  \
 | (((x)&0x1fUL)<<5) | ((y)&0x1fUL))

#define CELL_X(i) (int)((((i)>>5)&0x1f) | (((i)>>22)&0x03e0))
#define CELL_Y(i) (int)(((i)&0x1f) | (((i)>>17)&0x03e0))
#define CELL_N(i) (int)(((i)>>10)&0xfff)

#define CELL_X0(i) (int)(((i)>>22)&0x03e0)
#define CELL_Y0(i) (int)(((i)>>17)&0x03e0)

#define CELL_P(i, x, y) (((((x)&0x03e0UL)<<22) | (((y)&0x03e0UL)<<17)) == ((i)&0xffc00000UL))

//#define floatToInt(v) ((int)(v))
int floatToInt(float v){
	if (v < 0) return v-0.5;
	if (v > 0) return v+0.5;
	return 0;
}

void update_grid(void)
{
  uint32_t gdigit = 100000000;
  uint32_t fstart, fspan;
  uint32_t grid;
  if (frequency0 <= frequency1) {
    fstart = frequency0;
    fspan = frequency1 - frequency0;
  } else {
    fstart = frequency1;
    fspan = frequency0 - frequency1;
  }
  
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
  fgrid = grid;

  grid_offset = (WIDTH-1) * ((fstart % fgrid) / 100) / (fspan / 100);
  grid_width = (WIDTH-1) * (fgrid / 100) / (fspan / 1000);

  force_set_markmap();
  redraw_request |= REDRAW_FREQUENCY;
}

static inline int
circle_inout(int x, int y, int r)
{
  int d = x*x + y*y - r*r;
  if (d <= -r)
    return 1;
  if (d > r)
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
int
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
int
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

int
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

#if 0
int
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
  if (x < 0)
    return 0;
  if (x == 0 || x == WIDTH-1)
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
float logmag(const float *v)
{
  return log10f(v[0]*v[0] + v[1]*v[1]) * 10;
}

/*
 * calculate phase[-2:2] of coefficient
 */ 
float phase(const float *v)
{
  return 2 * atan2f(v[1], v[0]) / M_PI * 90;
}

/*
 * calculate groupdelay
 */
float groupdelay(const float *v, const float *w, float deltaf)
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
float linear(const float *v)
{
  return - sqrtf(v[0]*v[0] + v[1]*v[1]);
}

/*
 * calculate vswr; (1+gamma)/(1-gamma)
 */ 
float swr(const float *v)
{
  float x = sqrtf(v[0]*v[0] + v[1]*v[1]);
  if (x >= 1)
    return INFINITY;
  return (1 + x)/(1 - x);
}

float resitance(const float *v) {
  float z0 = 50;
  float d = z0 / ((1-v[0])*(1-v[0])+v[1]*v[1]);
  float zr = ((1+v[0])*(1-v[0]) - v[1]*v[1]) * d;
  return zr;
}

float reactance(const float *v) {
  float z0 = 50;
  float d = z0 / ((1-v[0])*(1-v[0])+v[1]*v[1]);
  float zi = 2*v[1] * d;
  return zi;
}

void
cartesian_scale(float re, float im, int *xp, int *yp, float scale)
{
  //float scale = 4e-3;
  int x = floatToInt(re * P_RADIUS * scale);
  int y = floatToInt(im * P_RADIUS * scale);
  if (x < -P_RADIUS) x = -P_RADIUS;
  if (y < -P_RADIUS) y = -P_RADIUS;
  if (x > P_RADIUS) x = P_RADIUS;
  if (y > P_RADIUS) y = P_RADIUS;
  *xp = P_CENTER_X + x;
  *yp = P_CENTER_Y - y;
}

float
groupdelay_from_array(int i, float array[POINTS_COUNT][2])
{
/*
  if (i == 0) {
    float deltaf = frequencies[1] - frequencies[0];
    return groupdelay(array[0], array[1], deltaf);
  } else if (i == 100) {
    float deltaf = frequencies[i] - frequencies[i-1];
    return groupdelay(array[i-1], array[i], deltaf);
  } else {
    float deltaf = frequencies[i+1] - frequencies[i-1];
    return groupdelay(array[i-1], array[i+1], deltaf);
  }
*/
  int bottom = (i ==   0) ?   0 : i - 1;
  int top    = (i == POINTS_COUNT-1) ? POINTS_COUNT-1 : i + 1;
  float deltaf = frequencies[top] - frequencies[bottom];
  return groupdelay(array[bottom], array[top], deltaf);
}

uint32_t
trace_into_index(int x, int t, int i, float array[POINTS_COUNT][2])
{
  int y = 0;
  float v = 0;
  float *coeff = array[i];
  float refpos = 10 - get_trace_refpos(t);
  float scale = 1 / get_trace_scale(t);
  switch (trace[t].type) {
  case TRC_LOGMAG:
    v = refpos - logmag(coeff) * scale;
    break;
  case TRC_PHASE:
    v = refpos - phase(coeff) * scale;
    break;
  case TRC_DELAY:
    v = refpos - groupdelay_from_array(i, array) * scale;
    break;
  case TRC_LINEAR:
    v = refpos + linear(coeff) * scale;
    break;
  case TRC_SWR:
    v = refpos+ (1 - swr(coeff)) * scale;
    break;
  case TRC_REAL:
    v = refpos - coeff[0] * scale;
    break;
  case TRC_IMAG:
    v = refpos - coeff[1] * scale;
    break;
  case TRC_R:
    v = refpos - resitance(coeff) * scale;
    break;
  case TRC_X:
    v = refpos - reactance(coeff) * scale;
    break;
  case TRC_SMITH:
  //case TRC_ADMIT:
  case TRC_POLAR:
    cartesian_scale(coeff[0], coeff[1], &x, &y, scale);
    return INDEX(x +CELLOFFSETX, y, i);
    break;
  }
  if (v < 0) v = 0;
  if (v > 10) v = 10;
  y = floatToInt(v * GRIDY);
  return INDEX(x +CELLOFFSETX, y, i);
}

static int
string_value_with_prefix(char *buf, int len, float val, char unit)
{
  char prefix;
  int n = 0;
  if (val < 0) {
    val = -val;
    *buf = '-';
    n++;
    len--;
  }
  if (val == INFINITY){
	  prefix = S_INFINITY[0];
  }
  else {
	  if (val < 1e-12) {
		  prefix = 'f';
		  val *= 1e15;
	  } else if (val < 1e-9) {
	  	 prefix = 'p';
	  	 val *= 1e12;
	  } else if (val < 1e-6) {
		  prefix = 'n';
		  val *= 1e9;
	  } else if (val < 1e-3) {
		  prefix = S_MICRO[0];
		  val *= 1e6;
	  } else if (val < 1) {
		  prefix = 'm';
		  val *= 1e3;
	  } else if (val < 1e3) {
		  prefix = 0;
	  } else if (val < 1e6) {
		  prefix = 'k';
		  val /= 1e3;
	  } else if (val < 1e9) {
		  prefix = 'M';
		  val /= 1e6;
	  } else {
		  prefix = 'G';
		  val /= 1e9;
	  }

	  if (val < 10) {
		  n += chsnprintf(&buf[n], len, "%.2f", val);
	  } else if (val < 100) {
		  n += chsnprintf(&buf[n], len, "%.1f", val);
	  } else {
		  n += chsnprintf(&buf[n], len, "%d", (int)val);
	  }
  }
  if (prefix)
    buf[n++] = prefix;
  if (unit)
    buf[n++] = unit;
  buf[n] = '\0';
  return n;
}

#define PI2 6.283184

static void
format_smith_value(char *buf, int len, const float coeff[2], uint32_t frequency)
{
  // z = (gamma+1)/(gamma-1) * z0
  float z0 = 50;
  float d = z0 / ((1-coeff[0])*(1-coeff[0])+coeff[1]*coeff[1]);
  float zr = ((1+coeff[0])*(1-coeff[0]) - coeff[1]*coeff[1]) * d;
  float zi = 2*coeff[1] * d;
  int n;

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
    n = string_value_with_prefix(buf, len, coeff[0], '\0');
    if (coeff[1] >= 0) buf[n++] = '+';
    string_value_with_prefix(buf+n, len-n, coeff[1], 'j');
    break;  

  case MS_RX:
    n = string_value_with_prefix(buf, len, zr, S_OHM[0]);
    if (zi >= 0)
      buf[n++] = ' ';
    string_value_with_prefix(buf+n, len-n, zi, 'j');
    break;

  case MS_RLC:
    n = string_value_with_prefix(buf, len, zr, S_OHM[0]);
    buf[n++] = ' ';

    char prefix;
    float value;
    if (zi < 0){// Capacity
    	prefix = 'F';
    	value = -1 / (PI2 * frequency * zi);
    }
    else {
    	prefix = 'H';
    	value = zi / (PI2 * frequency);
    }
    string_value_with_prefix(buf+n, len-n, value, prefix);
    break;
  }
}

static void
gamma2resistance(char *buf, int len, const float coeff[2])
{
  float z0 = 50;
  float d = z0 / ((1-coeff[0])*(1-coeff[0])+coeff[1]*coeff[1]);
  float zr = ((1+coeff[0])*(1-coeff[0]) - coeff[1]*coeff[1]) * d;
  string_value_with_prefix(buf, len, zr, S_OHM[0]);
}

static void
gamma2reactance(char *buf, int len, const float coeff[2])
{
  float z0 = 50;
  float d = z0 / ((1-coeff[0])*(1-coeff[0])+coeff[1]*coeff[1]);
  float zi = 2*coeff[1] * d;
  string_value_with_prefix(buf, len, zi, S_OHM[0]);
}

static void
trace_get_value_string(int t, char *buf, int len, float array[POINTS_COUNT][2], int i)
{
  float *coeff = array[i];
  float v;
  switch (trace[t].type) {
  case TRC_LOGMAG:
    v = logmag(coeff);
    if (v == -INFINITY)
      chsnprintf(buf, len, "-"S_INFINITY" dB");
    else
      chsnprintf(buf, len, "%.2fdB", v);
    break;
  case TRC_PHASE:
    v = phase(coeff);
    chsnprintf(buf, len, "%.2f" S_DEGREE, v);
    break;
  case TRC_DELAY:
    v = groupdelay_from_array(i, array);
    string_value_with_prefix(buf, len, v, 's');
    break;
  case TRC_LINEAR:
    v = linear(coeff);
    chsnprintf(buf, len, "%.2f", v);
    break;
  case TRC_SWR:
    v = swr(coeff);
    if (v == INFINITY)
    	chsnprintf(buf, len, S_INFINITY);
    else
    	chsnprintf(buf, len, "%.2f", v);
    break;
  case TRC_SMITH:
    format_smith_value(buf, len, coeff, frequencies[i]);
    break;
  case TRC_REAL:
    chsnprintf(buf, len, "%.2f", coeff[0]);
    break;
  case TRC_IMAG:
    chsnprintf(buf, len, "%.2fj", coeff[1]);
    break;
  case TRC_R:
    gamma2resistance(buf, len, coeff);
    break;
  case TRC_X:
    gamma2reactance(buf, len, coeff);
    break;
  //case TRC_ADMIT:
  case TRC_POLAR:
    chsnprintf(buf, len, "%.2f %.2fj", coeff[0], coeff[1]);
    break;
  }
}

static void
trace_get_value_string_delta(int t, char *buf, int len, float array[POINTS_COUNT][2], int index, int index_ref)
{
  float *coeff = array[index];
  float *coeff_ref = array[index_ref];
  float v;
  switch (trace[t].type) {
  case TRC_LOGMAG:
    v = logmag(coeff) - logmag(coeff_ref);
    if (v == -INFINITY)
      chsnprintf(buf, len, S_DELTA "-"S_INFINITY" dB");
    else
      chsnprintf(buf, len, S_DELTA "%.2fdB", v);
    break;
  case TRC_PHASE:
    v = phase(coeff) - phase(coeff_ref);
    chsnprintf(buf, len, S_DELTA "%.2f" S_DEGREE, v);
    break;
  case TRC_DELAY:
    v = groupdelay_from_array(index, array) - groupdelay_from_array(index_ref, array);
    string_value_with_prefix(buf, len, v, 's');
    break;
  case TRC_LINEAR:
    v = linear(coeff) - linear(coeff_ref);
    chsnprintf(buf, len, S_DELTA "%.2f", v);
    break;
  case TRC_SWR:
    v = swr(coeff) - swr(coeff_ref);
    chsnprintf(buf, len, S_DELTA "%.2f", v);
    break;
  case TRC_SMITH:
    format_smith_value(buf, len, coeff, frequencies[index]);
    break;
  case TRC_REAL:
    chsnprintf(buf, len, S_DELTA "%.2f", coeff[0] - coeff_ref[0]);
    break;
  case TRC_IMAG:
    chsnprintf(buf, len, S_DELTA "%.2fj", coeff[1] - coeff_ref[1]);
    break;
  case TRC_R:
    gamma2resistance(buf, len, coeff);
    break;
  case TRC_X:
    gamma2reactance(buf, len, coeff);
    break;
  //case TRC_ADMIT:
  case TRC_POLAR:
    chsnprintf(buf, len, "%.2f %.2fj", coeff[0], coeff[1]);
    break;
  }
}

void
trace_get_info(int t, char *buf, int len)
{
  const char *type = get_trace_typename(t);
  int n;
  switch (trace[t].type) {
  case TRC_LOGMAG:
    chsnprintf(buf, len, "%s %ddB/", type, (int)get_trace_scale(t));
    break;
  case TRC_PHASE:
    chsnprintf(buf, len, "%s %d" S_DEGREE "/", type, (int)get_trace_scale(t));
    break;
  case TRC_SMITH:
  //case TRC_ADMIT:
  case TRC_POLAR:
    chsnprintf(buf, len, "%s %.1fFS", type, get_trace_scale(t));
    break;
  default:
    n = chsnprintf(buf, len, "%s ", type);
    string_value_with_prefix(buf+n, len-n, get_trace_scale(t), '/');
    break;
  }
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
  if (y >= 0 && y < 8 && x >= 0 && x < 16)
    markmap[current_mappage][y] |= 1<<x;
}

static inline int
is_mapmarked(int x, int y)
{
  uint16_t bit = 1<<x;
  return (markmap[0][y] & bit) || (markmap[1][y] & bit);
}

static inline void
markmap_upperarea(void)
{
  markmap[current_mappage][0] |= 0xffff;
}

static inline void
swap_markmap(void)
{
  current_mappage = 1 - current_mappage;
}

static inline void
clear_markmap(void)
{
  memset(markmap[current_mappage], 0, sizeof markmap[current_mappage]);
}

inline void
force_set_markmap(void)
{
  memset(markmap[current_mappage], 0xff, sizeof markmap[current_mappage]);
}

void
mark_cells_from_index(void)
{
  int t;
  /* mark cells between each neighber points */
  for (t = 0; t < TRACES_MAX; t++) {
    if (!trace[t].enabled)
      continue;
    int x0 = CELL_X(trace_index[t][0]);
    int y0 = CELL_Y(trace_index[t][0]);
    int m0 = x0 >> 5;
    int n0 = y0 >> 5;
    int i;
    mark_map(m0, n0);
    for (i = 1; i < sweep_points; i++) {
      int x1 = CELL_X(trace_index[t][i]);
      int y1 = CELL_Y(trace_index[t][i]);
      int m1 = x1 >> 5;
      int n1 = y1 >> 5;
      while (m0 != m1 || n0 != n1) {
        if (m0 == m1) {
          if (n0 < n1) n0++; else n0--;
        } else if (n0 == n1) {
          if (m0 < m1) m0++; else m0--;
        } else {
          int x = (m0 < m1) ? (m0 + 1)<<5 : m0<<5;
          int y = (n0 < n1) ? (n0 + 1)<<5 : n0<<5;
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

void plot_into_index(float measured[2][POINTS_COUNT][2])
{
  int i, t;
  for (i = 0; i < sweep_points; i++) {
    int x = (i * (WIDTH-1) + sweep_points/2) / (sweep_points-1);
    for (t = 0; t < TRACES_MAX; t++) {
      if (!trace[t].enabled)
        continue;
      int n = trace[t].channel;
      trace_index[t][i] = trace_into_index(x, t, i, measured[n]);
    }
  }
#if 0
  for (t = 0; t < TRACES_MAX; t++)
    if (trace[t].enabled && trace[t].polar)
      quicksort(trace_index[t], 0, sweep_points);
#endif

  mark_cells_from_index();
  markmap_all_markers();
}

const uint8_t INSIDE = 0b0000;
const uint8_t LEFT   = 0b0001;
const uint8_t RIGHT  = 0b0010;
const uint8_t BOTTOM = 0b0100;
const uint8_t TOP    = 0b1000;

inline static uint8_t
_compute_outcode(int w, int h, int x, int y)
{
    uint8_t code = 0;
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
  uint8_t outcode0 = _compute_outcode(w, h, x0, y0);
  uint8_t outcode1 = _compute_outcode(w, h, x1, y1);

  if (outcode0 & outcode1) {
      // this line is out of requested area. early return
      return;
  }

  if (x0 > x1) {
    SWAP(x0, x1);
    SWAP(y0, y1);
  }

  int dx = x1 - x0;
  int dy = y1 - y0;
  int sy = dy > 0 ? 1 : -1;
  int e = 0;

  dy *= sy;

  if (dx >= dy) {
      e = dy * 2 - dx;
      while (x0 != x1) {
          if (y0 >= 0 && y0 < h && x0 >= 0 && x0 < w)  spi_buffer[y0*w+x0] |= c;
          x0++;
          e += dy * 2;
          if (e >= 0) {
              e -= dx * 2;
              y0 += sy;
          }
      }
      if (y0 >= 0 && y0 < h && x0 >= 0 && x0 < w)  spi_buffer[y0*w+x0] |= c;
  } else {
      e = dx * 2 - dy;
      while (y0 != y1) {
          if (y0 >= 0 && y0 < h && x0 >= 0 && x0 < w)  spi_buffer[y0*w+x0] |= c;
          y0 += sy;
          e += dx * 2;
          if (e >= 0) {
              e -= dy * 2;
              x0++;
          }
      }
      if (y0 >= 0 && y0 < h && x0 >= 0 && x0 < w)  spi_buffer[y0*w+x0] |= c;
  }
}

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

static int
search_index_range_x(int x, uint32_t index[POINTS_COUNT], int *i0, int *i1)
{
  int i, j;
  int head = 0;
  int tail = sweep_points;
  x &= 0x03e0;
  i = 0;
  while (head < tail) {
    i = (head + tail) / 2;
    if (x == CELL_X0(index[i]))
      break;
    else if (x < CELL_X0(index[i])) {
      if (tail == i+1)
        break;
      tail = i+1;      
    } else {
      if (head == i)
        break;
      head = i;
    }
  }

  if (x != CELL_X0(index[i]))
    return FALSE;

  j = i;
  while (j > 0 && x == CELL_X0(index[j-1]))
    j--;
  *i0 = j;
  j = i;
  while (j < POINTS_COUNT-1 && x == CELL_X0(index[j+1]))
    j++;
  *i1 = j;
  return TRUE;
}

void
draw_refpos(int w, int h, int x, int y, int c)
{
  // draw triangle
  int i, j;
  if (y < -3 || y > 32 + 3)
    return;
  for (j = 0; j < 3; j++) {
    int j0 = 6 - j*2;
    for (i = 0; i < j0; i++) {
      int x0 = x + i-5;
      int y0 = y - j;
      int y1 = y + j;
      if (y0 >= 0 && y0 < h && x0 >= 0 && x0 < w)
        spi_buffer[y0*w+x0] = c;
      if (j != 0 && y1 >= 0 && y1 < h && x0 >= 0 && x0 < w)
        spi_buffer[y1*w+x0] = c;
    }
  }
}


void
cell_draw_refpos(int m, int n, int w, int h)
{
  int x0 = m * CELLWIDTH;
  int y0 = n * CELLHEIGHT;
  int t;
  for (t = 0; t < TRACES_MAX; t++) {
    if (!trace[t].enabled)
      continue;
    if (trace[t].type == TRC_SMITH || trace[t].type == TRC_POLAR)
      continue;
    int x = 0 - x0 +CELLOFFSETX;
    int y = 10*GRIDY - floatToInt((get_trace_refpos(t) * GRIDY)) - y0;
    if (x > -5 && x < w && y >= -3 && y < h+3)
      draw_refpos(w, h, x, y, config.trace_color[t]);
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

static void draw_marker(int w, int h, int x, int y, int c, int ch)
{
	int y0=y;
	for (int j=0;j<MARKER_HEIGHT;j++,y0++)
	{
		int x0=x;
		uint8_t bits = marker_bitmap[ch*10+j];
		bool force_color = false;
		while (bits){
			if (bits&0x80)
				force_color = true;
			if (x0 >= 0 && x0 < w && y0 >= 0 && y0 < h)
			{
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
  if (mode == 0)
    compare = greater;
  else
    compare = lesser;
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
    int16_t dx = x - CELL_X(index[i]) - OFFSETX;
    int16_t dy = y - CELL_Y(index[i]) - OFFSETY;
    if (dx < 0) dx = -dx;
    if (dy < 0) dy = -dy;
    if (dx > 20 || dy > 20)
      continue;
    int d = dx*dx + dy*dy;
    if (d < min_d) {
      min_i = i;
    }
  }

  return min_i;
}

void
cell_draw_markers(int m, int n, int w, int h)
{
	  int t, i;
	  for (i = 0; i < MARKERS_MAX; i++) {
	    if (!markers[i].enabled)
	      continue;
	    for (t = 0; t < TRACES_MAX; t++) {
	      if (!trace[t].enabled)
	        continue;
	      uint32_t index = trace_index[t][markers[i].index];
	      int x = CELL_X(index) - m * CELLWIDTH - X_MARKER_OFFSET;
	      int y = CELL_Y(index) - n * CELLHEIGHT - Y_MARKER_OFFSET;

	      if (x >=-MARKER_WIDTH && x < w + MARKER_WIDTH && y >= -MARKER_HEIGHT && y < h + MARKER_HEIGHT)
	        draw_marker(w, h, x, y, config.trace_color[t], i);
	    }
	  }
}

void
markmap_marker(int marker)
{
  int t;
  if (!markers[marker].enabled)
    return;
  for (t = 0; t < TRACES_MAX; t++) {
    if (!trace[t].enabled)
      continue;
    uint32_t index = trace_index[t][markers[marker].index];
    int x = CELL_X(index);
    int y = CELL_Y(index);
    int m = x>>5;
    int n = y>>5;
    mark_map(m, n);
    if ((x&31) < 6)
      mark_map(m-1, n);
    if ((x&31) > 32-6)
      mark_map(m+1, n);
    if ((y&31) < 12) {
      mark_map(m, n-1);
      if ((x&31) < 6)
        mark_map(m-1, n-1);
      if ((x&31) > 32-6)
        mark_map(m+1, n-1);
    }
  }
}

void
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


static void
draw_cell(int m, int n)
{
  int x0 = m * CELLWIDTH;
  int y0 = n * CELLHEIGHT;
  int x0off = x0 - CELLOFFSETX;
  int w = CELLWIDTH;
  int h = CELLHEIGHT;
  int x, y;
  int i0, i1;
  int i;
  int t;

  if (x0off + w > area_width)
    w = area_width - x0off;
  if (y0 + h > area_height)
    h = area_height - y0;
  if (w <= 0 || h <= 0)
    return;

  uint16_t grid_mode = 0;
  for (t = 0; t < TRACES_MAX; t++) {
    if (!trace[t].enabled)
      continue;

    if (trace[t].type == TRC_SMITH)
      grid_mode |= GRID_SMITH;
    //else if (trace[t].type == TRC_ADMIT)
    //  grid_mode |= GRID_ADMIT;
    else if (trace[t].type == TRC_POLAR)
      grid_mode |= GRID_POLAR;
    else
      grid_mode |= GRID_RECTANGULAR;
  }

  PULSE;
  memset(spi_buffer, DEFAULT_BG_COLOR, sizeof spi_buffer);
  uint16_t c = config.grid_color;
  /* draw grid */
  if (grid_mode & GRID_RECTANGULAR) {
    for (x = 0; x < w; x++) {
      if (rectangular_grid_x(x+x0off)){
    	  for (y = 0; y < h; y++)
    		  spi_buffer[y * w + x] = c;
      }
    }
    for (y = 0; y < h; y++) {
      if (rectangular_grid_y(y+y0)){
    	  for (x = 0; x < w; x++)
    		  if (x+x0off >= 0 && x+x0off < WIDTH)
    			  spi_buffer[y * w + x] = c;
      }
    }
  }
  if (grid_mode & (GRID_SMITH|GRID_ADMIT|GRID_POLAR)) {
    for (y = 0; y < h; y++) {
      for (x = 0; x < w; x++) {
        int n = 0;
        if (grid_mode & GRID_SMITH)
          n = smith_grid(x+x0off, y+y0);
        else if (grid_mode & GRID_ADMIT)
          n = smith_grid3(x+x0off, y+y0);
        //n = smith_grid2(x+x0, y+y0, 0.5);
        else if (grid_mode & GRID_POLAR)
          n = polar_grid(x+x0off, y+y0);
        if (n)
          spi_buffer[y * w + x] = c;
      }
    }
  }
  PULSE;

#if 1
  /* draw rectanglar plot */
  for (t = 0; t < TRACES_MAX; t++) {
    if (!trace[t].enabled)
      continue;
    if (trace[t].type == TRC_SMITH || trace[t].type == TRC_POLAR)
      continue;
    
    if (search_index_range_x(x0, trace_index[t], &i0, &i1)) {
      if (i0 > 0)
        i0--;
      if (i1 < POINTS_COUNT-1)
        i1++;
      for (i = i0; i < i1; i++) {
        int x1 = CELL_X(trace_index[t][i]);
        int x2 = CELL_X(trace_index[t][i+1]);
        int y1 = CELL_Y(trace_index[t][i]);
        int y2 = CELL_Y(trace_index[t][i+1]);
        int c = config.trace_color[t];
        cell_drawline(w, h, x1 - x0, y1 - y0, x2 - x0, y2 - y0, c);
      }
    }
  }
#endif
#if 1
  /* draw polar plot */
  for (t = 0; t < TRACES_MAX; t++) {
    int c = config.trace_color[t];
    if (!trace[t].enabled)
      continue;
    if (trace[t].type != TRC_SMITH && trace[t].type != TRC_POLAR)
      continue;

    for (i = 1; i < sweep_points; i++) {
      //uint32_t index = trace_index[t][i];
      //uint32_t pindex = trace_index[t][i-1];
      //if (!CELL_P(index, x0, y0) && !CELL_P(pindex, x0, y0))
      //  continue;
      int x1 = CELL_X(trace_index[t][i-1]);
      int x2 = CELL_X(trace_index[t][i]);
      int y1 = CELL_Y(trace_index[t][i-1]);
      int y2 = CELL_Y(trace_index[t][i]);
      cell_drawline(w, h, x1 - x0, y1 - y0, x2 - x0, y2 - y0, c);
    }
  }
#endif

  PULSE;
  //draw marker symbols on each trace
  cell_draw_markers(m, n, w, h);
  // draw trace and marker info on the top
  cell_draw_marker_info(m, n, w, h);
  PULSE;

  if (m == 0)
    cell_draw_refpos(m, n, w, h);

  ili9341_bulk(OFFSETX + x0off, OFFSETY + y0, w, h);
}

void
draw_all_cells(bool flush_markmap)
{
  int m, n;
  for (m = 0; m < (area_width+CELLWIDTH-1) / CELLWIDTH; m++)
    for (n = 0; n < (area_height+CELLHEIGHT-1) / CELLHEIGHT; n++) {
      if (is_mapmarked(m, n))
        draw_cell(m, n);
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
    markmap[current_mappage][0] = 0xffff;

  draw_all_cells(TRUE);
}

void
request_to_draw_cells_behind_menu(void)
{
  int n, m;
  for (m = 7; m <= 9; m++)
    for (n = 0; n < 8; n++)
      mark_map(m, n);
  redraw_request |= REDRAW_CELLS;
}

void
request_to_draw_cells_behind_numeric_input(void)
{
  int n, m;
  for (m = 0; m <= 9; m++)
    for (n = 6; n < 8; n++)
      mark_map(m, n);
  redraw_request |= REDRAW_CELLS;
}


int
cell_drawchar(int w, int h, uint8_t ch, int x, int y, int invert)
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
	    if (invert)
	      bits = ~bits;
	    for (r = 0; r < ch_size; r++) {
	      if ((x+r) >= 0 && (x+r) < w && (0x80 & bits))
	        spi_buffer[(y+c)*w + (x+r)] = foreground_color;
	      bits <<= 1;
	    }
	  }
	  return ch_size;
}

void
cell_drawstring(int w, int h, char *str, int x, int y)
{
  while (*str) {
    x += cell_drawchar(w, h, *str, x, y, FALSE);
    str++;
  }
}

void
cell_drawstring_invert(int w, int h, char *str, int x, int y, int invert)
{
  while (*str) {
    x += cell_drawchar(w, h, *str, x, y, invert);
    str++;
  }
}

static void
cell_draw_marker_info(int m, int n, int w, int h)
{
  char buf[24];
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
      xpos -= m * CELLWIDTH -CELLOFFSETX;
      ypos -= n * CELLHEIGHT;
	  
	  setForegroundColor(config.trace_color[t]);
	  if (mk == active_marker)
    	cell_drawstring(w, h, S_SARROW, xpos, ypos);
	  xpos += 5;
      chsnprintf(buf, sizeof buf, "M%d", mk+1);
      cell_drawstring(w, h, buf, xpos, ypos);
      
	  xpos += 13;
      //trace_get_info(t, buf, sizeof buf);
      int32_t freq = frequencies[markers[mk].index];
      if (uistat.marker_delta && mk != active_marker) {
        freq -= frequencies[markers[active_marker].index];
        frequency_string_short(buf, sizeof buf, freq, S_DELTA[0]);
      } else {
        frequency_string_short(buf, sizeof buf, freq, 0);
      }
      cell_drawstring(w, h, buf, xpos, ypos);
      xpos += 64;
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
      int xpos = 185;
      int ypos = 1 + (j/2)*8;
      xpos -= m * CELLWIDTH -CELLOFFSETX;
      ypos -= n * CELLHEIGHT;
      strcpy(buf, S_DELTA "1:");
      buf[1] += previous_marker;
      setForegroundColor(DEFAULT_FG_COLOR);
      cell_drawstring(w, h, buf, xpos, ypos);
      xpos += 19;
      if ((domain_mode & DOMAIN_MODE) == DOMAIN_FREQ) {
    	uint32_t delta = frequencies[idx] > frequencies[idx0] ? frequencies[idx]-frequencies[idx0] : frequencies[idx0]-frequencies[idx];
        frequency_string(buf, sizeof buf, delta, "");
      } else {
        //chsnprintf(buf, sizeof buf, "%d ns %.1f m", (uint16_t)(time_of_index(idx) * 1e9 - time_of_index(idx0) * 1e9),
        //                                            distance_of_index(idx) - distance_of_index(idx0));
        int n = string_value_with_prefix(buf, sizeof buf, time_of_index(idx) - time_of_index(idx0), 's');
        buf[n++] = ' ';
        string_value_with_prefix(&buf[n], sizeof buf - n, distance_of_index(idx) - distance_of_index(idx0), 'm');
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
//      setForegroundColor(config.trace_color[t]);
//      strcpy(buf, "CH0");
//      buf[2] += trace[t].channel;
      //chsnprintf(buf, sizeof buf, "CH%d", trace[t].channel);
//      cell_drawstring_invert(w, h, buf, xpos, ypos, t == uistat.current_trace);
//      xpos += 20;
      setForegroundColor(config.trace_color[t]);
      if (t == uistat.current_trace)
		cell_drawstring(w, h, S_SARROW, xpos, ypos);
      xpos += 5;
      chsnprintf(buf, sizeof buf, "CH%d", trace[t].channel);
      cell_drawstring(w, h, buf, xpos, ypos);
      xpos += 19;

      trace_get_info(t, buf, sizeof buf);
      cell_drawstring(w, h, buf, xpos, ypos);
      xpos += 60;
      trace_get_value_string(t, buf, sizeof buf, measured[trace[t].channel], idx);
      cell_drawstring(w, h, buf, xpos, ypos);
      j++;
    }

    // draw marker frequency
    int xpos = 185;
    int ypos = 1 + (j/2)*8;
    xpos -= m * CELLWIDTH -CELLOFFSETX;
    ypos -= n * CELLHEIGHT;
    setForegroundColor(DEFAULT_FG_COLOR);
//    strcpy(buf, "1:");
//    buf[0] += active_marker;
//    xpos += 5;
//    setForegroundColor(0xffff);
//    cell_drawstring_invert(w, h, buf, xpos, ypos, uistat.lever_mode == LM_MARKER);
//	xpos += 14;
    if (uistat.lever_mode == LM_MARKER)
    	cell_drawstring(w, h, S_SARROW, xpos, ypos);
    xpos += 5;
    chsnprintf(buf, sizeof buf, "M%d:", active_marker+1);
    cell_drawstring(w, h, buf, xpos, ypos);
	xpos += 19;

    if ((domain_mode & DOMAIN_MODE) == DOMAIN_FREQ) {
      frequency_string(buf, sizeof buf, frequencies[idx], "");
    } else {
      //chsnprintf(buf, sizeof buf, "%d ns %.1f m", (uint16_t)(time_of_index(idx) * 1e9), distance_of_index(idx));
      int n = string_value_with_prefix(buf, sizeof buf, time_of_index(idx), 's');
      buf[n++] = ' ';
      string_value_with_prefix(&buf[n], sizeof buf-n, distance_of_index(idx), 'm');
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
    chsnprintf(buf, sizeof buf, "Edelay");
    cell_drawstring(w, h, buf, xpos, ypos);
    xpos += 7 * 5;
    int n = string_value_with_prefix(buf, sizeof buf, electrical_delay * 1e-12, 's');
    cell_drawstring(w, h, buf, xpos, ypos);
    xpos += n * 5 + 5;
    float light_speed_ps = 299792458e-12; //(m/ps)
    string_value_with_prefix(buf, sizeof buf, electrical_delay * light_speed_ps * velocity_factor, 'm');
    cell_drawstring(w, h, buf, xpos, ypos);
  }
}

void
frequency_string(char *buf, size_t len, uint32_t freq, char *prefix)
{
/*  if (freq < 0) {
    freq = -freq;
    *buf++ = '-';
    len -= 1;
  }*/
  if (freq < 1000) {
    chsnprintf(buf, len, "%s%d Hz", prefix, (int)freq);
  } else if (freq < 1000000U) {
    chsnprintf(buf, len, "%s%d.%03d kHz", prefix,
             (freq / 1000U),
             (freq % 1000U));
  } else {
    chsnprintf(buf, len, "%s%d.%03d %03d MHz", prefix,
             (freq / 1000000U),
             ((freq / 1000U) % 1000U),
             (freq % 1000U));
  }
}

void
frequency_string_short(char *b, size_t len, int32_t freq, char prefix)
{
  char *buf = b;
  if (prefix) {
    *buf++ = prefix;
    len -= 1;
  }
  if (freq < 0) {
    freq = -freq;
    *buf++ = '-';
    len -= 1;
  }
  if (freq < 1000) {
    chsnprintf(buf, len, "%d Hz", (int)freq);
  } else if (freq < 1000000) {
    chsnprintf(buf, len, "%d.%03dkHz",
             (int)(freq / 1000),
             (int)(freq % 1000));
  } else {
    chsnprintf(buf, len, "%d.%06d",
             (int)(freq / 1000000),
             (int)(freq % 1000000));
    strcpy(b+9, "MHz");
  }
}

void
draw_frequencies(void)
{
  char buf1[24];buf1[0]=' ';
  char buf2[24];buf2[0]=0;
  if ((domain_mode & DOMAIN_MODE) == DOMAIN_FREQ) {
      if (frequency0 < frequency1) {
        frequency_string(buf1+1, sizeof(buf1)-1, frequency0, "START ");
        frequency_string(buf2, sizeof buf2, frequency1, "STOP ");
      } else if (frequency0 > frequency1) {
        frequency_string(buf1+1, sizeof(buf1)-1, frequency0/2 + frequency1/2, "CENTER ");
        frequency_string(buf2, sizeof buf2, frequency0 - frequency1, "SPAN ");
      } else {
        frequency_string(buf1+1, sizeof(buf1)-1, frequency0,  "CW ");
      }
  } else {
	  chsnprintf(buf1+1, sizeof(buf1)-1, "START 0s");
      chsnprintf(buf2, sizeof buf2, "%s%dns (%.2fm)", "STOP ", (uint16_t)(time_of_index(POINTS_COUNT-1) * 1e9), distance_of_index(POINTS_COUNT-1));
  }
  setForegroundColor(DEFAULT_FG_COLOR);
  setBackgroundColor(DEFAULT_BG_COLOR);
  ili9341_fill(0, 232, 320, 8, DEFAULT_BG_COLOR);
  if (uistat.lever_mode == LM_SPAN || uistat.lever_mode == LM_CENTER)
	buf1[0] = S_SARROW[0];
  ili9341_drawstring(buf1, OFFSETX, 232);
  ili9341_drawstring(buf2, 205, 232);
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
#define BATTERY_TOP_LEVEL		4100
#define BATTERY_BOTTOM_LEVEL	3100
#define BATTERY_WARNING_LEVEL	3300

void
draw_battery_status(void)
{
    uint8_t string_buf[25];
	// Set battery color
	setForegroundColor(vbat < BATTERY_WARNING_LEVEL ? DEFAULT_LOW_BAT_COLOR : DEFAULT_NORMAL_BAT_COLOR);
    setBackgroundColor(DEFAULT_BG_COLOR);
//    chsnprintf(string_buf, sizeof string_buf, "V:%d", vbat);
//    ili9341_drawstringV(string_buf, 1, 60);
    // Prepare battery bitmap image
    // Battery top
    int x=0;
    string_buf[x++] = 0b00111100;
    string_buf[x++] = 0b00100100;
    string_buf[x++] = 0b11111111;
//    string_buf[x++] = 0b10000001;
	// Fill battery status
	for (int power=BATTERY_TOP_LEVEL; power > BATTERY_BOTTOM_LEVEL; power-=100)
		string_buf[x++] = (power > vbat) ? 0b10000001 : // Empty line
				                           0b11111111;  // Full line
	// Battery bottom
//	string_buf[x++] = 0b10000001;
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
