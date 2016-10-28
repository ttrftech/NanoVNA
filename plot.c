#include <math.h>
#include <string.h>
#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "nanovna.h"

void cell_draw_marker_info(int m, int n, int w, int h);
void draw_frequencies(void);
static inline void force_set_markmap(void);

#define SWAP(x,y) do { int z=x; x = y; y = z; } while(0)

int32_t fstart = 0;
int32_t fstop = 300000000;
int32_t fspan = 300000000;
int32_t fgrid = 50000000;
int grid_offset;
int grid_width;

#define OFFSETX 15
#define OFFSETY 0
#define WIDTH 291
#define HEIGHT 233

//#define GRID_COLOR 0x0863
uint16_t grid_color = 0x1084;

void set_sweep(int32_t start, int stop)
{
  int32_t gdigit = 100000000;
  int32_t grid;
  fstart = start;
  fstop = stop;
  fspan = stop - start;

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
  draw_frequencies();
}

int
circle_inout(int x, int y, int r)
{
  int d = x*x + y*y - r*r;
  if (d <= -r)
    return 1;
  if (d > r)
    return -1;
  return 0;
}

int
smith_grid(int x, int y)
{
  int d = circle_inout(x-146, y-116, 116);
  int c = grid_color;
  if (d < 0)
    return 0;
  else if (d == 0)
    return c;
  x -= 146+116;
  y -= 116;
  
  if (circle_inout(x, y+58, 58) == 0)
    return c;
  if (circle_inout(x, y-58, 58) == 0)
    return c;
  d = circle_inout(x+29, y, 29);
  if (d > 0) return 0;
  if (d == 0) return c;
  if (circle_inout(x, y+116, 116) == 0)
    return c;
  if (circle_inout(x, y-116, 116) == 0)
    return c;
  d = circle_inout(x+58, y, 58);
  if (d > 0) return 0;
  if (d == 0) return c;
  if (circle_inout(x, y+232, 232) == 0)
    return c;
  if (circle_inout(x, y-232, 232) == 0)
    return c;
  if (circle_inout(x+87, y, 87) == 0)
    return c;
  return 0;
}

int
rectangular_grid(int x, int y)
{
#define FREQ(x) (((x) * (fspan / 1000) / (WIDTH-1)) * 1000 + fstart)
  int c = grid_color;
  //int32_t n = FREQ(x-1) / fgrid;
  //int32_t m = FREQ(x) / fgrid;
  //if ((m - n) > 0)
  //if (((x * 6) % (WIDTH-1)) < 6)
  //if (((x - grid_offset) % grid_width) == 0)
  if ((((x + grid_offset) * 10) % grid_width) < 10)
    return c;
  if (x == 0 || x == (WIDTH-1))
    return c;
  if ((y % 29) == 0)
    return c;
  return 0;
}

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

trace_t trace[TRACES_MAX] = {
  { 1, TRC_LOGMAG, 0, 1.0, RGB565(0,255,255), 0 },
  { 1, TRC_LOGMAG, 1, 1.0, RGB565(255,0,40), 0 },
  { 1, TRC_SMITH, 0, 1.0, RGB565(0,0,255), 1 },
  { 1, TRC_PHASE, 1, 1.0, RGB565(50,255,0), 1 }
};

uint32_t trace_index[TRACES_MAX][101];

float logmag(float *v)
{
  return log10f(v[0]*v[0] + v[1]*v[1]);
}

float phase(float *v)
{
  return 2 * atan2f(v[1], v[0]) / M_PI;
}

float linear(float *v)
{
  return - sqrtf(v[0]*v[0] + v[1]*v[1]) * 8;
}

float swr(float *v)
{
  float x = sqrtf(v[0]*v[0] + v[1]*v[1]);
  return (1 + x)/(1 - x);
}


#define RADIUS ((HEIGHT-1)/2)
void
cartesian_scale(float re, float im, int *xp, int *yp, float scale)
{
  //float scale = 4e-3;
  int x = re * RADIUS * scale;
  int y = im * RADIUS * scale;
  if (x < -RADIUS) x = -RADIUS;
  if (y < -RADIUS) y = -RADIUS;
  if (x > RADIUS) x = RADIUS;
  if (y > RADIUS) y = RADIUS;
  *xp = WIDTH/2 + x;
  *yp = HEIGHT/2 - y;
}

#define INDEX(x, y, n) \
  ((((x)&0x03e0UL)<<22) | (((y)&0x03e0UL)<<17) | (((n)&0x0fffUL)<<10)  \
 | (((x)&0x1fUL)<<5) | ((y)&0x1fUL))
#define CELL_X(i) (int)((((i)>>5)&0x1f) | (((i)>>22)&0x03e0))
#define CELL_Y(i) (int)(((i)&0x1f) | (((i)>>17)&0x03e0))
#define CELL_N(i) (int)(((i)>>10)&0xfff)

#define CELL_X0(i) (int)(((i)>>22)&0x03e0)
#define CELL_Y0(i) (int)(((i)>>17)&0x03e0)

#define CELL_P(i, x, y) (((((x)&0x03e0UL)<<22) | (((y)&0x03e0UL)<<17)) == ((i)&0xffc00000UL))

#define CELLWIDTH 32
#define CELLHEIGHT 32


inline void swap(uint32_t *a, uint32_t *b)
{
  uint32_t t=*a; *a=*b; *b=t;
}

void insertionsort(uint32_t *arr, int start, int end)
{
  int i;
  for (i = start + 1; i < end; i++) {
    uint32_t val = arr[i];
    int j = i - 1;
    while (j >= start && val > arr[j]) {
      arr[j + 1] = arr[j];
      j--;
    }
    arr[j + 1] = val;
  }
}

void quicksort(uint32_t *arr, int beg, int end)
{
  if (end - beg <= 1) 
    return;
  else if (end - beg < 10) {
    insertionsort(arr, beg, end);
  } else {
    int l = beg;
    int r = end-1;
    uint32_t piv = arr[(beg + end) / 2];
    while (l < r) {
      while (arr[l] < piv)
        l++;
      while (arr[r] > piv)
        r--;
      if (l < r)
        swap(&arr[l], &arr[r]);
    }

    quicksort(arr, beg, l);
    quicksort(arr, r, end);
  }
}


uint16_t markmap[2][8];
uint16_t current_mappage = 0;

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

static void
swap_markmap(void)
{
  current_mappage = 1 - current_mappage;
}

static inline void
clear_markmap(void)
{
  memset(markmap[current_mappage], 0, sizeof markmap[current_mappage]);
}

static inline void
force_set_markmap(void)
{
  memset(markmap[current_mappage], 0xff, sizeof markmap[current_mappage]);
}

uint32_t
trace_into_index(int x, int t, int i, float coeff[2])
{
  int y = 0;
  float v = 0;
  switch (trace[t].type) {
  case TRC_LOGMAG:
    v = 1 - logmag(coeff);
    break;
  case TRC_PHASE:
    v = 4 + phase(coeff);
    break;
  case TRC_LINEAR:
    v = 8 + linear(coeff);
    break;
  case TRC_SWR:
    v = 9 - swr(coeff);
    break;
  case TRC_SMITH:
  case TRC_ADMIT:
  case TRC_POLAR:
    cartesian_scale(coeff[0], coeff[1], &x, &y, trace[t].scale);
    return INDEX(x, y, i);
    break;
  }
  if (v < 0) v = 0;
  if (v > 8) v = 8;
  y = v * 29;
  return INDEX(x, y, i);
}

void
trace_get_value_string(int t, char *buf, int len, float coeff[2])
{
  float v;
  switch (trace[t].type) {
  case TRC_LOGMAG:
    v = logmag(coeff);
    chsnprintf(buf, len, "%.2f dB", v * 10);
    break;
  case TRC_PHASE:
    v = phase(coeff);
    chsnprintf(buf, len, "%.2f deg", v * 90);
    break;
  case TRC_LINEAR:
    v = linear(coeff);
    chsnprintf(buf, len, "%.2f", v);
    break;
  case TRC_SWR:
    v = swr(coeff);
    chsnprintf(buf, len, "%.2f", v);
    break;
  case TRC_SMITH:
    chsnprintf(buf, len, "%.2f %.2fj", coeff[0], coeff[1]);
    break;
  }
}

void plot_into_index(float measured[2][101][2])
{
  int i, t;
  for (i = 0; i < 101; i++) {
    int x = i * (WIDTH-1) / (101-1);
    for (t = 0; t < TRACES_MAX; t++) {
      if (!trace[t].enabled)
        continue;
      int n = trace[t].channel;
      trace_index[t][i] = trace_into_index(x, t, i, measured[n][i]);
    }
  }
#if 0
  for (t = 0; t < TRACES_MAX; t++)
    if (trace[t].enabled && trace[t].polar)
      quicksort(trace_index[t], 0, 101);
#endif

  /* mark cells between each neighber points */
  for (t = 0; t < TRACES_MAX; t++) {
    int x0 = CELL_X(trace_index[t][0]);
    int y0 = CELL_Y(trace_index[t][0]);
    int m0 = x0 >> 5;
    int n0 = y0 >> 5;
    mark_map(m0, n0);
    for (i = 1; i < 101; i++) {
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

void
line_in_cell(int w, int h, int x0, int y0, int x1, int y1, int c)
{
  if (x0 > x1) {
    SWAP(x0, x1);
    SWAP(y0, y1);
  }

  while (x0 <= x1) {
    int dx = x1 - x0 + 1;
    int dy = y1 - y0;
    if (dy >= 0) {
      dy++;
      if (dy > dx) {
        dy /= dx; dx = 1;
      } else {
        dx /= dy; dy = 1;
      }
    } else {
      dy--;
      if (-dy > dx) {
        dy /= dx; dx = 1;
      } else {
        dx /= -dy; dy = -1;
      }
    }

    if (dx == 1) {
      if (dy > 0) {
        while (dy-- > 0) {
          if (y0 >= 0 && y0 < h && x0 >= 0 && x0 < w)
            spi_buffer[y0*w+x0] |= c;
          y0++;
        }
      } else {
        while (dy++ < 0) {
          if (y0 >= 0 && y0 < h && x0 >= 0 && x0 < w)
            spi_buffer[y0*w+x0] |= c;
          y0--;
        }
      }
      x0++;
    } else {
      while (dx-- > 0) {
        if (y0 >= 0 && y0 < h && x0 >= 0 && x0 < w)
          spi_buffer[y0*w+x0] |= c;
        x0++;
      }
      y0 += dy;
    }
  }
}

int
search_index(int x, int y, uint32_t index[101], int *i0, int *i1)
{
  int i, j;
  int head = 0;
  int tail = 101;
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
  while (j < 100 && x == CELL_X0(index[j+1]) && y == CELL_Y0(index[j+1]))
    j++;
  *i1 = j;
  return TRUE;
}

int
search_index_x(int x, uint32_t index[101], int *i0, int *i1)
{
  int i, j;
  int head = 0;
  int tail = 101;
  x &= 0x03e0;
  while (head < tail) {
    i = (head + tail) / 2;
    if (x < CELL_X0(index[i]))
      tail = i+1;
    else if (x > CELL_X0(index[i]))
      head = i;
    else
      break;
  }

  if (x != CELL_X0(index[i]))
    return FALSE;

  j = i;
  while (j > 0 && x == CELL_X0(index[j-1]))
    j--;
  *i0 = j;
  j = i;
  while (j < 100 && x == CELL_X0(index[j+1]))
    j++;
  *i1 = j;
  return TRUE;
}

void
draw_marker(int w, int h, int x, int y, int c, int ch)
{
  int i, j;
  for (j = 10; j >= 0; j--) {
    int j0 = j / 2;
    for (i = -j0; i <= j0; i++) {
      int x0 = x + i;
      int y0 = y - j;
      int cc = c;
      if (j <= 9 && j > 2 && i >= -1 && i <= 3) {
        uint16_t bits = x5x7_bits[(ch * 7) + (9-j)];
        if (bits & (0x8000>>(i+1)))
          cc = 0;
      }
      if (y0 >= 0 && y0 < h && x0 >= 0 && x0 < w)
        spi_buffer[y0*w+x0] = cc;
    }
  }
}

void
draw_cell(int m, int n)
{
  int x0 = m * CELLWIDTH;
  int y0 = n * CELLHEIGHT;
  int w = CELLWIDTH;
  int h = CELLHEIGHT;
  int x, y;
  int i0, i1;
  int i;
  int t;
  if (x0 + w > WIDTH)
    w = WIDTH - x0;
  if (y0 + h > HEIGHT)
    h = HEIGHT - y0;

  /* draw grid */
  for (y = 0; y < h; y++) {
    for (x = 0; x < w; x++) {
      uint16_t c = rectangular_grid(x+x0, y+y0);
      c |= smith_grid(x+x0, y+y0);
      spi_buffer[y * w + x] = c;
    }
  }

#if 1
  /* draw rectanglar plot */
  if (search_index_x(x0, trace_index[0], &i0, &i1)) {
    for (t = 0; t < TRACES_MAX; t++) {
      if (!trace[t].enabled || trace[t].polar)
        continue;
      if (i0 > 0)
        i0--;
      for (i = i0; i < i1; i++) {
        int x1 = CELL_X(trace_index[t][i]);
        int x2 = CELL_X(trace_index[t][i+1]);
        int y1 = CELL_Y(trace_index[t][i]);
        int y2 = CELL_Y(trace_index[t][i+1]);
        int c = trace[t].color;
        line_in_cell(w, h, x1 - x0, y1 - y0, x2 - x0, y2 - y0, c);
      }
    }
  }
#endif
#if 1
  /* draw polar plot */
  for (t = 0; t < TRACES_MAX; t++) {
    int c = trace[t].color;
    if (!trace[t].enabled || !trace[t].polar)
      continue;
    for (i = 1; i < 101; i++) {
      //uint32_t index = trace_index[t][i];
      //uint32_t pindex = trace_index[t][i-1];
      //if (!CELL_P(index, x0, y0) && !CELL_P(pindex, x0, y0))
      //  continue;
      int x1 = CELL_X(trace_index[t][i-1]);
      int x2 = CELL_X(trace_index[t][i]);
      int y1 = CELL_Y(trace_index[t][i-1]);
      int y2 = CELL_Y(trace_index[t][i]);
      line_in_cell(w, h, x1 - x0, y1 - y0, x2 - x0, y2 - y0, c);
    }
  }
#endif
#if 0
  /* draw polar plot */
  for (t = 0; t < TRACES_MAX; t++) {
    int prev = -100;
    int c = trace[t].color;
    if (!trace[t].enabled || !trace[t].polar)
      continue;
    if (search_index(x0, y0, trace_index[t], &i0, &i1)) {
      for (i = i0; i < i1; i++) {
        uint32_t index = trace_index[t][i];
        uint32_t pindex;
        int n = i;
        if (!CELL_P(index, x0, y0))
          continue;
        n = CELL_N(index);
        if (n - prev == 1) {
          pindex = trace_index[t][prev];
          line_in_cell(w, h, CELL_X(pindex) - x0, CELL_Y(pindex) - y0, CELL_X(index) - x0, CELL_Y(index) - y0, c);
          }
          prev = n;
      }
    }
  }
#endif

#if 0
  if (m == 0 && n == 0) {
    draw_marker(w, h, 8, 12, trace[0].color, '1');
    draw_marker(w, h, 18, 20, trace[1].color, '2');
    draw_marker(w, h, 4, 30, trace[2].color, '3');
  }
#endif

  i = 30;
  for (t = 0; t < TRACES_MAX; t++) {
    if (!trace[t].enabled)
      continue;
    uint32_t index = trace_index[t][i];
    int x = CELL_X(index) - x0;
    int y = CELL_Y(index) - y0;
    if (x > -12 && x < w+12 && y >= 0 && y < h+12)
      draw_marker(w, h, x, y, trace[t].color, '1');
  }

  cell_draw_marker_info(m, n, w, h);

  ili9341_bulk(OFFSETX + x0, OFFSETY + y0, w, h);
}

void
draw_cell_all(void)
{
  int m, n;
  for (m = 0; m < (WIDTH+CELLWIDTH-1) / CELLWIDTH; m++)
    for (n = 0; n < (HEIGHT+CELLHEIGHT-1) / CELLHEIGHT; n++)
      if (is_mapmarked(m, n))
        draw_cell(m, n);
  swap_markmap();
  clear_markmap();
}

void
cell_drawchar_5x7(int w, int h, uint8_t ch, int x, int y, uint16_t fg)
{
  uint16_t bits;
  int c, r;
  for(c = 0; c < 7; c++) {
    if ((y + c) < 0 || (y + c) >= h)
      continue;
    bits = x5x7_bits[(ch * 7) + c];
    for (r = 0; r < 5; r++) {
      if ((x+r) >= 0 && (x+r) < w && (0x8000 & bits)) 
        spi_buffer[(y+c)*w + (x+r)] = fg;
      bits <<= 1;
    }
  }
}

void
cell_drawstring_5x7(int w, int h, char *str, int x, int y, uint16_t fg)
{
  while (*str) {
    cell_drawchar_5x7(w, h, *str, x, y, fg);
    x += 5;
    str++;
  }
}

void
cell_draw_marker_info(int m, int n, int w, int h)
{
  char buf[24];
  int t;
  if (n != 0)
    return;
  if (m == 4 || m == 5 || m == 6) {
    int xpos = 128;
    int ypos = 1;
    xpos -= m * w;
    ypos -= n * h;
#if 0
    chsnprintf(buf, 24, "Ch0 LOGMAG 10dB/");
    cell_drawstring_5x7(w, h, buf, xpos, ypos, trace[0].color);
    chsnprintf(buf, 24, "Ch1 LogMag 10dB/");
    cell_drawstring_5x7(w, h, buf, xpos, ypos+7, trace[1].color);
    chsnprintf(buf, 24, "Ch0 SMITH   1.0/");
    cell_drawstring_5x7(w, h, buf, xpos, ypos+14, trace[2].color);
    chsnprintf(buf, 24, "Ch1 PHASE 90deg/");
    cell_drawstring_5x7(w, h, buf, xpos, ypos+21, trace[3].color);
#else
    for (t = 0; t < TRACES_MAX; t++) {
      if (!trace[t].enabled)
        continue;
      trace_get_info(t, buf, sizeof buf);
      cell_drawstring_5x7(w, h, buf, xpos, ypos, trace[t].color);
      ypos += 7;
    }
#endif
  }
#if 1
  if (m == 6 || m == 7 || m == 8) {
    int xpos = 216;
    int ypos = 1;
    xpos -= m * w;
    ypos -= n * h;
    for (t = 0; t < TRACES_MAX; t++) {
      int idx = 30;
      trace_get_value_string(t, buf, sizeof buf, measured[trace[t].channel][idx]);
      if (!trace[t].enabled)
        continue;
      cell_drawstring_5x7(w, h, buf, xpos, ypos, trace[t].color);
      ypos += 7;
    }
  }
#endif
}

void
draw_frequencies(void)
{
  char buf[24];
  chsnprintf(buf, 24, "START%3d.%03d %03d MHz",
             (int)(fstart / 1000000),
             (int)((fstart / 1000) % 1000),
             (int)(fstart % 1000));
  ili9341_drawstring_5x7(buf, OFFSETX, 233, 0xffff, 0x0000);
  chsnprintf(buf, 24, "STOP %3d.%03d %03d MHz",
             (int)(fstop / 1000000),
             (int)((fstop / 1000) % 1000),
             (int)(fstop % 1000));
  ili9341_drawstring_5x7(buf, 205, 233, 0xffff, 0x0000);
}

void
redraw(void)
{
  ili9341_fill(0, 0, 320, 240, 0);
  draw_frequencies();
}

void
plot_init(void)
{
  force_set_markmap();
}
