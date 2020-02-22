/*
    ChibiOS - Copyright (C) 2006..2016 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/*
   Concepts and parts of this file have been contributed by Fabio Utzig,
   chvprintf() added by Brent Roman.
 */

/**
 * @file    chprintf.c
 * @brief   Mini printf-like functionality.
 *
 * @addtogroup chprintf
 * @{
 */

#include "hal.h"
#include "chprintf.h"
#include "memstreams.h"
#include <math.h>

// Enable [flags], support:
// ' ' Prepends a space for positive signed-numeric types. positive = ' ', negative = '-'. This flag is ignored if the + flag exists.
//#define CHPRINTF_USE_SPACE_FLAG

#define MAX_FILLER 11
#define FLOAT_PRECISION 9

#pragma pack(push, 2)

static const uint32_t pow10[FLOAT_PRECISION+1] = {
    1, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000, 1000000000
};
// Prefixes for values bigger then 1000.0
//                            1  1e3, 1e6, 1e9, 1e12, 1e15, 1e18, 1e21, 1e24
static char bigPrefix[]  = {' ', 'k', 'M', 'G',  'T',  'P',  'E',  'Z',  'Y', 0};
// Prefixes for values less   then 1.0
//                          1e-3, 1e-6, 1e-9, 1e-12, 1e-15, 1e-18, 1e-21, 1e-24
static char smallPrefix[]= { 'm', 0x1d,  'n',   'p',   'f',   'a',   'z',   'y', 0};

#pragma pack(pop)

//86066	   5116	  11456	 102622	  190de	build/ch.elf
static char *long_to_string_with_divisor(char *p,
                                         uint32_t num,
										 uint32_t radix,
										 uint32_t precision) {
  char *q = p + MAX_FILLER;
  char *b = q;
  // convert to string from end buffer to begin
  do {
    uint8_t c = num % radix;
	num /= radix;
    *--q = c + ((c > 9) ? ('A'-10) : '0');
  }while((precision && --precision) || num);
  // copy string at begin
  int i = (int)(b - q);
  do
    *p++ = *q++;
  while (--i);
  return p;
}

// default prescision = 13
// g.mmm kkk hhh
#define MAX_FREQ_PRESCISION	13
#define FREQ_PSET			1
#define FREQ_NO_SPACE		2
#define FREQ_PREFIX_SPACE	4

static char *ulong_freq(char *p, uint32_t freq, uint32_t precision){
  uint8_t flag = FREQ_PSET;
  if (precision == 0)
    flag|=FREQ_PREFIX_SPACE;
  if (precision == 0 || precision > MAX_FREQ_PRESCISION)
    precision = MAX_FREQ_PRESCISION;
  char *q = p + MAX_FREQ_PRESCISION;
  char *b = q;
  // Prefix counter
  uint32_t s = 0;
  // Set format (every 3 digits add ' ' up to GHz)
  uint32_t format=0b00100100100;
  do {
#if 0
    uint8_t c = freq % 10;
    freq/= 10;
#else
    // Fast and compact division uint32_t on 10, using shifts, result:
    // c = freq % 10
    // freq = freq / 10;
    uint32_t c = freq;
    freq>>=1;
	freq+=freq>>1;
    freq+=freq>>4;
    freq+=freq>>8;
    freq+=freq>>16; // freq = 858993459*freq/1073741824 = freq * 0,799999999813735485076904296875
    freq>>=3;       // freq/=8; freq = freq * 0,09999999997671693563461303710938
    c-= freq*10;    // freq*10 = (freq*4+freq)*2 = ((freq<<2)+freq)<<1
    while (c>=10) {freq++;c-=10;}
#endif
	*--q = c + '0';
	if (freq==0)
		break;
	// Add spaces, calculate prefix
	if (format&1) {*--q = ' '; s++;}
	format>>=1;
  } while (1);
  s = bigPrefix[s];

  // Get string size
  uint32_t i = (b - q);
  // Limit string size, max size is - precision
  if (precision && i > precision) {
	  i = precision;
	  flag|=FREQ_NO_SPACE;
  }
  // copy string
  // Replace first ' ' by '.', remove ' ' if size too big
  do{
    char c = *q++;
    // replace first ' ' on '.'
    if (c == ' ') {
      if (flag&FREQ_PSET){
        c = '.';
        flag&=~FREQ_PSET;
      }
      else if (flag&FREQ_NO_SPACE)
        c = *q++;
    }
    *p++ = c;
  }while (--i);
  // Put pref (amd space before it if need)
  if (flag&FREQ_PREFIX_SPACE && s!=' ')
	  *p++ = ' ';
  *p++ = s;
  return p;
}

#if CHPRINTF_USE_FLOAT
static char *ftoa(char *p, float num, uint32_t precision) {
 // Check precision limit
  if (precision > FLOAT_PRECISION)
    precision = FLOAT_PRECISION;
  uint32_t multi = pow10[precision];
  uint32_t l = num;
  // Round value
  uint32_t k = ((num-l)*multi+0.5);
  // Fix rounding error if get
  if (k>=multi){k-=multi;l++;}
  p = long_to_string_with_divisor(p, l, 10, 0);
  if (precision && k){
	 *p++ = '.';
	p=long_to_string_with_divisor(p, k, 10, precision);
	// remove zeros at end
    while (p[-1]=='0') p--;
    if (p[-1]=='.') p--;
  }
  return p;
}

static char *ftoaS(char *p, float num, uint32_t precision) {
  char prefix=0;
  char *ptr;
  if (num > 1000.0){
	  for (ptr = bigPrefix+1; *ptr && num > 1000.0; num/=1000, ptr++)
		  ;
	  prefix = ptr[-1];
  }
  else if (num < 1){
	  for (ptr = smallPrefix; *ptr && num < 1.0; num*=1000, ptr++)
		  ;
	  prefix = num > 1e-3 ? ptr[-1] : 0;
  }
  // Auto set prescision
  uint32_t l = num;
  if (l < 10)
    precision+=2;
  else if (l < 100)
    precision+=1;
  p=ftoa(p, num, precision);
  if (prefix)
    *p++ = prefix;
  return p;
}
#endif

/**
 * @brief   System formatted output function.
 * @details This function implements a minimal @p vprintf()-like functionality
 *          with output on a @p BaseSequentialStream.
 *          The general parameters format is: %[-][width|*][.precision|*][l|L]p.
 *          The following parameter types (p) are supported:
 *          - <b>x</b> hexadecimal integer.
 *          - <b>X</b> hexadecimal long.
 *          - <b>o</b> octal integer.
 *          - <b>O</b> octal long.
 *          - <b>d</b> decimal signed integer.
 *          - <b>D</b> decimal signed long.
 *          - <b>u</b> decimal unsigned integer.
 *          - <b>U</b> decimal unsigned long.
 *          - <b>c</b> character.
 *          - <b>s</b> string.
 *          .
 *
 * @param[in] chp       pointer to a @p BaseSequentialStream implementing object
 * @param[in] fmt       formatting string
 * @param[in] ap        list of parameters
 * @return              The number of bytes that would have been
 *                      written to @p chp if no stream error occurs
 *
 * @api
 */
#define IS_LONG				1
#define LEFT_ALIGN			2
#define POSITIVE			4
#define NEGATIVE			8
#define PAD_ZERO			16
#define PLUS_SPACE			32
#define DEFAULT_PRESCISION	64

int chvprintf(BaseSequentialStream *chp, const char *fmt, va_list ap) {
  char *p, *s, c, filler=' ';
  int precision, width;
  int n = 0;
  uint32_t	state;
  union {
	  uint32_t u;
	  int32_t  l;
	  float    f;
  }value;
#if CHPRINTF_USE_FLOAT
  char tmpbuf[2*MAX_FILLER + 1];
#else
  char tmpbuf[MAX_FREQ_PRESCISION + 1];
#endif

  while (true) {
    c = *fmt++;
    if (c == 0)
      return n;
    if (c != '%') {
      streamPut(chp, (uint8_t)c);
      n++;
      continue;
    }
    // Parse %[flags][width][.precision][length]type
    p = tmpbuf;
    s = tmpbuf;
    state = 0;
    width = 0;
    precision = 0;

    // Get [flags], support:
    // '-' Left-align the output of this placeholder. (The default is to right-align the output.)
    // '+' Prepends a  plus for positive signed-numeric types. positive = '+', negative = '-'.
    // ' ' Prepends a space for positive signed-numeric types. positive = ' ', negative = '-'. This flag is ignored if the + flag exists.
    // '0' When the 'width' option is specified, prepends zeros for numeric types. (The default prepends spaces.)
    while (true){
      if (*fmt == '-')
        state|=LEFT_ALIGN;
      else if (*fmt == '+')
        state|=POSITIVE;
      else if (*fmt == '0')
    	state|=PAD_ZERO;
#ifdef CHPRINTF_USE_SPACE_FLAG
      else if (*fmt == ' ')
    	state|=PLUS_SPACE;
#endif
      else
    	  break;
      fmt++;
    }
    // Get [width] - The Width field specifies a minimum number of characters to output
    // if set *, get width as argument
    while (true) {
      c = *fmt++;
      if (c >= '0' && c <= '9')
        c -= '0';
      else if (c == '*')
        c = va_arg(ap, int);
      else
        break;
      width = width * 10 + c;
    }
    // Get [.precision]
    if (c == '.') {
      while (true) {
        c = *fmt++;
        if (c >= '0' && c <= '9')
          c -= '0';
        else if (c == '*')
          c = va_arg(ap, int);
        else
           break;
        precision = precision * 10 + c;
      }
    }
    else
		state|=DEFAULT_PRESCISION;
    //Get [length]
    /*
    if (c == 'l' || c == 'L') {
      state|=IS_LONG;
      if (*fmt)
        c = *fmt++;
    }
    else if((c >= 'A') && (c <= 'Z'))
    	state|=IS_LONG;
    */
    // Parse type
    switch (c) {
    case 'c':
      state&=~PAD_ZERO;
      *p++ = va_arg(ap, int);
      break;
    case 's':
      state&=~PAD_ZERO;
      if ((s = va_arg(ap, char *)) == 0)
        s = "(null)";
      if (state&DEFAULT_PRESCISION)
        precision = 32767;
      for (p = s; *p && (--precision >= 0); p++)
        ;
      break;
    case 'D':
    case 'd':
    case 'I':
    case 'i':/*
      if (state & IS_LONG)
        value.l = va_arg(ap, long);
      else*/
    	value.l = va_arg(ap, uint32_t);
      if (value.l < 0) {
    	state|=NEGATIVE;
        *p++ = '-';
        value.l = -value.l;
      }
      else if (state & POSITIVE)
    	*p++ = '+';
#ifdef CHPRINTF_USE_SPACE_FLAG
      else if (state & PLUS_SPACE)
    	*p++ = ' ';
#endif
      p = long_to_string_with_divisor(p, value.l, 10, 0);
      break;
	case 'q':
		value.u = va_arg(ap, uint32_t);
		p=ulong_freq(p, value.u, precision);
	  break;
#if CHPRINTF_USE_FLOAT
    case 'F':
    case 'f':
      value.f = va_arg(ap, double);
      if (value.f < 0) {
    	state|=NEGATIVE;
        *p++ = '-';
        value.f = -value.f;
      }
      else if (state & POSITIVE)
    	*p++ = '+';
#ifdef CHPRINTF_USE_SPACE_FLAG
      else if (state & PLUS_SPACE)
       	*p++ = ' ';
#endif
      if (value.f == INFINITY){
    	  *p++ = 0x19;
    	  break;
      }
      p = (c=='F') ? ftoaS(p, value.f, precision) : ftoa(p, value.f, state&DEFAULT_PRESCISION ? FLOAT_PRECISION : precision);
      break;
#endif
    case 'X':
    case 'x':
      c = 16;
      goto unsigned_common;
    case 'U':
    case 'u':
      c = 10;
      goto unsigned_common;
    case 'O':
    case 'o':
      c = 8;
unsigned_common:/*
      if (state & IS_LONG)
        value.u = va_arg(ap, unsigned long);
      else*/
        value.u = va_arg(ap, uint32_t);
      p = long_to_string_with_divisor(p, value.u, c, 0);
      break;
    default:
      *p++ = c;
      break;
    }
    // Now need print buffer s[{sign}XXXXXXXXXXXX]p and align it on width
    // Check filler width (if buffer less then width) and prepare filler if need fill
    if ((width -=(int)(p - s)) < 0)
      width = 0;
    else
      filler = (state&PAD_ZERO) ? '0' : ' ';
    // if left align, put sign digit, and fill
    // [{sign}ffffffXXXXXXXXXXXX]
    if (!(state&LEFT_ALIGN)) {
      // Put '+' or '-' or ' ' first if need
      if ((state&(NEGATIVE|POSITIVE|PLUS_SPACE)) && (state&PAD_ZERO)) {
        streamPut(chp, (uint8_t)*s++);
        n++;
      }
      // fill from left
      while (width){
        streamPut(chp, (uint8_t)filler);
        n++;
		width--;
      }
    }
    // put data
    while (s < p) {
      streamPut(chp, (uint8_t)*s++);
      n++;
    }
    // Put filler from right (if need)
    while (width) {
      streamPut(chp, (uint8_t)filler);
      n++;
      width--;
    }
  }
}

/**
 * @brief   System formatted output function.
 * @details This function implements a minimal @p printf() like functionality
 *          with output on a @p BaseSequentialStream.
 *          The general parameters format is: %[-][width|*][.precision|*][l|L]p.
 *          The following parameter types (p) are supported:
 *          - <b>x</b> hexadecimal integer.
 *          - <b>X</b> hexadecimal long.
 *          - <b>o</b> octal integer.
 *          - <b>O</b> octal long.
 *          - <b>d</b> decimal signed integer.
 *          - <b>D</b> decimal signed long.
 *          - <b>u</b> decimal unsigned integer.
 *          - <b>U</b> decimal unsigned long.
 *          - <b>c</b> character.
 *          - <b>s</b> string.
 *          .
 *
 * @param[in] chp       pointer to a @p BaseSequentialStream implementing object
 * @param[in] fmt       formatting string
 *
 * @api
 */
int chprintf(BaseSequentialStream *chp, const char *fmt, ...) {
  va_list ap;
  int formatted_bytes;

  va_start(ap, fmt);
  formatted_bytes = chvprintf(chp, fmt, ap);
  va_end(ap);

  return formatted_bytes;
}

/**
 * @brief   System formatted output function.
 * @details This function implements a minimal @p vprintf()-like functionality
 *          with output on a @p BaseSequentialStream.
 *          The general parameters format is: %[-][width|*][.precision|*][l|L]p.
 *          The following parameter types (p) are supported:
 *          - <b>x</b> hexadecimal integer.
 *          - <b>X</b> hexadecimal long.
 *          - <b>o</b> octal integer.
 *          - <b>O</b> octal long.
 *          - <b>d</b> decimal signed integer.
 *          - <b>D</b> decimal signed long.
 *          - <b>u</b> decimal unsigned integer.
 *          - <b>U</b> decimal unsigned long.
 *          - <b>c</b> character.
 *          - <b>s</b> string.
 *          .
 * @post    @p str is NUL-terminated, unless @p size is 0.
 *
 * @param[in] str       pointer to a buffer
 * @param[in] size      maximum size of the buffer
 * @param[in] fmt       formatting string
 * @return              The number of characters (excluding the
 *                      terminating NUL byte) that would have been
 *                      stored in @p str if there was room.
 *
 * @api
 */
int chsnprintf(char *str, size_t size, const char *fmt, ...) {
  va_list ap;
  MemoryStream ms;
  BaseSequentialStream *chp;
  size_t size_wo_nul;
  int retval;

  if (size > 0)
    size_wo_nul = size - 1;
  else
    size_wo_nul = 0;

  /* Memory stream object to be used as a string writer, reserving one
     byte for the final zero.*/
  msObjectInit(&ms, (uint8_t *)str, size_wo_nul, 0);

  /* Performing the print operation using the common code.*/
  chp = (BaseSequentialStream *)(void *)&ms;
  va_start(ap, fmt);
  retval = chvprintf(chp, fmt, ap);
  va_end(ap);

  /* Terminate with a zero, unless size==0.*/
  if (ms.eos < size)
      str[ms.eos] = 0;

  /* Return number of bytes that would have been written.*/
  return retval;
}

/** @} */
