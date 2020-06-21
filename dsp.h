/*
 * Copyright (c) 2019-2020, Dmitry (DiSlord) dislordlive@gmail.com
 * All rights reserved.
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * The software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNU Radio; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

// Cortex M4 DSP instructions assembly

// __smlabb inserts a SMLABB instruction. __smlabb returns the equivalent of
//  int32_t res = x[0] * y[0] + acc
//  where [0] is the lower 16 bits and [1] is the upper 16 bits. This operation sets the Q flag if overflow occurs on the addition.
__attribute__((always_inline)) __STATIC_INLINE int32_t __smlabb(int32_t x, int32_t y, int32_t acc)
{
  register int32_t r;
  __ASM volatile ("smlabb %[r], %[x], %[y], %[a]"
   : [r] "=r" (r) : [x] "r" (x), [y] "r" (y), [a] "r" (acc) : );
  return r;
}

// __smlabt inserts a SMLABT instruction. __smlabt returns the equivalent of
//  int32_t res = x[0] * y[1] + acc
//  where [0] is the lower 16 bits and [1] is the upper 16 bits. This operation sets the Q flag if overflow occurs on the addition.
__attribute__((always_inline)) __STATIC_INLINE int32_t __smlabt(int32_t x, int32_t y, int32_t acc)
{
  register int32_t r;
  __ASM volatile ("smlabt %[r], %[x], %[y], %[a]"
   : [r] "=r" (r) : [x] "r" (x), [y] "r" (y), [a] "r" (acc) : );
  return r;
}

// __smlatb inserts a SMLATB instruction. __smlatb returns the equivalent of
//  int32_t res = x[1] * y[0] + acc
//  where [0] is the lower 16 bits and [1] is the upper 16 bits. This operation sets the Q flag if overflow occurs on the addition.
__attribute__((always_inline)) __STATIC_INLINE int32_t __smlatb(int32_t x, int32_t y, int32_t acc)
{
  register int32_t r;
  __ASM volatile ("smlatb %[r], %[x], %[y], %[a]"
   : [r] "=r" (r) : [x] "r" (x), [y] "r" (y), [a] "r" (acc) : );
  return r;
}

// __smlatt inserts a SMLATT instruction. __smlatt returns the equivalent of
//  int32_t res = x[1] * y[1] + acc
//  where [0] is the lower 16 bits and [1] is the upper 16 bits. This operation sets the Q flag if overflow occurs on the addition.
__attribute__((always_inline)) __STATIC_INLINE int32_t __smlatt(int32_t x, int32_t y, int32_t acc)
{
  register int32_t r;
  __ASM volatile ("smlatt %[r], %[x], %[y], %[a]"
   : [r] "=r" (r) : [x] "r" (x), [y] "r" (y), [a] "r" (acc) : );
  return r;
}

// __smlalbb inserts a SMLALBB instruction. __smlalbb returns the equivalent of
//  int64_t res = x[0] * y[0] + acc
//  where [0] is the lower 16 bits and [1] is the upper 16 bits.
__attribute__((always_inline)) __STATIC_INLINE int64_t __smlalbb(int64_t acc, int32_t x, int32_t y)
{
  register union {
    struct { uint32_t lo; uint32_t hi; } s_rep;
    int64_t i_rep;
  } r;
  r.i_rep = acc;
  __ASM volatile ("smlalbb %[r_lo], %[r_hi], %[x], %[y]"
   : [r_lo] "+r" (r.s_rep.lo), [r_hi] "+r" (r.s_rep.hi)
   : [x] "r" (x), [y] "r" (y) : );
  return r.i_rep;
}

// __smlalbt inserts a SMLALBT instruction. __smlalbt returns the equivalent of
//  int64_t res = x[0] * y[1] + acc
//  where [0] is the lower 16 bits and [1] is the upper 16 bits.
__attribute__((always_inline)) __STATIC_INLINE int64_t __smlalbt(int64_t acc, int32_t x, int32_t y)
{
  register union {
    struct { uint32_t lo; uint32_t hi; } s_rep;
    int64_t i_rep;
  } r;
  r.i_rep = acc;
  __ASM volatile ("smlalbt %[r_lo], %[r_hi], %[x], %[y]"
   : [r_lo] "+r" (r.s_rep.lo), [r_hi] "+r" (r.s_rep.hi)
   : [x] "r" (x), [y] "r" (y) : );
  return r.i_rep;
}

// __smlaltb inserts a SMLALTB instruction. __smlaltb returns the equivalent of
//  int64_t res = x[1] * y[0] + acc
//  where [0] is the lower 16 bits and [1] is the upper 16 bits.
__attribute__((always_inline)) __STATIC_INLINE int64_t __smlaltb(int64_t acc, int32_t x, int32_t y)
{
  register union {
    struct { uint32_t lo; uint32_t hi; } s_rep;
    int64_t i_rep;
  } r;
  r.i_rep = acc;
  __ASM volatile ("smlaltb %[r_lo], %[r_hi], %[x], %[y]"
   : [r_lo] "+r" (r.s_rep.lo), [r_hi] "+r" (r.s_rep.hi)
   : [x] "r" (x), [y] "r" (y) : );
  return r.i_rep;
}

// __smlaltt inserts a SMLALTT instruction. __smlaltt returns the equivalent of
//  int64_t res = x[1] * y[1] + acc
//  where [0] is the lower 16 bits and [1] is the upper 16 bits.
static inline int64_t __smlaltt(int64_t acc, int32_t x, int32_t y)
{
  register union {
    struct { uint32_t lo; uint32_t hi; } s_rep;
    int64_t i_rep;
  } r;
  r.i_rep = acc;
  __ASM volatile ("smlaltt %[r_lo], %[r_hi], %[x], %[y]"
    : [r_lo] "+r" (r.s_rep.lo), [r_hi] "+r" (r.s_rep.hi)
    : [x] "r" (x), [y] "r" (y) : );
  return r.i_rep;
}
