/*
 * sizetype.hh -- define type representing errors and size bounds
 *
 * Copyright (C) 2001-2009 Norbert Mueller
 * Copyright     2014-2016 Franz Brausse
 *
 * This file is part of the iRRAM Library.
 *
 * The iRRAM Library is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Library General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * The iRRAM Library is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Library General Public
 * License for more details.
 *
 * You should have received a copy of the GNU Library General Public License
 * along with the iRRAM Library; see the file COPYING.LIB.  If not, write to
 * the Free Software Foundation, Inc., 59 Temple Place - Suite 330, Boston,
 * MA 02111-1307, USA.
 */

#ifndef SIZETYPE_HH
#define SIZETYPE_HH

#include <climits>
#include <cmath>
#include <cassert>
#include <limits>
#include <cfloat>

#include <iRRAM/core.h>
#include <iRRAM/STREAMS.h> /* iRRAM_DEBUG* */

namespace iRRAM {

/*! \addtogroup sizetype
 * @{
 */

static_assert(CHAR_BIT == 8, "iRRAM only checked for platforms of 8-bit bytes");

const int MANTISSA_BITS = (int)(CHAR_BIT * sizeof(sizetype::mantissa_t));
const int DIFF_BITS     = 3;
const int BIT_RANGE     = 8;
const int BIT_RANGE2    = 8;
const int GUARD_BITS    = MANTISSA_BITS - DIFF_BITS;

const typename sizetype::mantissa_t max_mantissa = 1 <<  GUARD_BITS   ;
const typename sizetype::exponent_t min_mantissa = 1 << (GUARD_BITS-BIT_RANGE);

/* if the exponent of value is smaller than MP_min, it should be increased(!) to (1,min_exponent) */
const typename sizetype::exponent_t min_exponent = MP_min + MANTISSA_BITS;

inline unsigned int scale(const unsigned int w,const int p) {return ((p<=GUARD_BITS)?(w>>p):0); }

/*!
 * \brief Try to keep the mantissa between \ref max_mantissa and \ref min_mantissa.
 *
 * It may not be larger than \ref max_mantissa.
 * It still may stay smaller than \ref min_mantissa afterwards.
 * The value of \a e may slighty increase during this operation!
 * The value of \a e does never decrease!
 * If the resulting exponent is out of range (\ref MP_min,\ref MP_max)
 * correct it with an increase in value or reiterate.
 *
 * \param [in,out] e
 *
 * \sa iRRAM_REITERATE
 * \exception Iteration when the resulting exponent is >= \ref MP_max
 */
inline void sizetype_normalize( sizetype& e) {
  if (iRRAM_unlikely(e.mantissa < min_mantissa)) {
      e.mantissa <<= BIT_RANGE;
      e.exponent -= BIT_RANGE;
  }
  if (iRRAM_unlikely( e.mantissa >=  max_mantissa ) ) {
      e.mantissa = ( e.mantissa>> DIFF_BITS ) + 1;
      e.exponent += DIFF_BITS;
  }
  if (iRRAM_unlikely( e.exponent < MP_min ) ){
    e.exponent = min_exponent;
  }
  if (iRRAM_unlikely( e.exponent >= MP_max ) )
  {
    iRRAM_DEBUG1(1,"exponent too big in sizetype_normalize ");
    iRRAM_REITERATE(0);
  }
}

/*!
 * \brief Try to keep the mantissa between \ref max_mantissa and \ref min_mantissa.
 * \sa sizetype_normalize(sizetype &)
 * \param [in] e value to normalize
 * \return       \a e, normalized
 * \sa iRRAM_REITERATE
 * \exception Iteration when the resulting exponent is >= \ref MP_max
 */
inline sizetype sizetype_normalize(sizetype &&e)
{
	sizetype_normalize(e);
	return e;
}

inline sizetype sizetype_power2(typename sizetype::exponent_t exp)
{
	return sizetype_normalize({1, exp});
}

/*!
 * \brief Increments \a x by \a y and normalizes \a x.
 *
 * Arguments \a x and \a y must differ!
 * The resulting value may be a bit larger than the exact value,
 * The resulting value will never be smaller than the exact value.
 *
 * \param [out] x
 * \param [in]  y
 */
inline void sizetype_inc(sizetype& x,const sizetype& y)
{
  if ( y.exponent >  x.exponent) {
  x.mantissa=scale(x.mantissa,(y.exponent-x.exponent))
            +y.mantissa +1 ;
  x.exponent=y.exponent;
  } else {
  x.mantissa +=
            scale(y.mantissa,(x.exponent-y.exponent)) +1 ;
  }
  sizetype_normalize(x);
}

/*!
 * \brief Increments \a x by \a y and normalizes \a x.
 *
 * Arguments \a x and \a y must differ!
 * The resulting value may be a bit larger than the exact value,
 * The resulting value will never be smaller than the exact value.
 *
 * \param [in,out] x
 * \param [in]     y
 */
inline sizetype & operator+=(sizetype &x, const sizetype &y)
{
	sizetype_inc(x, y);
	return x;
}

/*!
 * \brief Increments \a x by \a y + \a z and normalizes \a x.
 *
 * Arguments \a x, \a y and \a z must differ!
 * The resulting value may be a bit larger than the exact value,
 * The resulting value will never be smaller than the exact value.
 *
 * \param [in,out] x
 * \param [in]     y, z
 */
inline void sizetype_inc2(sizetype& x,const sizetype& y,const sizetype& z)
{ typename sizetype::exponent_t exponent;
  exponent=max({x.exponent,y.exponent,z.exponent});
  x.mantissa=scale(x.mantissa,(exponent-x.exponent))
            +scale(y.mantissa,(exponent-y.exponent))
            +scale(z.mantissa,(exponent-z.exponent)) +2 ;
  x.exponent=exponent;
  sizetype_normalize(x);
}


/*!
 * \brief Adds \a y and \a z yielding \a x; no normalization.
 *
 * Argument \a x must be different from \a y and \a z!
 * The resulting value may be a bit larger than the exact value,
 * The resulting value will never be smaller than the exact value.
 *
 * \param [out] x
 * \param [in]  y, z
 */
inline void sizetype_add_wo_norm(sizetype& x,const sizetype& y,const sizetype& z)
{
  if ( y.exponent >  z.exponent) {
    x.exponent= y.exponent;
    x.mantissa= y.mantissa + scale(z.mantissa,(x.exponent-z.exponent))  + 1 ;
  } else {
    x.exponent= z.exponent;
    x.mantissa= z.mantissa + scale(y.mantissa,(x.exponent-y.exponent))  + 1 ;
  }
}


/*!
 * \brief Adds \a y and \a z yielding \a x and normalizes \a x.
 *
 * Argument \a x must be different from \a y and \a z!
 * The resulting value may be a bit larger than the exact value,
 * The resulting value will never be smaller than the exact value.
 *
 * \param [out] x
 * \param [in]  y, z
 */
inline void sizetype_add(sizetype& x,const sizetype& y,const sizetype& z)
{
  sizetype_add_wo_norm(x,y,z);
  sizetype_normalize(x);
}

/*!
 * \brief Computes \a x+2^\a exp and returns the normalized result.
 *
 * The resulting value may be a bit larger than the exact value,
 * it will never be smaller than the exact value.
 *
 * \param [in] x, exp
 * \return \a x+2^\a exp, normalized
 * \sa iRRAM_REITERATE
 * \exception Iteration when the result is too large to be represented by a
 *                      normalized sizetype
 */
inline sizetype sizetype_add_power2(sizetype x,
                                    typename sizetype::exponent_t exp)
{
	/* leave as much of the original mantissa intact as possible */
	if (exp < x.exponent + GUARD_BITS) {
		x.mantissa += 1U << max(0, exp - x.exponent);
		/* x.exponent remains unchanged */
	} else {
		x.mantissa  = scale(x.mantissa, exp-(GUARD_BITS-1)-x.exponent)+1
		            + (1U << (GUARD_BITS-1));
		x.exponent  = exp-(GUARD_BITS-1);
	}
	sizetype_normalize(x);
	return x;
}

/*!
 * \brief Adds \a y and \a z yielding \a x and normalizes the result.
 *
 * The resulting value may be a bit larger than the exact value,
 * The resulting value will never be smaller than the exact value.
 *
 * \param [in] y, z
 * \return \a y+\a z, normalized
 */
inline sizetype operator+(const sizetype& y,const sizetype& z)
{
  sizetype x;
  sizetype_add(x,y,z);
  return x;
}


/*!
 * \brief Shift \a y by \a s yielding \a x.
 *
 * Arguments x,y may be identical.
 * The resulting value may be a bit larger than the exact value,
 * The resulting value will never be smaller than the exact value.
 *
 * \param [out] x
 * \param [in] y, s
 * \sa iRRAM_REITERATE
 * \exception Iteration when the result is too large to be represented by a
 *                      normalized sizetype
 */
inline void sizetype_shift(sizetype& x, const sizetype& y, int s)
{
  x.exponent=y.exponent+s;
  x.mantissa=y.mantissa;

  if (iRRAM_unlikely( x.exponent < MP_min ) ) {
    iRRAM_DEBUG1(2,"warning: small exponent found in sizetype_shift\n");
    x.exponent = min_exponent;
//    x.mantissa = 1;
  } else  if ( iRRAM_unlikely(x.exponent >= MP_max) )
  {
    iRRAM_DEBUG1(1,"exponent too big in sizetype_shift ");
    iRRAM_REITERATE(0);
  }
}

/*!
 * \brief Returns \a y left-shifted by \a s, normalized.
 *
 * The resulting value may be a bit larger than the exact value,
 * The resulting value will never be smaller than the exact value.
 *
 * \param [in] y, s
 * \return \a y left-shifted by \a s, normalized.
 * \sa iRRAM_REITERATE
 * \exception Iteration when the result is too large to be represented by a
 *                      normalized sizetype
 */
inline sizetype operator<<(const sizetype &y, int s)
{
	sizetype x;
	sizetype_shift(x, y, s);
	return x;
}

/*!
 * \brief Multiplies \a y and \a z yielding \a x and normalizes \a x.
 *
 * Argument \a x must be different from \a y and \a z!
 * The resulting value may be a bit larger than the exact value,
 * The resulting value will never be smaller than the exact value.
 *
 * \param [out] x
 * \param [in]  y, z
 */
inline void sizetype_mult(sizetype& x,const sizetype& y,const sizetype& z)
{ unsigned long long lmantissa=
     ((unsigned long long)(y.mantissa))*z.mantissa;
  x.exponent=y.exponent+z.exponent;

  while (iRRAM_unlikely(lmantissa >= max_mantissa) )
       { lmantissa=lmantissa>>BIT_RANGE2;x.exponent+=BIT_RANGE2;}

  x.mantissa=lmantissa+1;
  sizetype_normalize(x);
}

/*!
 * \brief Sets \a y to \a y * \a z, normalized.
 *
 * The resulting value may be a bit larger than the exact value,
 * The resulting value will never be smaller than the exact value.
 *
 * \param [in] y, z
 * \return \a y * \a z, normalized.
 */
inline sizetype operator*(const sizetype &y, const sizetype &z)
{
	sizetype x;
	sizetype_mult(x, y, z);
	return x;
}

/*!
 * \brief Computes the maximum of \a y and \a z and stores it in \a x.
 *
 * Arguments \a x, \a y and \a z may all be identical.
 * The resulting value is exactly the maximum.
 *
 * \param [out] x
 * \param [in]  y, z
 */
inline void sizetype_max(sizetype& x,const sizetype& y,const sizetype& z)
{ if (y.exponent>z.exponent)
  {
    if (scale(z.mantissa,y.exponent-z.exponent)>=y.mantissa) x=z; else x=y;
  } else {
    if (scale(y.mantissa,z.exponent-y.exponent)>=z.mantissa) x=y; else x=z;
  }
}

/*!
 * \brief Returns the maximum of \a y and \a z.
 *
 * Arguments \a y and \a z may be identical.
 * The resulting value is exactly the maximum.
 *
 * \param [in] y, z
 * \return The maximum of \a y and \a z.
 */
inline sizetype max(const sizetype &y, const sizetype &z)
{
	if (y.exponent > z.exponent)
		return scale(z.mantissa, y.exponent-z.exponent) >= y.mantissa ? z : y;
	else
		return scale(y.mantissa, z.exponent-y.exponent) >= z.mantissa ? y : z;
}

/*!
 * \brief Construct a sizetype value \a x from \a mantissa and \a exponent.
 *
 * The resulting value is allowed to be larger than the exact value.
 *
 * \param [out] x \a mantissa * 2^\a exponent, normalized
 * \param [in] mantissa, exponent
 */
inline void sizetype_set(sizetype& x, typename sizetype::mantissa_t mantissa, typename sizetype::exponent_t exponent)
{
  x.exponent=exponent;
  x.mantissa=mantissa;
  sizetype_normalize(x);
}

inline void sizetype_exact(sizetype& x)
{
  x.exponent=min_exponent;
  x.mantissa=0;
}

inline constexpr sizetype sizetype_exact() { return { 0, min_exponent }; }

/*!
 * \brief Tests whether y<z. If y=z the result is allowed to be true OR false!
 *
 * \param [in] y, z
 * \return \c true if \a y<\a z or \a y=\a z; or
 *         \c false if \a y>\a z or \a y=\a z
 */
inline bool sizetype_less(const sizetype& y,const sizetype& z)
{ unsigned int mantissa;
  if (iRRAM_unlikely(y.mantissa==0)) return true;
  if (iRRAM_unlikely(z.mantissa==0)) return false;
  if (y.exponent>z.exponent)
  {
    mantissa=scale(z.mantissa,y.exponent-z.exponent);
    return (mantissa>=y.mantissa);
  } else {
    mantissa=scale(y.mantissa,z.exponent-y.exponent);
    return (mantissa<z.mantissa);
  }
}

inline void sizetype_half(sizetype& x,const sizetype& y)
{
  x.exponent=y.exponent-1;
  x.mantissa=y.mantissa;
}

/*!
 * \brief Computes the integer approximation to the base-2 logarithm rounded
 *        towards positive infinity.
 * \param [in] x normalized value
 * \return \f$\lceil\log_2(x)\rceil\f$ for \f$x\neq0\f$;
 *         \ref min_exponent otherwise
 */
inline int sizetype_log2(const sizetype &x)
{
	int r;
	typename sizetype::mantissa_t v = x.mantissa;
	if (!v)
		return min_exponent;
	r  = CHAR_BIT * sizeof(v) - clz(v) - 1; /* floor(log2(v)) */
	r += (v & (v-1)) != 0;                  /* ceil(log2(v)) */
	r += x.exponent;
	return r;
}

/************************ the following functions are unchecked for over/underflow ***************/
/* also the exact semantics has still to be defined and compared to the applications ************/

/*!
 * \warning unchecked for over-/underflow
 * \todo exact semantics are not specified
 */
inline void sizetype_dec(sizetype& x, const sizetype& y )
{ x.mantissa=x.mantissa - scale(y.mantissa,(x.exponent-y.exponent)) -1 ;
  sizetype_normalize(x);
}

/*!
 * \warning unchecked for over-/underflow
 * \todo exact semantics are not specified
 */
inline void sizetype_dec(sizetype& x)
{ x.mantissa -= 1 ;
}

/*!
 * \brief Computes the square root of \a y and stores it, rounded towards
 *        positive infinity, in \a x.
 *
 * The result is never smaller than the exact value.
 *
 * \param [out] x non-normalized (over-approximation of) square root of \a y
 * \param [in]  y normalized sizetype value
 * \warning unchecked for over-/underflow
 */
inline void sizetype_sqrt(sizetype& x,const sizetype& y)
{
  static_assert(FLT_RADIX == 2 && MANTISSA_BITS <= DBL_MANT_DIG,
                "sizetype_sqrt() only proven for MANTISSA_BITS <= DBL_MANT_DIG");
  if (y.exponent&1) {
    x.exponent=(y.exponent-1)/2;
    x.mantissa=y.mantissa <<1;
  }  else {
    x.exponent=y.exponent/2;
    x.mantissa=y.mantissa;
  }
  /*! \todo
   * Cast to integral type depends on correct *rounding* of the *exact* result
   * of `sqrt(double)` provided by stdlib. Can we be sure of that?
   *
   * If so: The argument to sqrt() is exact (DBL_MANT_DIG >= MANTISSA_BITS), the
   * result will be the closest double <= precise result. This value rounded
   * upwards is returned. A wrong approximation would require the precise result
   * to be k+eps and sqrt() to return k-eps', however since the double type does
   * represent all 32-bit integers (like mantissa_t) exactly, the
   * closest double <= precise result would then always be (double)k (since
   * k >= 0), for any integer k and 0 < eps' <= eps < 1ulp. */
  x.mantissa=(typename sizetype::mantissa_t)std::sqrt(double(x.mantissa))+1;
}

/*!
 * \warning unchecked for over-/underflow
 * \todo exact semantics are not specified
 */
inline void sizetype_div(sizetype& x,const sizetype& y,const sizetype& z)
{ unsigned long long lmantissa=
         (((unsigned long long)(y.mantissa))<<GUARD_BITS)/z.mantissa;
  x.exponent=y.exponent-z.exponent-GUARD_BITS;

  while ( lmantissa >= max_mantissa )
       { lmantissa=lmantissa>>BIT_RANGE2;x.exponent+=BIT_RANGE2;}

  x.mantissa=lmantissa+1;
  sizetype_normalize(x);
}

//! @}

}

namespace std {

/*! \addtogroup sizetype */
template <typename M,typename E>
struct numeric_limits<::iRRAM::generic_sizetype<M,E>> {
	static constexpr bool is_specialized     = true;
	static constexpr bool is_signed          = std::numeric_limits<M>::is_signed;
	static constexpr bool is_integer         = false;
	static constexpr bool is_exact           = false;
	static constexpr bool has_infinity       = false;
	static constexpr bool has_quiet_NaN      = false;
	static constexpr bool has_signalling_NaN = false;
	static constexpr std::float_denorm_style has_denorm = std::denorm_present;
	static constexpr bool has_denorm_loss    = true; /*!< \todo is this correct? */
	static constexpr std::float_round_style round_style = std::round_toward_infinity;
	static constexpr bool is_iec559          = false;
	static constexpr bool is_bounded         = true; /*!< \todo should be false for verification */
	static constexpr bool is_modulo          = false;
	static constexpr int  digits             = std::numeric_limits<M>::digits - ::iRRAM::DIFF_BITS;
//	static constexpr int  digits10           = digits * std::log10(2);
//	static constexpr int  max_digits10       = digits10 + 2; /*!< \todo is this correct? */
	static constexpr int  radix              = 2;
	static constexpr int  min_exponent       = MP_min + std::numeric_limits<M>::digits;
//	static constexpr int  min_exponent10     = ;
	static constexpr int  max_exponent       = MP_max - std::numeric_limits<M>::digits;
//	static constexpr int  max_exponent10     = ;
	static constexpr bool traps              = false;
	static constexpr bool tinyness_before    = true; /*!< \todo is this correct? */

private:
	using T = ::iRRAM::generic_sizetype<M,E>;

public:
	static constexpr T min() { return { (M)1 << (digits - iRRAM::BIT_RANGE), min_exponent }; }
	static constexpr T lowest() { return { 0, min_exponent }; }
	static constexpr T max() { return { (M)1 << digits, max_exponent }; }
	static constexpr T epsilon() { return { (M)1 << (digits - iRRAM::BIT_RANGE), -(digits - iRRAM::BIT_RANGE) }; }
	// static constexpr T round_error() {}
	static constexpr T inifinity() { return min(); }
	static constexpr T quiet_NaN() { return min(); }
	static constexpr T signaling_NaN() { return min(); }
	static constexpr T denorm_min() { return { 0, min_exponent }; }
};

}

static_assert(std::numeric_limits<iRRAM::sizetype>::min_exponent == iRRAM::min_exponent, "internal error");


#endif
