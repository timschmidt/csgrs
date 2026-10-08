/***************************************************************************
 *            numeric/rounding.hpp
 *
 *  Copyright  2008-20  Pieter Collins
 *
 ****************************************************************************/

/*
 *  This file is part of Ariadne.
 *
 *  Ariadne is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Ariadne is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with Ariadne.  If not, see <https://www.gnu.org/licenses/>.
 */

/*! \file numeric/rounding.hpp
 *  \brief Functions to set and retrieve the processor rounding mode.
 *  May be platform-dependent.
 */

#ifndef ARIADNE_ROUNDING_HPP
#define ARIADNE_ROUNDING_HPP

#include <iosfwd>

#if defined __GNUC__ && ( defined __i386__ || defined __x86_64 || defined _M_IX86 || defined _M_X86 || defined __arm__ || defined __aarch64__ )
    #if ( defined __SSE_MATH__ &&  defined __SSE2__ )
        #define ARIADNE_SSE_ROUNDING
    #elif ( defined __i386__ || defined __x86_64 || defined _M_IX86 || defined _M_X86) && (__GNUC__ >= 5 || ( __GNUC__ == 4 && __GNUC_MINOR__ >= 3 ))
        #define ARIADNE_GCC_ROUNDING
    #else
        #define ARIADNE_C99_ROUNDING
    #endif
#endif


//#undef ARIADNE_SSE_ROUNDING
//#undef ARIADNE_GCC_ROUNDING
//#undef ARIADNE_C99_ROUNDING

//#define ARIADNE_GCC_ROUNDING


#if defined ARIADNE_SSE_ROUNDING

#include <cstdint>
#include <xmmintrin.h>

namespace Ariadne {

typedef __uint16_t rounding_mode_t;

const rounding_mode_t ROUND_TO_NEAREST  = _MM_ROUND_NEAREST;
const rounding_mode_t ROUND_DOWNWARD    = _MM_ROUND_DOWN;
const rounding_mode_t ROUND_UPWARD      = _MM_ROUND_UP;
const rounding_mode_t ROUND_TOWARD_ZERO = _MM_ROUND_TOWARD_ZERO;

inline void set_builtin_rounding_to_nearest() { _MM_SET_ROUNDING_MODE(_MM_ROUND_NEAREST);  }
inline void set_builtin_rounding_downward() { _MM_SET_ROUNDING_MODE(_MM_ROUND_DOWN);  }
inline void set_builtin_rounding_upward() { _MM_SET_ROUNDING_MODE(_MM_ROUND_UP);  }
inline void set_builtin_rounding_toward_zero() { _MM_SET_ROUNDING_MODE(_MM_ROUND_TOWARD_ZERO);  }

inline void set_builtin_rounding_mode(rounding_mode_t rnd) { _MM_SET_ROUNDING_MODE(rnd); }
inline rounding_mode_t get_builtin_rounding_mode() { return _MM_GET_ROUNDING_MODE(); }

enum class RoundingMode : rounding_mode_t {
    TO_NEAREST = ROUND_TO_NEAREST, DOWNWARD = ROUND_DOWNWARD, UPWARD = ROUND_UPWARD, TOWARD_ZERO = ROUND_TOWARD_ZERO
};

} // namespace Ariadne



#elif defined ARIADNE_C99_ROUNDING

#include <fenv.h>

namespace Ariadne {

typedef int rounding_mode_t;

const rounding_mode_t ROUND_TO_NEAREST  = FE_TONEAREST;
const rounding_mode_t ROUND_DOWNWARD    = FE_DOWNWARD;
const rounding_mode_t ROUND_UPWARD      = FE_UPWARD;
const rounding_mode_t ROUND_TOWARD_ZERO = FE_TOWARDZERO;

inline void set_builtin_rounding_to_nearest() { fesetround(FE_TONEAREST);  }
inline void set_builtin_rounding_downward() { fesetround(FE_DOWNWARD);  }
inline void set_builtin_rounding_upward() { fesetround(FE_UPWARD);  }
inline void set_builtin_rounding_toward_zero() { fesetround(FE_TOWARDZERO);  }

inline void set_builtin_rounding_mode(rounding_mode_t rnd) { fesetround(rnd); }
inline rounding_mode_t get_builtin_rounding_mode() { return fegetround(); }

} // namespace Ariadne



#elif defined ARIADNE_GCC_ROUNDING

namespace Ariadne {

typedef unsigned short rounding_mode_t;

const rounding_mode_t ROUND_TO_NEAREST  = 895;
const rounding_mode_t ROUND_DOWNWARD    = 895+1024;
const rounding_mode_t ROUND_UPWARD      = 895+2048;
const rounding_mode_t ROUND_TOWARD_ZERO = 895+3072;

inline void set_builtin_rounding_mode(rounding_mode_t rnd) { asm volatile ("fldcw %0" : : "m" (rnd) ); }
inline void set_builtin_rounding_to_nearest() { asm volatile ("fldcw %0" : : "m" (ROUND_TO_NEAREST) ); }
inline void set_builtin_rounding_downward() { asm volatile ("fldcw %0" : : "m" (ROUND_DOWNWARD) ); }
inline void set_builtin_rounding_upward() { asm volatile ("fldcw %0" : : "m" (ROUND_UPWARD) ); }
inline void set_builtin_rounding_toward_zero() { asm volatile ("fldcw %0" : : "m" (ROUND_TOWARD_ZERO) ); }

inline rounding_mode_t get_builtin_rounding_mode() { rounding_mode_t rnd; asm volatile ("fstcw %0" : "=m" (rnd) ); return rnd; }

} // namespace Ariadne



#elif defined ARIADNE_MSVC_ROUNDING

static const unsigned short ARIADNE_FENV_BASE = 895;
static unsigned short ARIADNE_ROUND_TMP = ARIADNE_FENV_BASE;

namespace Ariadne {

typedef unsigned short rounding_mode_t;

const rounding_mode_t ROUND_TO_NEAREST   = ARIADNE_FENV_BASE;
const rounding_mode_t ROUND_DOWNWARD     = ARIADNE_FENV_BASE + 1024;
const rounding_mode_t ROUND_UPWARD       = ARIADNE_FENV_BASE + 2048;
const rounding_mode_t ROUND_TOWARD_ZERO  = ARIADNE_FENV_BASE + 3072;

inline void set_builtin_rounding_to_nearest() { __asm fldcw to_nearest; }
inline void set_builtin_rounding_downward() { __asm fldcw downward; }
inline void set_builtin_rounding_upward() { __asm fldcw upward; }
inline void set_builtin_rounding_toward_zero() { __asm fldcw toward_zero; }

inline void set_builtin_rounding_mode(rounding_mode_t rnd) { ARIADNE_RND_TMP=rnd; __asm fldcw ARIADNE_ROUND_TMP; }
inline rounding_mode_t get_builtin_rounding_mode() { __asm fstcw ARIADNE_ROUND_TMP; ARIADNE_RND_TMP; }

} // namespace Ariadne



#else // No rounding

namespace Ariadne {

typedef unsigned short rounding_mode_t;

const rounding_mode_t ROUND_TO_NEAREST   = 0000;
const rounding_mode_t ROUND_DOWNWARD     = 1024;
const rounding_mode_t ROUND_UPWARD       = 2048;
const rounding_mode_t ROUND_TOWARD_ZERO  = 3072;

inline void set_builtin_rounding_to_nearest() { }
inline void set_builtin_rounding_downward() { }
inline void set_builtin_rounding_upward() { }
inline void set_builtin_rounding_toward_zero() { }

inline void set_builtin_rounding_mode(rounding_mode_t rnd) { }
inline rounding_mode_t get_builtin_rounding_mode() { return ROUND_TO_NEAREST; }

} // namespace Ariadne

#endif


/************  Import MPFR rounding mode controls *******************/

#include <mpfr.h>

/************  Publicly-accessible rounding-mode changing *******************/

namespace Ariadne {

using OutputStream = std::ostream;

//!@{
//! \ingroup NumericModule
//! \name Rounding mode control

//! \brief The rounding mode type used for builtin floating-point objects such as FloatDP. \ingroup NumericModule
typedef rounding_mode_t BuiltinRoundingModeType;
//! \brief The rounding mode type used for multiple-precision floating-point objects such as FloatMP. \ingroup NumericModule
typedef mpfr_rnd_t MPFRRoundingModeType;

//! \brief The floating-point environment value for rounding arithmetic to the nearest exactly-representable value.
extern const BuiltinRoundingModeType ROUND_TO_NEAREST;
//! \brief The floating-point environment value for upwards-rounded arithmetic.
extern const BuiltinRoundingModeType ROUND_DOWNWARD;
//! \brief The floating-point environment value for downwards-rounded arithmetic.
extern const BuiltinRoundingModeType ROUND_UPWARD;
//! \brief The floating-point environment value for rounding arithmetic to zero.
extern const BuiltinRoundingModeType ROUND_TOWARD_ZERO;

#ifdef DOXYGEN
//! \brief Set the builtin rounding mode. \ingroup NumericModule
void set_builtin_rounding_mode(BuiltinRoundingModeType rnd);
//! \brief Get the current rounding mode. \ingroup NumericModule
BuiltinRoundingModeType get_builtin_rounding_mode();

//! \brief Set the rounding mode to nearest. \ingroup NumericModule
void set_builtin_rounding_to_nearest();
//! \brief Set the rounding mode to downwards rounding. \ingroup NumericModule
void set_builtin_rounding_downward();
//! \brief Set the rounding mode to upwards rounding. \ingroup NumericModule
void set_builtin_rounding_upward();
//! \brief Set the rounding mode to towards-zero rounding. \ingroup NumericModule
void set_builtin_rounding_toward_zero();

//! \brief Set the rounding mode to the expected default rounding mode.
void set_default_builtin_rounding();
#endif

//! \brief The rounding mode type used for multiple-precision floating-point objects such as FloatMP. \ingroup NumericModule
typedef mpfr_rnd_t MPFRRoundingModeType;

//! \brief Tag class for downward rounding. Constants \ref downward, \ref down. \ingroup NumericModule
struct RoundDownward {
    constexpr operator BuiltinRoundingModeType() const { return ROUND_DOWNWARD; }
    constexpr operator MPFRRoundingModeType() const { return MPFR_RNDD; }
};
//! \brief Tag class for rounding to nearest. Constants \ref to_nearest, \ref near. \ingroup NumericModule
struct RoundToNearest {
    constexpr operator BuiltinRoundingModeType() const { return ROUND_TO_NEAREST; }
    constexpr operator MPFRRoundingModeType() const { return MPFR_RNDN; }
};
//! \brief Tag class for upward rounding. Constants \ref upward, \ref up. \ingroup NumericModule
struct RoundUpward {
    constexpr operator BuiltinRoundingModeType() const { return ROUND_UPWARD; }
    constexpr operator MPFRRoundingModeType() const { return MPFR_RNDU; }
};
//! \brief Tag class for rounding towards zero. %Constant \ref toward_zero. \ingroup NumericModule
struct RoundTowardZero {
    constexpr operator BuiltinRoundingModeType() const { return ROUND_TOWARD_ZERO; }
    constexpr operator MPFRRoundingModeType() const { return MPFR_RNDZ; }
};
//! \brief Tag class for approximate rounding. %Constant \ref approx. \ingroup NumericModule
struct RoundApproximately {
    constexpr operator BuiltinRoundingModeType() const { return ROUND_TO_NEAREST; }
    constexpr operator MPFRRoundingModeType() const { return MPFR_RNDN; }
};

struct CurrentRoundingMode { };
static const CurrentRoundingMode rounded = CurrentRoundingMode();

//! \brief General rounding mode class. \ingroup NumericModule
class Rounding {
    BuiltinRoundingModeType _rbp; MPFRRoundingModeType _rmp;
  public:
    Rounding(BuiltinRoundingModeType rbp, MPFRRoundingModeType rmp) : _rbp(rbp), _rmp(rmp) { } //!< <p/>
    Rounding(RoundDownward) : Rounding(ROUND_DOWNWARD,MPFR_RNDD) { } //!< <p/>
    Rounding(RoundToNearest) : Rounding(ROUND_TO_NEAREST,MPFR_RNDN) { } //!< <p/>
    Rounding(RoundUpward) :  Rounding(ROUND_UPWARD,MPFR_RNDU) { } //!< <p/>
    operator BuiltinRoundingModeType() const { return _rbp; } //!< <p/>
    operator MPFRRoundingModeType() const { return _rmp; } //!< <p/>
    friend OutputStream& operator<<(OutputStream& os, Rounding const& rnd) {
        return os << ( rnd._rbp == ROUND_TO_NEAREST ? "near" : (rnd._rbp == ROUND_DOWNWARD ? "down" : "up") ); } //!< <p/>
};


const RoundDownward downward = RoundDownward(); //!< Round exact answer downward to a representable value. Synonymous with \ref down. \ingroup NumericModule
const RoundToNearest to_nearest = RoundToNearest(); //!< Round exact answer to a nearest representable value. Synonymous with \ref near. \ingroup NumericModule
const RoundUpward upward = RoundUpward(); //!< Round exact answer upward to a representable value. Synonymous with \ref up. \ingroup NumericModule
const RoundTowardZero toward_zero = RoundTowardZero(); //!< Round exact answer to a representable value at least as close to zero. \ingroup NumericModule
const RoundApproximately approximately = RoundApproximately(); //!< Round exact answer to some close representable value, which need not be the nearest. Synonymous with \ref approx. \ingroup NumericModule
using RoundApprox = RoundApproximately; //!< . \ingroup NumericModule

const RoundDownward down = downward; //!< Round exact answer downward to a representable value. Synonymous with \ref downward. \ingroup NumericModule
const RoundToNearest near = to_nearest; //!< Round exact answer to a nearest representable value. Synonymous with \ref to_nearest. \ingroup NumericModule
const RoundUpward up = upward; //!< Round exact answer upward to a representable value. Synonymous with \ref upward. \ingroup NumericModule
const RoundApproximately approx = approximately; //!< Round exact answer to some close representable value, which need not be the nearest. Synonymous with \ref approximately. \ingroup NumericModule

//!@}

} // namespace Ariadne

#endif // ARIADNE_ROUNDING_HPP

