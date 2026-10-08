/*

iRRAM_core.h -- basic file for the errorsize arithmetic 
                and for the templates in the iRRAM library
 
Copyright (C) 2001-2009 Norbert Mueller
Copyright     2014-2016 Franz Brausse
 
This file is part of the iRRAM Library.
 
The iRRAM Library is free software; you can redistribute it and/or modify
it under the terms of the GNU Library General Public License as published by
the Free Software Foundation; either version 2 of the License, or (at your
option) any later version.
 
The iRRAM Library is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Library General Public
License for more details.
 
You should have received a copy of the GNU Library General Public License
along with the iRRAM Library; see the file COPYING.LIB.  If not, write to
the Free Software Foundation, Inc., 59 Temple Place - Suite 330, Boston,
MA 02111-1307, USA. 
*/
#ifndef IRRAM_CORE_H
#define IRRAM_CORE_H

#include <cstdio>	/* fprintf(3) */
#include <algorithm>	/* std::min, std::max */
#include <cstdint>	/* int32_t, uint32_t */
#include <climits>
#include <memory>	/* std::unique_ptr<state_t> */

#include <iRRAM/common.h>

#if iRRAM_HAVE_TLS
# include <thread>	/* std::this_thread */
#endif

#ifndef iRRAM_BACKENDS
# error error: no usable backend, defined iRRAM_BACKENDS
#endif

#if iRRAM_BACKEND_MPFR
# include <iRRAM/MPFR_interface.h>
#else
# error "Currently no additional backend!"
#endif

namespace iRRAM {

/*!
 * \brief Computes the number of leading zeros of its integral argument.
 * \param x integer
 * \return Number of leading zero bits of x, also when `x == 0`.
 */
template <typename T>
static inline unsigned clz(T x)
{
	unsigned r = x != 0;
	for (unsigned m = (sizeof(x) * CHAR_BIT) >> 1; m > 0; m >>= 1)
		if (x >= (T)1 << m) {
			r += m;
			x >>= m;
		}
	return sizeof(x) * CHAR_BIT - r;
}
#if defined(__GNUC__) || defined(__clang__)
# define CLZ_BODY(x,clz) { return x ? clz(x) : sizeof(x) * CHAR_BIT; }
template <> inline unsigned clz(unsigned x)           CLZ_BODY(x,__builtin_clz)
template <> inline unsigned clz(unsigned long x)      CLZ_BODY(x,__builtin_clzl)
template <> inline unsigned clz(unsigned long long x) CLZ_BODY(x,__builtin_clzll)
# undef CLZ_BODY
#endif

using std::min;
using std::max;

/* \ingroup sizetype */
template <typename M,typename E>
struct generic_sizetype {
	typedef M mantissa_t;
	typedef E exponent_t;
	M mantissa;
	E exponent;
};

/* \ingroup sizetype */
typedef generic_sizetype<uint32_t,int32_t> sizetype;

// forward declaration of some classes

class INTEGER;
class RATIONAL;
class DYADIC;
class LAZY_BOOLEAN;
class REAL;
class COMPLEX;
class INTERVAL;
class REALMATRIX;
class SPARSEREALMATRIX;
template <typename R,typename... Args> class FUNCTION;
namespace internal {
struct cachelist;
}
struct mv_cache;


struct iRRAM_Numerical_Exception {
	iRRAM_Numerical_Exception(const int msg) noexcept : type(msg) {}
	// private:
	int type;
};

struct ITERATION_DATA {
	int prec_policy;
	int inlimit;
	int actual_prec;
	int prec_step;
};

struct state_t {
	int debug = iRRAM_DEFAULT_DEBUG;
	int infinite = 0;
	int prec_skip = iRRAM_DEFAULT_PREC_SKIP;
	int max_prec = 1;
	int prec_start = iRRAM_DEFAULT_PREC_START;
	int prec_array[iRRAM_prec_steps];
	bool highlevel = false; /* TODO: remove: iRRAM-timings revealed no performance loss */
	/* The following boolean "inReiterate" is used to distinguish voluntary
	 * deletions of rstreams from deletions initiated by iterations.
	 * The latter should be ignored, as stream operations using this stream
	 * might continue in later iterations! */
	bool inReiterate = false;
	int DYADIC_precision = -60;
	internal::cachelist *cache_active = nullptr;
	int max_active = 0;
	mv_cache *cache_address = nullptr;

	iRRAM_ext_mpfr_cache_t ext_mpfr_cache = iRRAM_EXT_MPFR_CACHE_INIT;
	iRRAM_mpz_cache_t mpz_cache = iRRAM_MPZ_CACHE_INIT;
	iRRAM_mpq_cache_t mpq_cache = iRRAM_MPQ_CACHE_INIT;

	REAL *ln2_val = nullptr;
	int   ln2_err = 0;
	REAL *pi_val = nullptr;
	int   pi_err = 0;

	// the two counters are used to determine whether output is actually
	// produced
	long long requests = 0;
	long long outputs  = 0;

	ITERATION_DATA ACTUAL_STACK{
		 1, /* prec_policy relative */
		 0, /* !inlimit */
		-1,
		-1,
	};

	state_t();
	~state_t();
};

namespace internal {

template <bool tls> struct state_proxy;

template <> struct state_proxy<true> : protected std::unique_ptr<state_t> {
	using std::unique_ptr<state_t>::operator*;
	using std::unique_ptr<state_t>::operator->;
	using std::unique_ptr<state_t>::release;

	state_proxy();
};

template <> struct state_proxy<false> {
	state_proxy() {}

	const state_t & operator*() const { return st; }
	      state_t & operator*()       { return st; }

	const state_t * operator->() const { return &st; }
	      state_t * operator->()       { return &st; }

	void release() const {}
private:
	state_t st;
};

}

extern iRRAM_TLS internal::state_proxy<iRRAM_HAVE_TLS> state;

inline const ITERATION_DATA & actual_stack(const state_t &st = *state)
{
	return st.ACTUAL_STACK;
}

template <typename T> class cache;

template <typename T> cache<T> & get_cache(const state_t &st = *state);
template <typename T> bool get_cached(T &t, const state_t &st = *state);
template <typename T> void put_cached(const T &t, const state_t &st = *state);
template <typename T> void modify_cached(const T &t, const state_t &st = *state);

template <typename T>
inline cache<T> & get_cache(const state_t &st)
{
	return *static_cast<cache<T> *>(st.cache_address);
}

template <typename T>
inline bool get_cached(T &t, const state_t &st)
{
	return st.ACTUAL_STACK.inlimit == 0 && get_cache<T>(st).get(t);
}

template <typename T>
inline void put_cached(const T &t, const state_t &st)
{
	if (st.ACTUAL_STACK.inlimit == 0)
		get_cache<T>(st).put(t);
}

template <typename T>
inline void modify_cached(const T &t, const state_t &st)
{
	if (st.ACTUAL_STACK.inlimit == 0)
		get_cache<T>(st).modify(t);
}

static struct {
	int operator[](int n) const noexcept { return state->prec_array[n]; }
} const iRRAM_prec_array;

extern void resources(double&,unsigned int&);
extern double ln2_time;
extern double pi_time;
void show_statistics();

inline bool debug_enabled(int level)
{
	const state_t &st = *state;
	return iRRAM_unlikely(st.debug >= actual_stack(st).inlimit + level);
}

#ifndef NODEBUG
  #define iRRAM_DEBUG0(level,...)                                               \
	do {                                                                    \
		if (debug_enabled(level)) {                                     \
			__VA_ARGS__;                                            \
		}                                                               \
	} while (0)
#else
  #define iRRAM_DEBUG0(level,...)
#endif
#if iRRAM_HAVE_TLS
# define iRRAM_DEBUG1(level,p) \
	iRRAM_DEBUG0((level),cerr << "thread " << std::this_thread::get_id() << ": " << p)
# define iRRAM_DEBUG2(level,fmt,...) \
	iRRAM_DEBUG0((level),fprintf(stderr,"thread 0x%zx " fmt,\
	             []{\
			union {\
				std::thread::id id;\
				typename std::thread::native_handle_type nat;\
			} u = { std::this_thread::get_id() };\
			return u.nat;}(),\
	             __VA_ARGS__))
#else
# define iRRAM_DEBUG1(level,p)	iRRAM_DEBUG0((level),cerr << p)
# define iRRAM_DEBUG2(level,...)	iRRAM_DEBUG0((level),fprintf(stderr,__VA_ARGS__))
#endif

struct Iteration {
	int prec_diff;
	constexpr Iteration(int p) : prec_diff(p) {}
};

// inline void iRRAM_REITERATE(int p_diff){inReiterate = true; throw Iteration(p_diff); }
#define iRRAM_REITERATE(x)                                                           \
	do {                                                                   \
		state->inReiterate = true;                                     \
		throw Iteration(x);                                            \
	} while (0)


enum struct float_form : int {
	absolute,
	relative,
	show,
};

#define iRRAM_float_absolute ::iRRAM::float_form::absolute
#define iRRAM_float_relative ::iRRAM::float_form::relative
#define iRRAM_float_show     ::iRRAM::float_form::show

} /* ! namespace iRRAM */

#endif /* ! iRRAM_CORE_H  */
