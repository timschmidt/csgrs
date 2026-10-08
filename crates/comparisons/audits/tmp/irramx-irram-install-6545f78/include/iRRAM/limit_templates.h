/*

iRRAM_limit_templates.h -- template definitions file for the limit operators of the iRRAM library
 
Copyright (C) 2005 Norbert Mueller
Copyright     2016 Franz Brausse
 
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
#ifndef iRRAM_LIMIT_TEMPLATES_H
#define iRRAM_LIMIT_TEMPLATES_H

#include <iRRAM/core.h>
#include <iRRAM/helper-templates.hh>
#include <iRRAM/sizetype.hh>
#include <iRRAM/SWITCHES.h>
#include <iRRAM/cache.h>
#include <iRRAM/STREAMS.h> /* iRRAM_DEBUG* */

/*!
 * \defgroup limits Limit operations
 * \ingroup maths
 *
 * Limit operations
 * ================
 * These functions compute the mathematical limit \f$y\f$ of sequences \f$f_p\f$
 * of continuous objects, optionally depending on discrete or continuous
 * arguments \f$x\f$ satisfying
 *
 *     \f[\forall p\in\mathbb{Z} : \Vert f_p(x)-y\Vert_\infty < 2^p\f]
 *
 * in the iRRAM limit for \f$x, f_p\f$ and \f$y\f$. In the first case, \f$x\f$
 * is discrete and therefore exact, the errors introduced stem from imprecision
 * of the computation \f$f_p\f$ itself, which is unbounded. Since \f$f_p\f$ is
 * executed within iRRAM, its mathematical operations are exact within
 * \f$2^q\f$, where \f$q\f$ is the current working precision, however the
 * objects operated upon are imprecise intervals and therefore \f$f_p\f$
 * introduces additional errors. To reduce their effect, the current working
 * precision \f$q\f$ is successively increased, starting at the initial \f$P\f$
 * when limit was invoked (`env.saved_prec()`), whenever \f$f_q\f$ could not be
 * computed. If at some point the computation \f$f_q\f$ succeeds, it has
 * computed \f$y\f$ up to \f$2^q < 2^p\f$. The error \f$2^p\f$ is added to
 * \f$y\f$'s error to ensure correctness.
 *
 * If \f$x\f$ is not exact, \f$|f_p(x)-y|\f$ generally is larger than \f$2^p\f$.
 * This additional error cannot get arbitrarily small when increasing \f$p\f$,
 * therefore a different scheme is employed: when computing \f$f_P(x)\f$ failed
 * and also \f$f_{p_0}(x)\f$ failed (where \f$p_0\f$ is the lowest precision
 * available in iRRAM), the limit cannot be computed with the current precision
 * of \f$x\f$ and therefore a reiteration is done.
 * If \f$f_P(x)\f$ succeeded but the resulting error \f${}+ 2^P\f$ is larger
 * than \f$2^{P[-1]}\f$ and is larger than \f$x\f$'s error \f${}- 2^{P[-1]}\f$,
 * then \f$p\f$ is set to \f$p_0\f$ for the potential next iteration. If
 * \f$y\f$'s error already is smaller than \f$2^P\f$, the iteration succeeded.
 * Otherwise the result is deemed too imprecise and the computation is repeated
 * with \f$p\f$ increased by 4 precision steps.
 *
 * Note
 * ----
 * There fundamentally are two different variants of limit(), one taking *only*
 * parameters representing discrete values and one for the mixed/only-continuous
 * case. Continuity of a type is determined through specializations of the type
 * trait
 *
 *     template <typename T> struct is_continuous;
 *
 * providing a static `bool value` as defined in <iRRAM/lib.h>.
 *
 * Which one is called when invoking `limit(f,args...)` is not determined by the
 * actual parameter types of f but by the types of args...
 * This is deliberate and consistent with how iRRAM works - i.e. allowing
 * implicit conversions from the types of smaller domain to those with larger
 * domain. Usually there is no need to explicitely specify the template types.
 *
 * Example:
 *
 *     REAL f(int prec, const REAL &x);
 *     REAL y = limit(f, 42)
 *
 * will *not* invoke the continuous variant since a REAL will be constructed
 * from the given int using REAL's implicit converting constructor while limit()
 * is being executed. That REAL's precision therefore is controlled by limit()
 * and being adjusted as to f's and limit's requirements to yield a precise
 * enough result.
 *
 * This behaviour is desirable as the non-continuous parameter version of
 * limit() is more efficient.
 *
 * Regarding the same example but calling
 *
 *     REAL x = 42; // or a complicated computation
 *     REAL y = limit(f, x);
 *
 * explains the necessity for the second version. It will use the continuous
 * version as the precision of the result of f(p,x) typically will depend on the
 * precision of x and not just on p. Therefore the continuous variant of limit()
 * provisions for reiterations outside of the limit() call to enable increased
 * precision of x itself.
 *
 * For a continuous parameter pack T (as determined by `any_continuous<T...>`) a
 * function
 *
 *     int geterror_exp(const T &...)
 *
 * needs to be provided which defaults to returning the maximum error exponent
 * as determined by
 *
 *     int geterror_exp(const Ti &t)
 *
 * the default implementation of which is
 *
 *     return geterror(t).exponent;
 *
 * but may be specialized according to the type's characteristics. In the future
 * it is possible that the continuous variant of limit() will use more
 * fine-grained error information than just the exponent, so it may be helpful
 * to additionally provide
 *
 *     sizetype geterror(const Ti &)
 *
 * for all continuous types Ti used as limit() parameters. Additionally,
 * declarations of
 *
 *     void     seterror(R &, sizetype)
 *     sizetype geterror(const R &)
 *
 * for result type R must be provided for both of the limit() variants. For
 * iRRAM's types REAL, *REALMATRIX and COMPLEX implementations are available.
 */

namespace iRRAM {

/*! \ingroup debug */
inline void limit_debug(const char* c){
  if ( iRRAM_unlikely(state->debug > 0) ){
    if (state->debug >=actual_stack().inlimit + 2 )
		cerr << c <<"...\n";
    if (state->max_prec <= actual_stack().prec_step) 
		state->max_prec  = actual_stack().prec_step;
  }
}

/*! \ingroup debug */
inline void limit_debug2(const char* c){
      if ( iRRAM_unlikely(state->debug > 0) ) {
	    if (state->debug >=actual_stack().inlimit + 2 )
		cerr << c <<", increasing precision locally to "
			<<actual_stack().actual_prec<<"\n";
	    if (state->max_prec <= actual_stack().prec_step) 
		state->max_prec  = actual_stack().prec_step;
      }
}

template <typename C,typename... ContArgs>
int      geterror_exp(const C &, const ContArgs &...);

template <typename S>
int geterror_exp(const S &x)
{
	return geterror(x).exponent;
}

template <typename S,typename T,typename... ContArgs>
typename std::enable_if<internal::is_continuous<S>::value &&
                        internal::any_continuous<T,ContArgs...>::value,int>::type
geterror_exp(const S &x, const T &y, const ContArgs &... z)
{
	return max(geterror_exp(x), geterror_exp(y, z...));
}

template <typename S,typename T,typename... ContArgs>
typename std::enable_if<internal::is_continuous<S>::value &&
                        !internal::any_continuous<T,ContArgs...>::value,int>::type
geterror_exp(const S &x, const T &, const ContArgs &...)
{
	return geterror_exp(x);
}

template <typename S,typename T,typename... ContArgs>
typename std::enable_if<!internal::is_continuous<S>::value,int>::type
geterror_exp(const S &, const T &y, const ContArgs &... z)
{
	static_assert(internal::any_continuous<T,ContArgs...>::value,
	              "geterror_exp() is only applicable to continuous types");
	return geterror_exp(y, z...);
}

/*! \addtogroup limits
 * @{ */

/* Requires for Result and ContArgs functions
 *  - int geterror_exp(const C &, const ContArgs &...)
 *    (usually it's enough to specialize the single-argument versions of
 *     geterror_exp(const C &) and geterror_exp(const ContArgs &)... or to
 *     overload geterror(const C &) and geterror(const ContArgs &)...;
 *     then the maximum-norm is used on the vector of all continuous parameters)
 *  - sizetype geterror(const Result &)
 *  - void seterror(Result &, const sizetype &)
 */
/*!
 * \brief Continuous variant of the limit operator.
 *
 * \param[in] f functional object of signature
 *              `Result (int prec, const ContArgs &...)`
 * \param[in] cont_args the mixed- or continuous arguments to be passed when
 *                      invoking f
 */
template <typename F,typename... ContArgs>
auto limit(F f, const ContArgs &... cont_args)
-> typename std::enable_if<internal::any_continuous<ContArgs...>::value,
                           decltype(f(0,cont_args...))>::type
{
	using Result = decltype(f(0,cont_args...));

	limit_computation env;

	Result lim, limnew;
	sizetype limnew_error, lim_error;
	int args_error_exp;

	int element      = env.saved_prec();
	int element_step = env.saved_step();
	int firsttime    = 2;

	args_error_exp = geterror_exp(cont_args...);

	limit_debug("starting general limit_gen1");

	while (1) {
		try {
			iRRAM_DEBUG2(2,"trying to compute general limit_gen1 "
			               "with precicion 2^(%d)...\n", element);
			limnew = f(element,cont_args...);
			limnew_error = sizetype_add_power2(geterror(limnew), element);
			if (firsttime == 2)
				if (limnew_error.exponent > env.saved_prec(-1)
				    && limnew_error.exponent > args_error_exp - env.saved_prec(-1)) {
					iRRAM_DEBUG2(2,"computation not precise enough (%d*2^%d), "
						"trying normal p-sequence\n",
						limnew_error.mantissa,
						limnew_error.exponent);
					element_step = 1;
					element = 4+iRRAM_prec_array[element_step];
					firsttime = 1;
				}
			if (firsttime != 0 ||
			    sizetype_less(limnew_error,lim_error)) {
				lim = limnew;
				lim_error = limnew_error;
				iRRAM_DEBUG2(2,"getting result with error %d*2^(%d)\n",
				               lim_error.mantissa, lim_error.exponent);
			} else {
				iRRAM_DEBUG1(2,"computation successful, but no improvement\n");
			}
			firsttime = 0;
			if (element <= env.saved_prec())
				break;
			element_step += 4;
			element = iRRAM_prec_array[element_step];
		} catch (const Iteration &it) {
			if (firsttime == 0) {
				iRRAM_DEBUG1(2,"computation failed, using best success\n");
				break;
			} else if (firsttime == 2) {
				iRRAM_DEBUG1(2,"computation failed, trying normal p-sequence\n");
				element_step = 1;
				element = 4+iRRAM_prec_array[element_step];
				firsttime = 1;
			} else {
				iRRAM_DEBUG1(1,"computation of general limit_gen1 failed totally\n");
				iRRAM_REITERATE(0);
			}
		}
	}
	seterror(lim, lim_error);
	iRRAM_DEBUG2(2,"end of general limit_gen1 with error %d*2^(%d)\n",
	             lim_error.mantissa, lim_error.exponent);
	return lim;
}

template <typename F,typename... DiscArgs>
auto limit(F f, const DiscArgs &... disc_args)
-> typename std::enable_if<!internal::any_continuous<DiscArgs...>::value,
                           decltype(f(0,disc_args...))>::type
{
	using Result = decltype(f(0,disc_args...));

	limit_computation env;

	Result lim;
	sizetype lim_error;

	limit_debug("starting general limit_0");

	while (1) {
		try {
			iRRAM_DEBUG2(2,"trying to compute general limit_0 with precision %d...\n",actual_stack().actual_prec);
			lim = f(env.saved_prec(), disc_args...);
			lim_error = geterror(lim);
			iRRAM_DEBUG2(2,"getting result with local error %d*2^(%d)\n", lim_error.mantissa, lim_error.exponent);
			break;
		} catch (const Iteration &it) {
			env.inc_step(2);
			limit_debug2("general limit_0 failed");
		}
	}
	lim_error = sizetype_add_power2(lim_error, env.saved_prec());
	seterror(lim, lim_error);
	iRRAM_DEBUG2(2,"end of limit_0 with error %d*2^(%d)\n",
	               lim_error.mantissa,lim_error.exponent);
	return lim;
}

template <typename R, typename C, typename... Args>
std::enable_if_t<internal::is_cacheable<C>::value, R>
limit_mv(R (*seq)(int prec, C &choice, const Args &... args),
         C choice, const Args &... x)
{
	bool inlimit = actual_stack().inlimit != 0;

	limit_computation env;

	R lim, limnew;
	sizetype limnew_error, element_error;
	sizetype lim_error;

	int element = env.saved_prec();
	int element_step = env.saved_step();
	int firsttime = 2;

	if (!inlimit &&
	    !get_cached(choice)) /* TODO: why only in case of !inlimit? */
		put_cached(choice);

	int x_error_exp = geterror_exp(x...);

	limit_debug("starting gen limit_mv");

	while (1) {
		try {
			iRRAM_DEBUG2(2, "trying to compute gen limit_mv with "
			                "precicion 2^(%d)...\n",
			             element);
			limnew = seq(element, choice, x...);
			modify_cached(choice);
			element_error = sizetype_power2(element);
			limnew.geterror(limnew_error);
			limnew_error += element_error;
			if (firsttime == 2)
				if (limnew_error.exponent >
				            env.saved_prec(-1) &&
				    limnew_error.exponent >
				            x_error_exp - env.saved_prec(-1)) {
					iRRAM_DEBUG2(2,
					             "computation not precise "
					             "enough (%d*2^%d), trying "
					             "normal p-sequence\n",
					             limnew_error.mantissa,
					             limnew_error.exponent);
					element_step = 1;
					element =
					        4 +
					        iRRAM_prec_array[element_step];
					firsttime = 1;
				}
		} catch (const Iteration &it) {
			if (firsttime == 0) {
				iRRAM_DEBUG1(2, "computation failed, using "
				                "best success\n");
				break;
			} else if (firsttime == 2) {
				iRRAM_DEBUG1(2, "computation failed, trying "
				                "normal p-sequence\n");
				element_step = 1;
				element = 4 + iRRAM_prec_array[element_step];
				firsttime = 1;
			} else {
				iRRAM_DEBUG1(1, "computation of limit_gen1 "
				                "failed totally\n");
				iRRAM_REITERATE(0);
			}
		}
		if (firsttime != 0 || sizetype_less(limnew_error, lim_error)) {
			lim = limnew;
			lim_error = limnew_error;
			iRRAM_DEBUG2(2, "getting result with error %d*2^(%d)\n",
			             lim_error.mantissa, lim_error.exponent);
		} else {
			iRRAM_DEBUG1(
			        2,
			        "computation successful, but no improvement\n");
		}
		firsttime = 0;
		if (element <= env.saved_prec())
			break;
		element_step += 4;
		element = iRRAM_prec_array[element_step];
	}
	lim.seterror(lim_error);
	iRRAM_DEBUG2(2, "end of gen limit_mv with error %d*2^(%d)\n",
	             lim_error.mantissa, lim_error.exponent);
	return lim;
}

template <class ARGUMENT, class DISCRETE, class RESULT>
RESULT  limit_lip (RESULT  (*f)(int,const ARGUMENT&,DISCRETE param),
            int lip_value,
	    bool (*on_domain)(const ARGUMENT&),
            const ARGUMENT& x,DISCRETE param)
{
  if ( on_domain(x) != true ) iRRAM_REITERATE(0);

  limit_computation env;

  ARGUMENT x_new;
  RESULT lim;
  sizetype lim_error,x_error;

  x_new=x;
  x_new.geterror(x_error);
  sizetype_exact(lim_error);
  x_new.seterror(lim_error);

  limit_debug("starting limit_lip");

  while (1) {
     try{
      iRRAM_DEBUG2(2,"trying to compute limit_lip with precision %d...\n",actual_stack().actual_prec);
    lim=f(env.saved_prec(),x_new,param);
    lim.geterror(lim_error);
    if (lim_error.exponent > env.saved_prec()) {
      env.inc_step(2);
      limit_debug2("limit_lip too imprecise");

   } else {
      iRRAM_DEBUG2(2,"getting result with local error %d*2^(%d)\n",
             lim_error.mantissa, lim_error.exponent);
      break;
    }}
    catch ( Iteration it){
      env.inc_step(2);
      limit_debug2("limit_lip failed");
    } }
  lim_error = sizetype_power2(env.saved_prec());
  lim.adderror(lim_error);
  lim_error = x_error << lip_value;
  lim.adderror(lim_error);
  iRRAM_DEBUG2(2,"end of limit_lip with error %d*2^(%d)\n"
                 "  error of argument: %d*2^(%d)\n",
                 lim.geterror().mantissa,lim.geterror().exponent,
                 x_error.mantissa,x_error.exponent);
  return lim;
}




// template <class ARGUMENT, class PARAM, class RESULT>
// RESULT lipschitz (RESULT f(const PARAM& param,const ARGUMENT& ),
//             REAL lip_f(const PARAM& param, const ARGUMENT&),
// 	    bool on_domain(const PARAM& param, const ARGUMENT&),
//             const PARAM& param, const ARGUMENT& x)
// {
//   if ( on_domain(param,x) != true ) iRRAM_REITERATE(0);
// 
//   ITERATION_STACK SAVED_STACK;
// 
//   single_valued code;
//   iRRAM_DEBUG1(2,"starting lipschitz1b ...\n");
// 
// // for the computation of the Lipschitz bound, we work with
// // reduced precision:
//   ACTUAL_STACK.prec_step=(ACTUAL_STACK.prec_step+1)/2;
//   ACTUAL_STACK.actual_prec=iRRAM_prec_array[ACTUAL_STACK.prec_step];
//   iRRAM_highlevel = (ACTUAL_STACK.prec_step > 1);
// 
//   lip_bound=lip_f(param,x);
// 
//   ACTUAL_STACK.prec_step=SAVED_STACK.prec_step;
//   ACTUAL_STACK.actual_prec=iRRAM_prec_array[ACTUAL_STACK.prec_step];
//   iRRAM_highlevel = (ACTUAL_STACK.prec_step > 1);
// 
//   ARGUMENT x_new;
//   RESULT lip_result,lip_bound;
//   sizetype lip_error,lip_size,tmp_size,x_error;
// 
//   bool try_it=true;
//   x_new=x;
//   x_new.geterror(x_error);
//   sizetype_exact(x_new.error);
// 
// 
//   while (try_it) {
//   try { try_it=false;
//         lip_result=f(param,x_new); }
//   catch ( Iteration it)  { try_it=true;
//       ACTUAL_STACK.prec_step+=2;
//       ACTUAL_STACK.actual_prec=iRRAM_prec_array[ACTUAL_STACK.prec_step];
//       iRRAM_highlevel = (ACTUAL_STACK.prec_step > 1);
//       iRRAM_DEBUG2(2,"limit_lip2 failed, increasing precision locally to %d...\n",ACTUAL_STACK.actual_prec);
//     }
//   }
// 
//   iRRAM_DEBUG2(2,"getting result with local error %d*2^(%d)\n",
//              lip_result.error.mantissa, lip_result.error.exponent);
//   lip_size=lip_bound.vsize;
//   lip_bound.geterror(tmp_size);
//   lip_size += tmp_size;
//   lip_error = lip_size * x_error;
//   lip_result.adderror(lip_error);
//   iRRAM_DEBUG0(2,{lip_result.geterror(lip_error);
//             fprintf(stderr,"end of lipschitz_1b with error %d*2^(%d)\n",
//               lip_error.mantissa,lip_error.exponent);
//             fprintf(stderr,"  for argument with error %d*2^(%d)\n",
//               x_error.mantissa,x_error.exponent);});
//   return lip_result;
// }

template <class CONT_ARGUMENT, class DISCRETE_ARGUMENT, class RESULT, class PARAM>
RESULT lipschitz_1p_1a (RESULT (*f)(const DISCRETE_ARGUMENT&, const PARAM& param),
            int lip,
            bool (*on_domain)(const CONT_ARGUMENT&, const PARAM& param),
            const CONT_ARGUMENT& x, const PARAM& param )
{
  if ( on_domain(x,param) != true ) iRRAM_REITERATE(0);

  limit_computation env(0);
  iRRAM_DEBUG1(2,"starting lipschitz_1p_1a ...\n");

  DISCRETE_ARGUMENT x_center;
  RESULT lip_result;
  sizetype lip_error,x_error;

  bool try_it=true;
  x.to_formal_ball(x_center,x_error);

  while (try_it) {
    try { try_it=false;
        lip_result=f(x_center,param); }
    catch ( Iteration it)  { try_it=true;
      env.inc_step(2);
      iRRAM_DEBUG2(2,"lipschitz_1p_1a failed, increasing precision locally to step %d...\n",actual_stack().prec_step);
    }
  }

  iRRAM_DEBUG2(2,"getting result with local error %d*2^(%d)\n",
             lip_result.error.mantissa, lip_result.error.exponent);
  lip_error = x_error << lip;
  lip_result.adderror(lip_error);
  iRRAM_DEBUG2(2,"end of lipschitz_1p_1a with error %d*2^(%d)\n"
                 "  for argument with error %d*2^(%d)\n",
                 lip_result.geterror().mantissa,lip_result.geterror().exponent,
                 x_error.mantissa,x_error.exponent);
  return lip_result;
}

//********************************************************************************
// general limit operator for FUNCTION objects on REAL numbers
//
// REAL limit ( FUNCTION<REAL,int> f )
// if FUNCTION f defines a normed Cauchy sequence, i.e. |f(i)-x|<= 2^{i}
// then limit(f) returns x
//********************************************************************************
REAL limit(FUNCTION<REAL,int> f);

REAL limit_hint(REAL (*f)(int, const REAL &),
                int hint, const REAL &x);
REAL limit_hint(REAL (*f)(int, const REAL &, const REAL &),
                int hint, const REAL &x, const REAL &y);
REAL iteration (void (*f)(REAL &, REAL &, const int &param),
                const REAL &l, const REAL &r, const int &param);
REAL limit     (const FUNCTION<REAL,int> &f);

/****************************************************************************/
// extern templates for limit operators
/****************************************************************************/

extern template REAL       limit(REAL (*f)(int));

extern template REAL       limit(REAL (*f)(int,const REAL &),
                                 const REAL &);
extern template REAL       limit(REAL (*f)(int,const REAL &,const REAL &),
                                 const REAL &, const REAL &);
extern template REAL       limit(REAL (*)(int, const REAL &, int),
                                 const REAL &, const int &);
extern template COMPLEX    limit(COMPLEX (*)(int, COMPLEX const &),
                                 const COMPLEX &);
extern template REALMATRIX limit(REALMATRIX (*)(int, REALMATRIX const &),
                                 const REALMATRIX &);

// extern templates for multi-valued limits:
extern template COMPLEX limit_mv
    (COMPLEX (*)(int, int &, COMPLEX const &), int, COMPLEX const &);

extern template REALMATRIX limit_mv<REALMATRIX,int,REALMATRIX>
    (REALMATRIX (*)(int, int &, REALMATRIX const &), int, REALMATRIX const &);


//! @} /* end group limits */

} /* ! namespace iRRAM */


#endif /* ! iRRAM_LIMIT_TEMPLATES_H */
