/*

iRRAM/SWITCHES.h -- header file for SWITCHES class of the iRRAM library
 
Copyright (C) 2001-2013 Norbert Mueller
 
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

/*! \defgroup switches Switches
 * \brief Control and tune iRRAM's state.
 *
 * Definition of switches
 * ----------------------
 * The iRRAM uses "switches" that start with a default behaviour.
 * Their value can be changed by declaring a variable of the corresponding type.
 * This change is effective until an other variable of the type is declared or
 * the variable is destroyed.
 * With the end of the lifetime of a variable, the previous behaviour will be
 * restored.
 *
 * Such switches exist for:
 * - explicit declaration of single_valued behaviour of a code section despite
 *   multi-valued operations
 * - switching the precision policy to absolute or relative precision
 * - temporary increase or decrease of the working precision for REAL
 * - declaration of the working precision of DYADIC operators (see DYACIC.h)
 *
 * The switches are thread-specific. */

#ifndef iRRAM_SWITCHES_H
#define iRRAM_SWITCHES_H

#include <cassert>
#include <iRRAM/common.h>

namespace iRRAM {

/*! \addtogroup switches
 * @{ */

/*! \brief
 * Explicit declaration of single_valued behaviour of a code section despite
 * multi-valued operations.
 *
 * single_valued code --> all possible computation paths again lead to
 * single-valued function
 *
 * obsoletes `continous_begin()`, `continous_end()` */
struct single_valued
{
	inline  single_valued() noexcept { state->ACTUAL_STACK.inlimit++; }
	inline ~single_valued() noexcept { --state->ACTUAL_STACK.inlimit; }
};


#define iRRAM_ABSOLUTE 0
#define iRRAM_RELATIVE 1
/*! \brief Switch the precision policy to absolute or relative precision.
 *
 * `precision_mode code(` \ref iRRAM_ABSOLUTE `)` -> work with error control based on absolute precision
 *
 * `precision_mode code(` \ref iRRAM_RELATIVE `)` -> work with error control based on relative precision
 *
 * \sa iRRAM_ABSOLUTE
 * \sa iRRAM_RELATIVE
 */
class precision_mode
{
	int saved;
public:
	inline precision_mode(int policy) noexcept
	: saved(state->ACTUAL_STACK.prec_policy)
	{
		state->ACTUAL_STACK.prec_policy = policy;
	}
	inline ~precision_mode() noexcept { state->ACTUAL_STACK.prec_policy = saved; }
};

/*! \brief Temporary increase of the working precision.
 *
 * stiff code    --> temporarily work with next precision step 
 * stiff code(n) --> temporarily work with precision n steps higher (n>0) or lower (n<0)
 *
 * obsoletes `stiff_begin()`, `stiff_end()` */
class stiff
{
	int saved;
protected:
	static inline void set_prec_step(int n) noexcept
	{
		if (n<1) n=1;
		if (iRRAM_prec_steps <= n) n = iRRAM_prec_steps-1;
		state->ACTUAL_STACK.prec_step = n;
		state->ACTUAL_STACK.actual_prec = iRRAM_prec_array[state->ACTUAL_STACK.prec_step];
		state->highlevel = (state->ACTUAL_STACK.prec_step > iRRAM_DEFAULT_PREC_START);
	}

public:
	struct abs {};
	struct rel {};

	template <typename T=rel> explicit stiff(int=1, T=T{}) noexcept;

	inline ~stiff() noexcept {
		set_prec_step(saved);
	}
	stiff(const stiff &) = delete;
	stiff operator=(const stiff &) = delete;

	int saved_step() const noexcept { return saved; }
	int saved_prec(int delta_step = 0) const noexcept { return iRRAM_prec_array[saved + delta_step]; }

	void inc_step(int n) const noexcept
	{
		set_prec_step(state->ACTUAL_STACK.prec_step + n);
	}
};
//! @} /* end group switches */

template <> inline stiff::stiff(int abs_step, abs) noexcept
: saved(state->ACTUAL_STACK.prec_step)
{ set_prec_step(abs_step); }

template <> inline stiff::stiff(int rel_step, rel) noexcept
: stiff(state->ACTUAL_STACK.prec_step + rel_step, abs{}) {}

/*! \addtogroup switches
 * @{ */

/*! \brief Temporarily halve the working precision. */
struct relaxed : public stiff {
	relaxed() : stiff((state->ACTUAL_STACK.prec_step+1)/2, abs{}) {}
};

/*! \brief Combination of \ref stiff and single_valued. */
struct limit_computation : public stiff, single_valued { using stiff::stiff; };

//! @} /* end group switches */

} // namespace iRRAM

#endif
