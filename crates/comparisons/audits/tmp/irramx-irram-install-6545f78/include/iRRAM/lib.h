/*

iRRAM_lib.h -- central header file for the iRRAM library
 
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


/*
Authors:  all by Norbert, except: 

  2003-07 INTERVAL extensions partly by Shao Qi

  2001-07 INTEGER and RATIONAL partly by Tom van Diessen


*/


/*!
 * \defgroup types Data Types
 * \brief Discrete and continuous data types.
 */

/*!
 * \defgroup sizetype sizetype
 * \brief Positive arithmetic type representing error and size bounds.
 * \ingroup types
 */

/*!
 * \defgroup maths Mathematical functions
 */

/*!
 * \defgroup debug Debug
 */

#ifndef iRRAM_LIB_H
#define iRRAM_LIB_H

#include <iRRAM/LAZYBOOLEAN.h>
#include <iRRAM/REAL.h>
#include <iRRAM/DYADIC.h>
#include <iRRAM/INTEGER.h>
#include <iRRAM/RATIONAL.h>
#include <iRRAM/REALMATRIX.h>
#include <iRRAM/SPARSEREALMATRIX.h>
#include <iRRAM/COMPLEX.h>
#include <iRRAM/INTERVAL.h>
#include <iRRAM/STREAMS.h>
#include <iRRAM/SWITCHES.h>
#include <iRRAM/FUNCTION.h>
#include <iRRAM/helper-templates.hh>
#include <iRRAM/limit_templates.h>

namespace iRRAM {

/*****************************************/
// iRRAM_exec template

namespace internal {

void init();

class run {

	state_t &st;
	stiff code;

	void loop_init();
	void loop_fini(int p_end);
public:
	run(state_t &st);
	~run();

	template <typename F,typename... Args>
	ret_void_t<F,Args...> exec(F f, const Args &... args)
	{
		ITERATION_DATA &actual_stack = st.ACTUAL_STACK;
		while (true) {
			loop_init();

			int p_end = 0;
			try {
				f(args...);
				if (iRRAM_likely(!st.infinite))
					break;
			} catch (const Iteration &it) {
				p_end = actual_stack.actual_prec + it.prec_diff;
			} catch (const iRRAM_Numerical_Exception &exc) {
				/* not handled by run::exec */
				cerr << "iRRAM exception: "
				     << iRRAM_error_msg[exc.type] << "\n";
				throw;
			}

			loop_fini(p_end);
		}
	}

	template <typename F,typename... Args>
	ret_value_t<F,Args...> exec(F f, const Args &... args)
	{
		ret_value_t<F,Args...> r;
		exec([f,&r](const Args &... args){ r = f(args...); }, args...);
		return r;
	}
};
}

template <typename F, typename... Args>
internal::ret_value_t<F,Args...> exec(F f, const Args &... args)
{
	internal::init();
	return internal::run(*state).exec(f, args...);
}

template <typename F, typename... Args>
internal::ret_void_t<F,Args...> exec(F f, const Args &... args)
{
	internal::init();
	internal::run(*state).exec(f, args...);
}

} // namespace iRRAM

#endif /* ! iRRAM_LIB_H */
