/*

iRRAM_FUNCTION.h -- header file for the FUNCTION part of the iRRAM library

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

#ifndef iRRAM_FUNCTION_H
#define iRRAM_FUNCTION_H
#include <utility>
#include <vector>
#include <functional>	/* std::function */
#include <memory>	/* std::shared_ptr */

namespace iRRAM {


template<class RESULT,class... PARAM>
class FUNCTION;

template<class RESULT,class... PARAM>
class FUNCTIONAL_object
{
public:
	FUNCTIONAL_object() {}
	virtual ~FUNCTIONAL_object() {}

	virtual RESULT eval(const PARAM &...) {RESULT r; return r;}
};

template<class RESULT,class... PARAM>
class FUNCTIONAL_algorithm :public FUNCTIONAL_object<RESULT,PARAM...>
{
public:
	std::function<RESULT(const PARAM &...)> _eval;

	FUNCTIONAL_algorithm(const std::function<RESULT(const PARAM &...)> &evalp) : _eval(evalp) {}
	FUNCTIONAL_algorithm(std::function<RESULT(const PARAM &...)> &&evalp) : _eval(evalp) {}

	RESULT eval(const PARAM &... z){return _eval(z...);};
};


template<class RESULT,class... PARAM>
class FUNCTIONAL_value :public FUNCTIONAL_object<RESULT,PARAM...>
{
public:
	RESULT _value;

	FUNCTIONAL_value(
		const RESULT &value
	) {_value=value;}

	RESULT eval(const PARAM&...) {return _value;}
};

//********************************************************************************


template<class RESULT,class... PARAM>
class FUNCTION {
public: 
	std::shared_ptr<FUNCTIONAL_object<RESULT,PARAM...>> _fobject;

	FUNCTION(
		FUNCTIONAL_object<RESULT,PARAM...> * fobject
	) : _fobject(fobject) {}

	FUNCTION() : _fobject(new FUNCTIONAL_object<RESULT,PARAM...>) {}

	FUNCTION(RESULT (*evalp)(const PARAM &... z))
	: _fobject(new FUNCTIONAL_algorithm<RESULT,PARAM...>(evalp)) {}

	FUNCTION(
		const FUNCTION &z
	) : _fobject(z._fobject) {}

	inline FUNCTION& operator = (
		const FUNCTION &z
	) {
		_fobject = z._fobject;
		return (*this);
	}

	template <typename... Args>
	inline auto bind(Args &&... args) -> decltype(from_algorithm(std::bind(*_fobject, args...)))
	{
		return from_algorithm(std::bind(*_fobject, args...));
	}

	inline RESULT operator()(
		const PARAM &... z
	) const { return _fobject->eval(z...); } 

	template<class PRIMARY> inline FUNCTION<PRIMARY,RESULT> operator () (
		const FUNCTION<PRIMARY,PARAM...> &inner
	) const {
		return compose(*this, inner);
	}
};

//********************************************************************************

template<class RESULT,class... PARAM>
inline FUNCTION<RESULT,PARAM...> from_algorithm(
	RESULT (evalp)(const PARAM &... z)
) {
	return new FUNCTIONAL_algorithm<RESULT,PARAM...>(evalp);
}

template<class RESULT,class... PARAM>
inline FUNCTION<RESULT,PARAM...> from_algorithm(
	const std::function<RESULT(const PARAM &...)> &evalp
) {
	return new FUNCTIONAL_algorithm<RESULT,PARAM...>(evalp);
}

template<class RESULT,class... PARAM>
inline FUNCTION<RESULT,PARAM...> from_value(const RESULT &value)
{
	return new FUNCTIONAL_value<RESULT,PARAM...>(value);
}

template<class RESULT,class INTERMEDIATE,class... PARAM>
inline FUNCTION<RESULT,PARAM...> compose(
	const FUNCTION<RESULT,INTERMEDIATE>&outer,
	const FUNCTION<INTERMEDIATE,PARAM...> &inner
) {
	return from_algorithm(
		std::function<RESULT(const PARAM &...)>(
			[=](const PARAM &... z)
			{
				return outer(inner(z...));
			}
		)
	);
}




template<class RESULT1, class... PARAM1, class RESULT2, class... PARAM2>
inline FUNCTION<std::pair<RESULT1,RESULT2>,std::pair<PARAM1...,PARAM2...>> product( 
	const FUNCTION<RESULT1,PARAM1...> &f_1, 
	const FUNCTION<RESULT2,PARAM2...> &f_2
) {
	return from_algorithm(
		std::function<std::pair<RESULT1,RESULT2>(const std::pair<PARAM1...,PARAM2...> &)>(
			[=](const std::pair<PARAM1...,PARAM2...> &z)
			{
				return std::make_pair(f_1(z.first), f_2(z.second));
			}
		)
	);
}

template<class RESULT1, class RESULT2, class... PARAM>
inline FUNCTION<std::pair<RESULT1,RESULT2>,PARAM... > juxtaposition(
	const FUNCTION<RESULT1,PARAM...> &f_1, 
	const FUNCTION<RESULT2,PARAM...> &f_2
) {
	return from_algorithm(
		std::function<std::pair<RESULT1,RESULT2>(const PARAM &...)>(
			[=](const PARAM &... z)
			{
				return std::make_pair(f_1(z...),f_2(z...));
			}
		)
	);
}

template<class RESULT1, class RESULT2, class... PARAM>
inline FUNCTION<RESULT1,PARAM... > first(
	const FUNCTION<std::pair<RESULT1,RESULT2>,PARAM... > &f
) {
	return from_algorithm(
		std::function<RESULT1(const PARAM &...)>(
			[=](const PARAM &... z)
			{
				return f(z...).first;
			}
		)
	);
}

template<class RESULT1, class RESULT2, class... PARAM>
inline FUNCTION<RESULT2,PARAM... > second(
	const FUNCTION<std::pair<RESULT1,RESULT2>,PARAM... > &f
) {
	return from_algorithm(
		std::function<RESULT2(const PARAM &...)>(
			[=](const PARAM &... z)
			{
				return f(z...).second;
			}
		)
	);
}
/*
template<class PARAM1, class PARAM2, class RESULT>
inline FUNCTION<PARAM2,RESULT> bind_first ( 
	const FUNCTION<std::pair<PARAM1,PARAM2>,RESULT > &f,
	const PARAM1& x
	){
		return new FUNCTIONAL_bind_first<PARAM1,PARAM2,RESULT>(f._fobject,x);
	}

template<class PARAM1, class PARAM2, class RESULT>
inline FUNCTION<PARAM1,RESULT> bind_second ( 
	const FUNCTION<std::pair<PARAM1,PARAM2>,RESULT > &f,
	const PARAM2& y
	){
		return new FUNCTIONAL_bind_second<PARAM1,PARAM2,RESULT>(f._fobject,y);
	}
*/
template<class RESULT,class... PARAM>
inline FUNCTION<RESULT,PARAM...> projection(
	const FUNCTION<std::vector<RESULT>,PARAM... > &f,
	const int i 
) {
	return from_algorithm(
		std::function<RESULT(const PARAM &...)>(
			[=](const PARAM &... z)
			{
				return f(z...)[i];
			}
		)
	);
}

//********************************************************************************


template<class PARAM>
inline FUNCTION<PARAM,PARAM> polynomial(
	const std::vector<PARAM> &coeff
) {
	return from_algorithm(
		std::function<PARAM(const PARAM &)>(
			[=](const PARAM &z)
			{
				PARAM sum = 0;
				PARAM factor = 1;
				for (unsigned int i=0; i<coeff.size(); i++) {
					sum += factor * coeff[i];
					factor *= z;
				}
				return sum;
			}
		)
	);
}


//********************************************************************************


template<class RESULT,class... PARAM>
inline FUNCTION<RESULT,PARAM...> addition(
	const FUNCTION<std::pair<RESULT,RESULT>,PARAM... > &a
) {
	return from_algorithm(
		std::function<RESULT(const PARAM &...)>(
			[=](const PARAM &... z)
			{
				std::pair<RESULT,RESULT> r = a(z...);
				return r.first + r.second;
			}
		)
	);
}

template<class RESULT,class... PARAM>
inline FUNCTION<RESULT,PARAM...> operator + ( 
	const FUNCTION<RESULT,PARAM...> &a,
	const FUNCTION<RESULT,PARAM...> &b
	){
		return addition(juxtaposition(a,b));
	}


//********************************************************************************


template<class RESULT,class... PARAM>
inline FUNCTION<RESULT,PARAM...> subtraction(
	const FUNCTION<std::pair<RESULT,RESULT>,PARAM... > &a
) {
	return from_algorithm(
		std::function<RESULT(const PARAM &...)>(
			[=](const PARAM &... z)
			{
				std::pair<RESULT,RESULT> r = a(z...);
				return r.first - r.second;
			}
		)
	);
}

template<class RESULT,class... PARAM>
inline FUNCTION<RESULT,PARAM...> operator - ( 
	const FUNCTION<RESULT,PARAM...> &a,
	const FUNCTION<RESULT,PARAM...> &b
	){
		return subtraction(juxtaposition(a,b));
	}


//********************************************************************************


template<class RESULT,class... PARAM>
inline FUNCTION<RESULT,PARAM...> multiplication(
	const FUNCTION<std::pair<RESULT,RESULT>,PARAM... > &a
) {
	return from_algorithm(
		std::function<RESULT(const PARAM &...)>(
			[=](const PARAM &... z)
			{
				std::pair<RESULT,RESULT> r = a(z...);
				return r.first * r.second;
			}
		)
	);
}

template<class RESULT,class... PARAM>
inline FUNCTION<RESULT,PARAM...> operator * ( 
	const FUNCTION<RESULT,PARAM...> &a,
	const FUNCTION<RESULT,PARAM...> &b
	){
		return multiplication(juxtaposition(a,b));
	}


//********************************************************************************


template<class RESULT,class... PARAM>
inline FUNCTION<RESULT,PARAM...> division(
	const FUNCTION<std::pair<RESULT,RESULT>,PARAM... > &a
) {
	return from_algorithm(
		std::function<RESULT(const PARAM &...)>(
			[=](const PARAM &... z)
			{
				std::pair<RESULT,RESULT> r = a(z...);
				return r.first / r.second;
			}
		)
	);
}

template<class RESULT,class... PARAM>
inline FUNCTION<RESULT,PARAM...> operator / ( 
	const FUNCTION<RESULT,PARAM...> &a,
	const FUNCTION<RESULT,PARAM...> &b
	){
		return division(juxtaposition(a,b));
	}


//********************************************************************************


template<class RESULT,class... PARAM>
inline FUNCTION<RESULT,PARAM...> operator + ( 
	const FUNCTION<RESULT,PARAM...> &a,
	const RESULT &b
	){
		return addition(juxtaposition(a,from_value<RESULT,PARAM...> (b)));
	}

template<class RESULT,class... PARAM>
inline FUNCTION<RESULT,PARAM...> operator + ( 
	const RESULT &a,
	const FUNCTION<RESULT,PARAM...> &b
	){
		return addition(juxtaposition(from_value<RESULT,PARAM...> (a) ,b));
	}


//********************************************************************************


template<class RESULT,class... PARAM>
inline FUNCTION<RESULT,PARAM...> operator * ( 
	const FUNCTION<RESULT,PARAM...> &a,
	const RESULT &b
	){
		return multiplication(juxtaposition(a,from_value<RESULT,PARAM...> (b)));
	}

template<class RESULT,class... PARAM>
inline FUNCTION<RESULT,PARAM...> operator * ( 
	const RESULT &a,
	const FUNCTION<RESULT,PARAM...> &b
	){
		return multiplication(juxtaposition(from_value<RESULT,PARAM...> (a) ,b));
	}


//********************************************************************************


template<class RESULT,class... PARAM>
inline FUNCTION<RESULT,PARAM...> operator - ( 
	const FUNCTION<RESULT,PARAM...> &a,
	const RESULT &b
	){
		return subtraction(juxtaposition(a,from_value<RESULT,PARAM...> (b)));
	}

template<class RESULT,class... PARAM>
inline FUNCTION<RESULT,PARAM...> operator - ( 
	const RESULT &a,
	const FUNCTION<RESULT,PARAM...> &b
	){
		return subtraction(juxtaposition(from_value<RESULT,PARAM...> (a) ,b));
	}


//********************************************************************************


template<class RESULT,class... PARAM>
inline FUNCTION<RESULT,PARAM...> operator / ( 
	const FUNCTION<RESULT,PARAM...> &a,
	const RESULT &b
	){
		return division(juxtaposition(a,from_value<RESULT,PARAM...> (b)));
	}

template<class RESULT,class... PARAM>
inline FUNCTION<RESULT,PARAM...> operator / ( 
	const RESULT &a,
	const FUNCTION<RESULT,PARAM...> &b
	){
		return division(juxtaposition(from_value<RESULT,PARAM...> (a) ,b));
	}


//********************************************************************************


// Be careful: The following operator "f<g" yields a function h,
// such that h(x)=true <=> f(x)<g(x)   (pointwise!)

template<class RESULT,class... PARAM>
inline FUNCTION<LAZY_BOOLEAN,PARAM...> less_than(
	const FUNCTION<std::pair<RESULT,RESULT>,PARAM... > &a
) {
	// return new FUNCTIONAL_less_than<RESULT,PARAM...>(a._fobject);
	return from_algorithm(
		std::function<LAZY_BOOLEAN(const PARAM &...)>(
			[=](const PARAM &... z)
			{
				std::pair<RESULT,RESULT> r = a(z...);
				return (r.first) < (r.second);
			}
		)
	);
}

template<class RESULT,class... PARAM>
inline FUNCTION<LAZY_BOOLEAN,PARAM...> operator < ( 
	const FUNCTION<RESULT,PARAM...> &a,
	const FUNCTION<RESULT,PARAM...> &b
	){
		return less_than(juxtaposition(a,b));
	}

template<class RESULT,class... PARAM>
inline FUNCTION<LAZY_BOOLEAN,PARAM...> operator > ( 
	const FUNCTION<RESULT,PARAM...> &a,
	const FUNCTION<RESULT,PARAM...> &b
	){
		return less_than(juxtaposition(b,a));
	}


//********************************************************************************


template<class RESULT,class... PARAM>
inline FUNCTION<LAZY_BOOLEAN,PARAM...> operator < ( 
	const FUNCTION<RESULT,PARAM...> &a,
	const RESULT &b
	){
		return less_than(juxtaposition(a,from_value<RESULT,PARAM...> (b)));
	}

template<class RESULT,class... PARAM>
inline FUNCTION<LAZY_BOOLEAN,PARAM...> operator < ( 
	const RESULT &a,
	const FUNCTION<RESULT,PARAM...> &b
	){
		return less_than(juxtaposition(from_value<RESULT,PARAM...> (a) ,b));
	}

template<class RESULT,class... PARAM>
inline FUNCTION<LAZY_BOOLEAN,PARAM...> operator > ( 
	const FUNCTION<RESULT,PARAM...> &a,
	const RESULT &b
	){
		return less_than(juxtaposition(from_value<RESULT,PARAM...> (b),a));
	}

template<class RESULT,class... PARAM>
inline FUNCTION<LAZY_BOOLEAN,PARAM...> operator > ( 
	const RESULT &a,
	const FUNCTION<RESULT,PARAM...> &b
	){
		return less_than(juxtaposition(b,from_value<RESULT,PARAM...> (a)));
	}

//********************************************************************************


template<class... PARAM>
inline FUNCTION<LAZY_BOOLEAN,PARAM...> LAZY_BOOLEAN_FUNCTION_and(
	const FUNCTION<std::pair<LAZY_BOOLEAN,LAZY_BOOLEAN>,PARAM... > &a
) {
	return from_algorithm(
		std::function<LAZY_BOOLEAN(const PARAM &...)>(
			[=](const PARAM &... z)
			{
				std::pair<LAZY_BOOLEAN,LAZY_BOOLEAN> r = a(z...);
				return (r.first) && (r.second);
			}
		)
	);
}

template<class... PARAM>
inline FUNCTION<LAZY_BOOLEAN,PARAM...> operator && ( 
	const FUNCTION<LAZY_BOOLEAN,PARAM...> &a,
	const FUNCTION<LAZY_BOOLEAN,PARAM...> &b
	){
		return LAZY_BOOLEAN_FUNCTION_and(juxtaposition(a,b));
	}


template<class... PARAM>
inline FUNCTION<LAZY_BOOLEAN,PARAM...> LAZY_BOOLEAN_FUNCTION_or(
	const FUNCTION<std::pair<LAZY_BOOLEAN,LAZY_BOOLEAN>,PARAM... > &a
) {
	return from_algorithm(
		std::function<LAZY_BOOLEAN(const PARAM &...)>(
			[=](const PARAM &... z)
			{
				std::pair<LAZY_BOOLEAN,LAZY_BOOLEAN> r = a(z...);
				return (r.first) || (r.second);
			}
		)
	);
}

template<class... PARAM>
inline FUNCTION<LAZY_BOOLEAN,PARAM...> operator || ( 
	const FUNCTION<LAZY_BOOLEAN,PARAM...> &a,
	const FUNCTION<LAZY_BOOLEAN,PARAM...> &b
	){
		return LAZY_BOOLEAN_FUNCTION_or(juxtaposition(a,b));
	}


template<class... PARAM>
inline FUNCTION<LAZY_BOOLEAN,PARAM...> LAZY_BOOLEAN_FUNCTION_not(
	const FUNCTION<LAZY_BOOLEAN,PARAM...> &a
) {
	// return new FUNCTIONAL_not<PARAM...>(a._fobject);
	return from_algorithm(
		std::function<LAZY_BOOLEAN(const PARAM &...)>(
			[=](const PARAM &... z)
			{
				return !a(z...);
			}
		)
	);
}

template<class... PARAM>
inline FUNCTION<LAZY_BOOLEAN,PARAM...> operator ! ( 
	const FUNCTION<LAZY_BOOLEAN,PARAM...> &a
	){
		return LAZY_BOOLEAN_FUNCTION_not(a);
	}


//********************************************************************************
// Special additional operators for functions
//********************************************************************************


//********************************************************************************
// general limit operator for FUNCTION objects on REAL numbers
//
// REAL limit ( FUNCTION<REAL,int> f )
// if FUNCTION f defines a normed Cauchy sequence, i.e. |f(i)-x|<= 2^{i}
// then limit(f) returns x
//********************************************************************************

REAL limit (const FUNCTION<REAL,int>& f );


} //namespace iRRAM

#endif
